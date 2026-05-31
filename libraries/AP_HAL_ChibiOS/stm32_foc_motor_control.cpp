#include "stm32_foc_motor_control.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <hal.h>
#include <string.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

namespace ChibiOS {

namespace {

constexpr uint32_t PHASE_CURRENT_SAMPLE_TIME =
#if defined(ADC_SMPR_SMP_47P5)
    ADC_SMPR_SMP_47P5;
#else
    ADC_SMPR_SMP_61P5;
#endif

constexpr uint32_t OPAMP_CSR_VPSEL_VINP0        = 0U;
constexpr uint32_t OPAMP_CSR_PGA_MODE           = OPAMP_CSR_VMSEL_1;
constexpr uint32_t OPAMP_CSR_PGA_EXTERNAL_VINM0 = OPAMP_CSR_PGGAIN_3;
constexpr uint32_t OPAMP_CSR_PGA_GAIN_16        = OPAMP_CSR_PGA_EXTERNAL_VINM0 |
                                                   OPAMP_CSR_PGGAIN_0 |
                                                   OPAMP_CSR_PGGAIN_1;
// NOTE: OPAMPINTEN (OPAINTOEN, bit 8) is intentionally NOT set. On STM32G4 that
// bit routes the OpAmp output to a dedicated *internal* ADC channel and
// DISCONNECTS it from the VOUT pin. This board samples the external pins
// PA2/PA6 = ADC1_IN3/ADC2_IN3 (see hwdef), so the output must reach the pin —
// i.e. OPAINTOEN must be 0. Setting it left IN3 reading an undriven pin (~0).
constexpr uint32_t PHASE_CURRENT_OPAMP_ENABLED_CSR =
    OPAMP_CSR_HIGHSPEEDEN |
    OPAMP_CSR_VPSEL_VINP0 |
    OPAMP_CSR_PGA_MODE    |
    OPAMP_CSR_PGA_GAIN_16 |
    OPAMP_CSR_OPAMPxEN;

// TIM1_TRGO2 injected trigger for ADC1/ADC2 on STM32G4: JEXTSEL = 8, rising edge.
// TRGO2 sources OC4REF; with centre-aligned PWM mode 1 the OC4REF level has a
// single rising edge per period (when CNT counts down through CCR4), giving one
// ADC trigger per PWM cycle without needing an auxiliary timer.
constexpr uint32_t PHASE_CURRENT_ADC_CHANNEL    = ADC_CHANNEL_IN3;
constexpr uint32_t PHASE_CURRENT_ADC_JEXTSEL    = 8U;
constexpr uint8_t  PHASE_CURRENT_PENDING_U      = 1U;
constexpr uint8_t  PHASE_CURRENT_PENDING_V      = 2U;

const ioline_t OPAMP1_VINP_LINE = PAL_LINE(GPIOA, 1U);
const ioline_t OPAMP1_VINM_LINE = PAL_LINE(GPIOA, 3U);
const ioline_t OPAMP1_VOUT_LINE = PAL_LINE(GPIOA, 2U);
const ioline_t OPAMP2_VINM_LINE = PAL_LINE(GPIOA, 5U);
const ioline_t OPAMP2_VOUT_LINE = PAL_LINE(GPIOA, 6U);
const ioline_t OPAMP2_VINP_LINE = PAL_LINE(GPIOA, 7U);
const ioline_t OPAMP3_VINP_LINE = PAL_LINE(GPIOB, 0U);
const ioline_t OPAMP3_VOUT_LINE = PAL_LINE(GPIOB, 1U);
const ioline_t OPAMP3_VINM_LINE = PAL_LINE(GPIOB, 2U);

struct DriverState {
    bool     initialized              = false;
    bool     current_sense_ok         = false;
    uint16_t period_ticks             = 0;
    uint16_t current_sample_delay_ticks = 1;
    Stm32FocMotorControlCallbacks callbacks{};
    PWMConfig pwm_cfg{};
    volatile uint16_t pending_sample_u = 0U;
    volatile uint16_t pending_sample_v = 0U;
    volatile uint8_t  pending_mask     = 0U;
} driver_state;


void init_opamps()
{
#if defined(STM32G4)
    palSetLineMode(OPAMP1_VINP_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP1_VINM_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP1_VOUT_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP2_VINP_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP2_VINM_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP2_VOUT_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP3_VINP_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP3_VINM_LINE, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OPAMP3_VOUT_LINE, PAL_MODE_INPUT_ANALOG);

    OPAMP1->CSR = PHASE_CURRENT_OPAMP_ENABLED_CSR;
    OPAMP2->CSR = PHASE_CURRENT_OPAMP_ENABLED_CSR;
    OPAMP3->CSR = 0U;

    osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));
#endif
}

void init_tim1_adc_trigger(uint16_t period_ticks, uint16_t delay_ticks)
{
    // OC4REF in PWM mode 1: high while CNT < CCR4.  In centre-aligned mode
    // OC4REF has exactly one rising edge per period (when the down-counter
    // crosses CCR4).  Place that edge `delay_ticks` past the peak so the ADC
    // samples while the low-side FETs are still conducting.
    const uint16_t ccr4 = (period_ticks > delay_ticks)
                              ? uint16_t(period_ticks - delay_ticks)
                              : uint16_t((period_ticks > 1U) ? (period_ticks - 1U) : 1U);

    TIM1->CCR4   = ccr4;
    TIM1->CCMR2  = (TIM1->CCMR2 & ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M |
                                    TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE | TIM_CCMR2_OC4CE))
                   | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;            // PWM mode 1
    TIM1->CCER  |= TIM_CCER_CC4E;
    TIM1->CR2    = (TIM1->CR2 & ~TIM_CR2_MMS2)
                   | TIM_CR2_MMS2_0 | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_2;  // TRGO2 = OC4REF
}

void calibrate_adc(ADC_TypeDef *adc)
{
    adc->CR = 0U;
    adc->CR = ADC_CR_ADVREGEN;
    osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));

    // Differential calibration
    adc->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF;
    adc->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF | ADC_CR_ADCAL;
    while ((adc->CR & ADC_CR_ADCAL) != 0U) {}

    osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));

    // Single-ended calibration
    adc->CR = ADC_CR_ADVREGEN;
    adc->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
    while ((adc->CR & ADC_CR_ADCAL) != 0U) {}

    osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));
}

void init_adc_unit(ADC_TypeDef *adc)
{
    calibrate_adc(adc);

    adc->CR   = ADC_CR_ADVREGEN;
    adc->ISR  = adc->ISR;  // clear all flags
    adc->SMPR1 = (adc->SMPR1 & ~ADC_SMPR1_SMP3_Msk) |
                  ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_47P5);
    adc->JSQR =
        (0U << ADC_JSQR_JL_Pos)                              |  // 1 injected conversion
        (PHASE_CURRENT_ADC_CHANNEL << ADC_JSQR_JSQ1_Pos)     |  // rank 1 = IN3
        (PHASE_CURRENT_ADC_JEXTSEL << ADC_JSQR_JEXTSEL_Pos)  |  // TIM4_TRGO (STM32G4 ADC1/2)
        ADC_JSQR_JEXTEN_0;                                      // rising edge
    adc->IER = ADC_IER_JEOCIE | ADC_IER_JEOSIE;
    adc->CR |= ADC_CR_ADEN;
    while ((adc->ISR & ADC_ISR_ADRDY) == 0U) {}

    nvicEnableVector(ADC1_2_IRQn, STM32_ADC_ADC3_IRQ_PRIORITY);
    adc->CR |= ADC_CR_JADSTART;
}

bool init_current_sense()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE && STM32_ADC_USE_ADC2 == TRUE
    rccResetADC12();
    rccEnableADC12(true);
    ADC12_COMMON->CCR = STM32_ADC_ADC12_PRESC | STM32_ADC_ADC12_CLOCK_MODE;
    init_adc_unit(ADC1);
    init_adc_unit(ADC2);
    return true;
#else
    return false;
#endif
}

// Assembles a U+V sample pair from the two separate ADC ISR hooks and fires
// the callback once both are available.
void handle_phase_current_sample_isr(uint16_t sample, uint8_t mask)
{
    if (!driver_state.current_sense_ok || driver_state.callbacks.phase_current == nullptr) {
        return;
    }

    if (mask == PHASE_CURRENT_PENDING_U) {
        driver_state.pending_sample_u = sample;
    } else {
        driver_state.pending_sample_v = sample;
    }
    driver_state.pending_mask |= mask;

    if (driver_state.pending_mask == (PHASE_CURRENT_PENDING_U | PHASE_CURRENT_PENDING_V)) {
        driver_state.callbacks.phase_current(driver_state.callbacks.ctx,
                                             driver_state.pending_sample_u,
                                             driver_state.pending_sample_v);
        driver_state.pending_mask = 0U;
    }
}

} // anonymous namespace

extern "C" void motor_control_adc1_irq_hook(uint32_t isr);
extern "C" void motor_control_adc2_irq_hook(uint32_t isr);

Stm32FocMotorControlInitResult stm32_foc_motor_control_init(const Stm32FocMotorControlSetup &setup,
                                                            const Stm32FocMotorControlCallbacks &callbacks)
{
    Stm32FocMotorControlInitResult result{};

#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (driver_state.initialized) {
        result.ok              = true;
        result.current_sense_ok = driver_state.current_sense_ok;
        result.period_ticks    = driver_state.period_ticks;
        result.update_rate_hz  = setup.pwm_frequency_hz;
        return result;
    }

    if (setup.pwm_clock_hz == 0U || setup.pwm_frequency_hz == 0U) {
        return result;
    }

    memset(&driver_state.pwm_cfg, 0, sizeof(driver_state.pwm_cfg));
    driver_state.callbacks      = callbacks;
    driver_state.pending_sample_u = 0U;
    driver_state.pending_sample_v = 0U;
    driver_state.pending_mask   = 0U;

    driver_state.pwm_cfg.frequency = setup.pwm_clock_hz;
    const uint32_t period_divisor  = setup.center_aligned ?
                                     (2U * setup.pwm_frequency_hz) : setup.pwm_frequency_hz;
    driver_state.pwm_cfg.period    = setup.pwm_clock_hz / period_divisor;
    if (driver_state.pwm_cfg.period == 0U) {
        driver_state.callbacks = {};
        return result;
    }

    driver_state.period_ticks = uint16_t(driver_state.pwm_cfg.period);
    driver_state.current_sample_delay_ticks =
        (setup.current_sample_delay_ticks == 0U) ? 1U : setup.current_sample_delay_ticks;

    // No PWM period callback — all updates are driven from the ADC ISR.
    driver_state.pwm_cfg.callback      = nullptr;
    driver_state.pwm_cfg.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    driver_state.pwm_cfg.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    driver_state.pwm_cfg.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    driver_state.pwm_cfg.channels[3].mode = PWM_OUTPUT_DISABLED;
    driver_state.pwm_cfg.bdtr =
        STM32_TIM_BDTR_DTG(setup.deadtime_ticks) |
        STM32_TIM_BDTR_OSSI |
        STM32_TIM_BDTR_OSSR;
    if (setup.break_input_enabled) {
        driver_state.pwm_cfg.bdtr |= STM32_TIM_BDTR_BKE;
    }

    pwmStart(&PWMD1, &driver_state.pwm_cfg);
    if (setup.center_aligned) {
        TIM1->CR1 &= ~STM32_TIM_CR1_CMS_MASK;
        TIM1->CR1 |=  STM32_TIM_CR1_CMS(1);
    }

    init_opamps();
    init_tim1_adc_trigger(driver_state.period_ticks, driver_state.current_sample_delay_ticks);
    driver_state.current_sense_ok = init_current_sense();

    // Zero duty, outputs gated (MOE cleared)
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CR1 &= ~TIM_CR1_UDIS;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    driver_state.initialized = true;
    result.ok              = true;
    result.current_sense_ok = driver_state.current_sense_ok;
    result.period_ticks    = driver_state.period_ticks;
    result.update_rate_hz  = setup.pwm_frequency_hz;
#else
    (void)setup;
    (void)callbacks;
#endif

    return result;
}

void stm32_foc_motor_control_enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!driver_state.initialized) {
        return;
    }
    osalSysLock();
    TIM1->BDTR = driver_state.pwm_cfg.bdtr | TIM_BDTR_MOE;
    osalSysUnlock();
#endif
}

void stm32_foc_motor_control_disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!driver_state.initialized) {
        return;
    }
    osalSysLock();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    osalSysUnlock();
#endif
}

void stm32_foc_motor_control_disable_outputs_isr()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    // Single register write; safe to call from ISR without an OS lock.
    TIM1->BDTR &= ~TIM_BDTR_MOE;
#endif
}

// Write CCRs atomically using UDIS.  Called from ADC ISR — no syslock.
void stm32_foc_motor_control_write_pwm(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CCR1 = phase_u;
    TIM1->CCR2 = phase_v;
    TIM1->CCR3 = phase_w;
    TIM1->CR1 &= ~TIM_CR1_UDIS;
#else
    (void)phase_u; (void)phase_v; (void)phase_w;
#endif
}

extern "C" void motor_control_adc1_irq_hook(uint32_t isr)
{
    if ((isr & ADC_ISR_JEOC) == 0U) {
        return;
    }
    handle_phase_current_sample_isr(uint16_t(ADC1->JDR1 & 0xFFFFU), PHASE_CURRENT_PENDING_U);
}

extern "C" void motor_control_adc2_irq_hook(uint32_t isr)
{
    if ((isr & ADC_ISR_JEOC) == 0U) {
        return;
    }
    handle_phase_current_sample_isr(uint16_t(ADC2->JDR1 & 0xFFFFU), PHASE_CURRENT_PENDING_V);
}

// VBUS sense: PA0 → ADC1_IN1, divider 357k over 22k, 3.3V Vref, 12-bit.
float stm32_foc_vbus_read_volts()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE
    static bool vbus_inited = false;
    constexpr float VBUS_SCALE = 3.3f * (357.0f + 22.0f) / (22.0f * 4096.0f);

    if (!driver_state.current_sense_ok) {
        return 0.0f;
    }
    if (!vbus_inited) {
        palSetLineMode(PAL_LINE(GPIOA, 0U), PAL_MODE_INPUT_ANALOG);
        ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP1_Msk) |
                       ADC_SMPR1_SMP_AN1(PHASE_CURRENT_SAMPLE_TIME);
        ADC1->SQR1  = (1U << ADC_SQR1_SQ1_Pos);  // L=0 (one conv), SQ1 = channel 1
        vbus_inited = true;
    }
    ADC1->ISR  = ADC_ISR_EOC;
    ADC1->CR  |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
    return float(ADC1->DR & 0xFFFFU) * VBUS_SCALE;
#else
    return 0.0f;
#endif
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
