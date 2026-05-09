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
constexpr uint32_t PHASE_CURRENT_OPAMP_ENABLED_CSR =
    OPAMP_CSR_HIGHSPEEDEN |
    OPAMP_CSR_VPSEL_VINP0 |
    OPAMP_CSR_OPAMPINTEN  |
    OPAMP_CSR_PGA_MODE    |
    OPAMP_CSR_PGA_GAIN_16 |
    OPAMP_CSR_OPAMPxEN;

// TIM4_TRGO injected trigger for ADC1/ADC2 on STM32G4: JEXTSEL = 5, rising edge.
// (TIM4_CC4 itself is wired only to ADC3/4/5 on G4; routing CC4 through TIM4_TRGO
// via MMS = OC4REF gives ADC1/ADC2 the same delayed trigger.)
constexpr uint32_t PHASE_CURRENT_ADC_CHANNEL    = ADC_CHANNEL_IN3;
constexpr uint32_t PHASE_CURRENT_ADC_JEXTSEL    = 5U;
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

void deinit_opamps()
{
#if defined(STM32G4)
    OPAMP1->CSR = 0U;
    OPAMP2->CSR = 0U;
    OPAMP3->CSR = 0U;
#endif
}

void init_tim4_trigger(uint16_t period_ticks, uint16_t delay_ticks)
{
    rccEnableTIM4(true);
    rccResetTIM4();

    TIM4->CR1   = 0U;
    TIM4->CR2   = 0U;
    TIM4->SMCR  = 0U;
    TIM4->DIER  = 0U;
    TIM4->CCER  = 0U;
    TIM4->CCMR2 = 0U;
    TIM4->CNT   = 0U;
    TIM4->PSC   = 0U;
    TIM4->ARR   = 0xFFFFU;
    TIM4->CCR4  = (delay_ticks == 0U) ? 1U : delay_ticks;

    if (period_ticks > 1U && TIM4->CCR4 >= period_ticks) {
        TIM4->CCR4 = period_ticks - 1U;
    }

    // PWM mode 2 on OC4 with preload: OC4REF goes high at CNT == CCR4, giving
    // a rising edge after `delay_ticks` from the TIM1-driven counter reset.
    TIM4->CCMR2 = TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
    TIM4->CCER  = TIM_CCER_CC4E;

    // TRGO follows OC4REF so ADC1/ADC2 (which can't see TIM4_CC4 directly on
    // STM32G4) get the delayed trigger via TIM4_TRGO.
    TIM4->CR2  = STM32_TIM_CR2_MMS(7U);
    // TIM4 slaves off TIM1 ITR0 in reset mode
    TIM4->SMCR = TIM_SMCR_SMS_2;
    TIM4->CR1  = TIM_CR1_ARPE;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void deinit_tim4_trigger()
{
    TIM4->CR1  = 0U;
    TIM4->DIER = 0U;
    TIM4->CCER = 0U;
    rccDisableTIM4();
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

void stop_adc_unit(ADC_TypeDef *adc)
{
    adc->IER = 0U;
    if ((adc->CR & ADC_CR_ADSTART) != 0U) {
        adc->CR |= ADC_CR_ADSTP;
        while ((adc->CR & ADC_CR_ADSTP) != 0U) {}
    }
    if ((adc->CR & ADC_CR_ADEN) != 0U) {
        adc->CR |= ADC_CR_ADDIS;
        while ((adc->CR & ADC_CR_ADEN) != 0U) {}
    }
    adc->CR = 0U;
    adc->CR = ADC_CR_DEEPPWD;
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

void deinit_current_sense()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE && STM32_ADC_USE_ADC2 == TRUE
    stop_adc_unit(ADC1);
    stop_adc_unit(ADC2);
    rccDisableADC12();
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

    // TIM1 TRGO = Update event; enable master sync for TIM4
    TIM1->CR2  &= ~TIM_CR2_MMS;
    TIM1->CR2  |= STM32_TIM_CR2_MMS(2U);
    TIM1->SMCR |= TIM_SMCR_MSM;

    init_opamps();
    init_tim4_trigger(driver_state.period_ticks, driver_state.current_sample_delay_ticks);
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

void stm32_foc_motor_control_deinit()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!driver_state.initialized) {
        return;
    }
    stm32_foc_motor_control_disable_outputs();
    pwmStop(&PWMD1);
    deinit_tim4_trigger();
    deinit_current_sense();
    deinit_opamps();
    driver_state = {};
#endif
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

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
