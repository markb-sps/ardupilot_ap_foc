#include "stm32_foc_motor_control.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <hal.h>
#include <string.h>

// The whole driver needs the PWM/ADC HAL (TIM1 + injected ADC). Builds without
// them (e.g. the bootloader) compile this to an empty translation unit.
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && (HAL_USE_PWM == TRUE)

namespace ChibiOS {

namespace {

constexpr uint32_t PHASE_CURRENT_SAMPLE_TIME =
#if defined(ADC_SMPR_SMP_47P5)
    ADC_SMPR_SMP_47P5;
#elif defined(ADC_SMPR_SMP_61P5)
    ADC_SMPR_SMP_61P5;
#else
    0U;  // ADC HAL not built (e.g. bootloader); value is unused there
#endif

// v2 PCB: phase currents come from external INA181A1 amps (gain 20 V/V) on
// plain ADC inputs — no STM32 internal OpAmps. All three amp outputs are
// sampled directly:
//   PHASE_I1 -> PA1 = ADC1_IN2   (phase U, ADC1 injected rank 1)
//   PHASE_I3 -> PB0 = ADC1_IN15  (phase W, ADC1 injected rank 2)
//   PHASE_I2 -> PA7 = ADC2_IN4   (phase V, ADC2 injected rank 1)
// U and V convert simultaneously (ADC1/ADC2 rank 1, both TRGO2-triggered); W
// converts one conversion later on ADC1, so its sample lags U/V by ~1 µs. That
// skew is immaterial to the sum-of-currents check and 2-of-3 reconstruction it
// enables; the control Clarke transform still uses the simultaneous U/V pair.

// TIM1_TRGO2 injected trigger for ADC1/ADC2 on STM32G4: JEXTSEL = 8, rising edge.
// TRGO2 sources OC4REF; with centre-aligned PWM mode 1 the OC4REF level has a
// single rising edge per period (when CNT counts down through CCR4), giving one
// ADC trigger per PWM cycle without needing an auxiliary timer.
constexpr uint32_t PHASE_CURRENT_ADC1_CHANNEL_U = 2U;   // PA1 = ADC1_IN2  (PHASE_I1)
constexpr uint32_t PHASE_CURRENT_ADC1_CHANNEL_W = 15U;  // PB0 = ADC1_IN15 (PHASE_I3)
constexpr uint32_t PHASE_CURRENT_ADC2_CHANNEL   = 4U;   // PA7 = ADC2_IN4  (PHASE_I2)
constexpr uint32_t VBUS_ADC1_CHANNEL            = 1U;   // PA0  = ADC1_IN1
constexpr uint32_t TEMP_ADC1_CHANNEL            = 11U;  // PB12 = ADC1_IN11 (TEMP_ADC net)
constexpr uint32_t PHASE_CURRENT_ADC_JEXTSEL    = 8U;
constexpr uint8_t  PHASE_CURRENT_PENDING_U      = 1U;
constexpr uint8_t  PHASE_CURRENT_PENDING_V      = 2U;
constexpr uint8_t  PHASE_CURRENT_PENDING_W      = 4U;
constexpr uint8_t  PHASE_CURRENT_PENDING_ALL    =
    PHASE_CURRENT_PENDING_U | PHASE_CURRENT_PENDING_V | PHASE_CURRENT_PENDING_W;

const ioline_t PHASE_I1_LINE = PAL_LINE(GPIOA, 1U);  // ADC1_IN2  (U)
const ioline_t PHASE_I2_LINE = PAL_LINE(GPIOA, 7U);  // ADC2_IN4  (V)
const ioline_t PHASE_I3_LINE = PAL_LINE(GPIOB, 0U);  // ADC1_IN15 (W)

struct DriverState {
    bool     initialized              = false;
    bool     current_sense_ok         = false;
    uint16_t period_ticks             = 0;
    uint16_t current_sample_delay_ticks = 1;
    Stm32FocMotorControlCallbacks callbacks{};
    PWMConfig pwm_cfg{};
    volatile uint16_t pending_sample_u = 0U;
    volatile uint16_t pending_sample_v = 0U;
    volatile uint16_t pending_sample_w = 0U;
    volatile uint8_t  pending_mask     = 0U;
    // VBUS + board temp: sampled by ADC1 regular conversions (PA0 / ADC1_IN1
    // and PB12 / ADC1_IN11), opportunistically driven from the ADC1 ISR so the
    // thread side never has to wait on the ADC. The single regular conversion
    // alternates between the two channels, so each updates at ~10 kHz.
    volatile uint16_t vbus_raw         = 0U;
    volatile uint16_t temp_raw         = 0U;
    bool              regular_is_temp  = false;   // which channel converts next
} driver_state;


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

// Program the per-channel sample time. Channels 0..9 live in SMPR1, 10..18 in
// SMPR2 (three bits each).
void set_channel_sample_time(ADC_TypeDef *adc, uint32_t channel)
{
    if (channel < 10U) {
        const uint32_t smp_shift = channel * 3U;
        adc->SMPR1 = (adc->SMPR1 & ~(0x7U << smp_shift)) |
                      (uint32_t(PHASE_CURRENT_SAMPLE_TIME) << smp_shift);
    } else {
        const uint32_t smp_shift = (channel - 10U) * 3U;
        adc->SMPR2 = (adc->SMPR2 & ~(0x7U << smp_shift)) |
                      (uint32_t(PHASE_CURRENT_SAMPLE_TIME) << smp_shift);
    }
}

// Set up one ADC for TRGO2-triggered injected sampling of `nconv` (1 or 2)
// channels. Only the end-of-sequence interrupt (JEOS) is enabled, so a
// two-conversion sequence raises a single IRQ with both JDR1 and JDR2 valid.
void init_adc_unit(ADC_TypeDef *adc, uint32_t ch1, uint32_t ch2, uint8_t nconv)
{
    calibrate_adc(adc);

    adc->CR   = ADC_CR_ADVREGEN;
    adc->ISR  = adc->ISR;  // clear all flags
    set_channel_sample_time(adc, ch1);
    uint32_t jsqr =
        (PHASE_CURRENT_ADC_JEXTSEL << ADC_JSQR_JEXTSEL_Pos)  |  // TIM1_TRGO2 (STM32G4 ADC1/2)
        ADC_JSQR_JEXTEN_0                                    |  // rising edge
        (ch1 << ADC_JSQR_JSQ1_Pos);                             // rank 1 = ch1
    if (nconv >= 2U) {
        set_channel_sample_time(adc, ch2);
        jsqr |= (1U << ADC_JSQR_JL_Pos)     |                   // 2 injected conversions (JL = N-1)
                (ch2 << ADC_JSQR_JSQ2_Pos);                     // rank 2 = ch2
    }
    adc->JSQR = jsqr;
    adc->IER = ADC_IER_JEOSIE;   // fire once per sequence, after the last rank
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

    // INA181 outputs feed plain analog inputs; drive the pins as analog.
    palSetLineMode(PHASE_I1_LINE, PAL_MODE_INPUT_ANALOG);  // PA1 -> ADC1_IN2  (U)
    palSetLineMode(PHASE_I2_LINE, PAL_MODE_INPUT_ANALOG);  // PA7 -> ADC2_IN4  (V)
    palSetLineMode(PHASE_I3_LINE, PAL_MODE_INPUT_ANALOG);  // PB0 -> ADC1_IN15 (W)

    // ADC1 samples U (rank 1) then W (rank 2); ADC2 samples V. U and V start
    // together on the shared TRGO2 trigger, keeping the control pair simultaneous.
    init_adc_unit(ADC1, PHASE_CURRENT_ADC1_CHANNEL_U, PHASE_CURRENT_ADC1_CHANNEL_W, 2U);
    init_adc_unit(ADC2, PHASE_CURRENT_ADC2_CHANNEL, 0U, 1U);

    // Configure ADC1 regular sequence for VBUS (PA0 = ADC1_IN1) and board temp
    // (PB12 = ADC1_IN11, NTC divider) and kick off the first conversion. From
    // here on the ADC1 ISR keeps re-arming a single regular conversion after
    // each result, alternating channels — see motor_control_adc1_irq_hook.
    palSetLineMode(PAL_LINE(GPIOA, 0U), PAL_MODE_INPUT_ANALOG);
    palSetLineMode(PAL_LINE(GPIOB, 12U), PAL_MODE_INPUT_ANALOG);
    ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP1_Msk) |
                   ADC_SMPR1_SMP_AN1(PHASE_CURRENT_SAMPLE_TIME);
    set_channel_sample_time(ADC1, TEMP_ADC1_CHANNEL);
    driver_state.regular_is_temp = false;
    ADC1->SQR1  = (VBUS_ADC1_CHANNEL << ADC_SQR1_SQ1_Pos);   // L=0 (one conv)
    ADC1->IER  |= ADC_IER_EOCIE;              // route EOC into the existing ADC1 ISR
    ADC1->ISR   = ADC_ISR_EOC | ADC_ISR_OVR;
    ADC1->CR   |= ADC_CR_ADSTART;
    return true;
#else
    return false;
#endif
}

// Collects phase samples from the two ADC ISR hooks (ADC1 delivers U+W, ADC2
// delivers V) and fires the callback once all three are available. ADC1 and
// ADC2 raise separate JEOS interrupts, so the pending mask reassembles them
// regardless of arrival order.
void handle_phase_current_sample_isr(uint16_t sample_a, uint16_t sample_b, uint8_t mask)
{
    if (!driver_state.current_sense_ok || driver_state.callbacks.phase_current == nullptr) {
        return;
    }

    if (mask & PHASE_CURRENT_PENDING_U) {
        driver_state.pending_sample_u = sample_a;   // ADC1 JDR1
        driver_state.pending_sample_w = sample_b;   // ADC1 JDR2
    } else {
        driver_state.pending_sample_v = sample_a;   // ADC2 JDR1
    }
    driver_state.pending_mask |= mask;

    if (driver_state.pending_mask == PHASE_CURRENT_PENDING_ALL) {
        driver_state.callbacks.phase_current(driver_state.callbacks.ctx,
                                             driver_state.pending_sample_u,
                                             driver_state.pending_sample_v,
                                             driver_state.pending_sample_w);
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
    driver_state.pending_sample_w = 0U;
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

// Emergency bridge shutdown invoked from the CPU exception handlers (see the
// weak hook in system.cpp). A bare MOE clear = coast (all six FETs off): the
// safe "do no harm" state that holds even if the power stage is already faulted
// and needs no OS/driver state. Gated on the bridge being initialised so builds
// that use TIM1 for something else are untouched.
extern "C" void motor_control_fault_stop(void)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (driver_state.initialized) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
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
    // JEOS = injected sequence complete: both ranks converted, JDR1 (U) and
    // JDR2 (W) are valid. (JEOC after rank 1 is not enabled, so W is never read
    // stale.) The dispatcher already cleared ISR and passed this snapshot.
    if ((isr & ADC_ISR_JEOS) != 0U) {
        handle_phase_current_sample_isr(uint16_t(ADC1->JDR1 & 0xFFFFU),
                                        uint16_t(ADC1->JDR2 & 0xFFFFU),
                                        PHASE_CURRENT_PENDING_U | PHASE_CURRENT_PENDING_W);
    }
    // VBUS/temp opportunistic sample — if the regular conversion completed,
    // latch the result, swap to the other channel and re-arm. Never spins; if
    // EOC isn't set yet we just leave it and pick up next cycle. (Single
    // conversion, so ADSTART has auto-cleared → SQR1 write is legal here.)
    if ((isr & ADC_ISR_EOC) != 0U) {
        const uint16_t dr = uint16_t(ADC1->DR & 0xFFFFU);
        if (driver_state.regular_is_temp) {
            driver_state.temp_raw = dr;
            ADC1->SQR1 = (VBUS_ADC1_CHANNEL << ADC_SQR1_SQ1_Pos);
            driver_state.regular_is_temp = false;
        } else {
            driver_state.vbus_raw = dr;
            ADC1->SQR1 = (TEMP_ADC1_CHANNEL << ADC_SQR1_SQ1_Pos);
            driver_state.regular_is_temp = true;
        }
        ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
        ADC1->CR |= ADC_CR_ADSTART;
    }
}

extern "C" void motor_control_adc2_irq_hook(uint32_t isr)
{
    if ((isr & ADC_ISR_JEOS) == 0U) {
        return;
    }
    handle_phase_current_sample_isr(uint16_t(ADC2->JDR1 & 0xFFFFU), 0U, PHASE_CURRENT_PENDING_V);
}

// VBUS sense: PA0 → ADC1_IN1, divider 357k over 10k, 3.3V Vref, 12-bit.
// Read is a pure cached load — the regular conversion is driven from the
// ADC1 ISR (see motor_control_adc1_irq_hook), so this never blocks the
// thread and never races the injected phase-current trigger.
float stm32_foc_vbus_read_volts()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE
    constexpr float VBUS_SCALE = 3.3f * (357.0f + 10.0f) / (10.0f * 4096.0f);
    if (!driver_state.current_sense_ok) {
        return 0.0f;
    }
    return float(driver_state.vbus_raw) * VBUS_SCALE;
#else
    return 0.0f;
#endif
}

// Board-temp NTC divider (3V3 → 10k → PB12 → NTC 10k B3435 → GND) as a
// fraction of Vref. Pure cached load, same scheme as the VBUS read.
float stm32_foc_temp_read_ratio()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE
    if (!driver_state.current_sense_ok) {
        return 0.0f;
    }
    return float(driver_state.temp_raw) * (1.0f / 4095.0f);
#else
    return 0.0f;
#endif
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
