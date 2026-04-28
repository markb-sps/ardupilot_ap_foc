#include "MotorControl.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <string.h>

extern const AP_HAL::HAL& hal;

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS = 3.3f / 4095.0f;
constexpr uint32_t PHASE_CURRENT_SAMPLE_TIME =
#if defined(ADC_SMPR_SMP_47P5)
    ADC_SMPR_SMP_47P5;
#else
    ADC_SMPR_SMP_61P5;
#endif

const ioline_t OPAMP1_VINP_LINE = PAL_LINE(GPIOA, 1U);
const ioline_t OPAMP1_VINM_LINE = PAL_LINE(GPIOA, 3U);
const ioline_t OPAMP1_VOUT_LINE = PAL_LINE(GPIOA, 2U);
const ioline_t OPAMP2_VINM_LINE = PAL_LINE(GPIOA, 5U);
const ioline_t OPAMP2_VOUT_LINE = PAL_LINE(GPIOA, 6U);
const ioline_t OPAMP2_VINP_LINE = PAL_LINE(GPIOA, 7U);
const ioline_t OPAMP3_VINP_LINE = PAL_LINE(GPIOB, 0U);
const ioline_t OPAMP3_VOUT_LINE = PAL_LINE(GPIOB, 1U);
const ioline_t OPAMP3_VINM_LINE = PAL_LINE(GPIOB, 2U);

const ADCConversionGroup phase_current_adc1_group = {
    .circular = false,
    .num_channels = 2U,
    .end_cb = nullptr,
    .error_cb = nullptr,
    .cfgr = 0U,
    .cfgr2 = 0U,
    .tr1 = ADC_TR_DISABLED,
    .tr2 = ADC_TR_DISABLED,
    .tr3 = ADC_TR_DISABLED,
    .awd2cr = 0U,
    .awd3cr = 0U,
    .smpr = {
        ADC_SMPR1_SMP_AN3(PHASE_CURRENT_SAMPLE_TIME),
        ADC_SMPR2_SMP_AN12(PHASE_CURRENT_SAMPLE_TIME)
    },
    .sqr = {
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN12),
        0U,
        0U,
        0U
    }
};

const ADCConversionGroup phase_current_adc2_group = {
    .circular = false,
    .num_channels = 1U,
    .end_cb = nullptr,
    .error_cb = nullptr,
    .cfgr = 0U,
    .cfgr2 = 0U,
    .tr1 = ADC_TR_DISABLED,
    .tr2 = ADC_TR_DISABLED,
    .tr3 = ADC_TR_DISABLED,
    .awd2cr = 0U,
    .awd3cr = 0U,
    .smpr = {
        ADC_SMPR1_SMP_AN3(PHASE_CURRENT_SAMPLE_TIME),
        0U
    },
    .sqr = {
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3),
        0U,
        0U,
        0U
    }
};

}

bool MotorControl::init()
{
    return init(Config{});
}

bool MotorControl::init(const Config &cfg)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (_initialized) {
        return true;
    }
    if (cfg.pwm_clock_hz == 0 || cfg.pwm_frequency_hz == 0) {
        return false;
    }

    memset(&_pwm_cfg, 0, sizeof(_pwm_cfg));
    _center_aligned = cfg.center_aligned;
    _deadtime_ticks = cfg.deadtime_ticks;
    _phase_ticks[0] = 0;
    _phase_ticks[1] = 0;
    _phase_ticks[2] = 0;

    _driver = &PWMD1;
    _pwm_cfg.frequency = cfg.pwm_clock_hz;
    const uint32_t period_divisor = cfg.center_aligned ? (2U * cfg.pwm_frequency_hz) : cfg.pwm_frequency_hz;
    _pwm_cfg.period = cfg.pwm_clock_hz / period_divisor;
    if (_pwm_cfg.period == 0) {
        return false;
    }

    _pwm_cfg.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    _pwm_cfg.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    _pwm_cfg.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
    _pwm_cfg.channels[3].mode = PWM_OUTPUT_DISABLED;

    _pwm_cfg.bdtr = STM32_TIM_BDTR_DTG(cfg.deadtime_ticks) |
                    STM32_TIM_BDTR_OSSI |
                    STM32_TIM_BDTR_OSSR;
    if (cfg.break_input_enabled) {
        _pwm_cfg.bdtr |= STM32_TIM_BDTR_BKE;
    }

    pwmStart(_driver, &_pwm_cfg);

    if (cfg.center_aligned) {
        _driver->tim->CR1 &= ~STM32_TIM_CR1_CMS_MASK;
        _driver->tim->CR1 |= STM32_TIM_CR1_CMS(1);
    }

    init_opamps();
    _current_sense_initialized = init_current_sense();
    set_phase_duty_ticks(0, 0, 0);
    disable_outputs();

    _initialized = true;
    return true;
#else
    (void)cfg;
    return false;
#endif
}

void MotorControl::deinit()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _driver == nullptr) {
        return;
    }
    pwmStop(_driver);
    _driver = nullptr;
    if (_adc1_started) {
        adcStop(&ADCD1);
        _adc1_started = false;
    }
#if STM32_ADC_USE_ADC2 == TRUE
    if (_adc2_started) {
        adcStop(&ADCD2);
        _adc2_started = false;
    }
#endif
    _current_sense_initialized = false;
    _initialized = false;
#endif
}

void MotorControl::enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _driver == nullptr) {
        return;
    }
    osalSysLock();
    _driver->tim->BDTR = _pwm_cfg.bdtr | STM32_TIM_BDTR_MOE;
    osalSysUnlock();
#endif
}

void MotorControl::disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _driver == nullptr) {
        return;
    }
    osalSysLock();
    _driver->tim->BDTR &= ~STM32_TIM_BDTR_MOE;
    osalSysUnlock();
#endif
}

void MotorControl::set_phase_duty(float phase_u, float phase_v, float phase_w)
{
    if (!_initialized) {
        return;
    }

    const auto period = period_ticks();
    const auto scale = [period](float duty) -> uint16_t {
        if (duty <= 0.0f) {
            return 0;
        }
        if (duty >= 1.0f) {
            return period;
        }
        return uint16_t(duty * period);
    };

    set_phase_duty_ticks(scale(phase_u), scale(phase_v), scale(phase_w));
}

void MotorControl::set_phase_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _driver == nullptr) {
        return;
    }

    _phase_ticks[0] = clamp_width(phase_u);
    _phase_ticks[1] = clamp_width(phase_v);
    _phase_ticks[2] = clamp_width(phase_w);

    osalSysLock();
    pwmEnableChannelI(_driver, 0, _phase_ticks[0]);
    pwmEnableChannelI(_driver, 1, _phase_ticks[1]);
    pwmEnableChannelI(_driver, 2, _phase_ticks[2]);
    osalSysUnlock();
#else
    (void)phase_u;
    (void)phase_v;
    (void)phase_w;
#endif
}

uint16_t MotorControl::period_ticks() const
{
    if (!_initialized) {
        return 0;
    }
    return _pwm_cfg.period;
}

MotorControl::PhaseCurrentSense MotorControl::read_phase_current_voltages()
{
    PhaseCurrentSense sense;
    uint16_t phase_u = 0;
    uint16_t phase_v = 0;
    uint16_t phase_w = 0;
    if (!sample_phase_current_counts(phase_u, phase_v, phase_w)) {
        return sense;
    }

    sense.u = float(phase_u) * ADC_LSB_VOLTS;
    sense.v = float(phase_v) * ADC_LSB_VOLTS;
    sense.w = float(phase_w) * ADC_LSB_VOLTS;
    return sense;
}

uint16_t MotorControl::clamp_width(uint16_t width) const
{
    if (width > _pwm_cfg.period) {
        return _pwm_cfg.period;
    }
    return width;
}

void MotorControl::init_opamps()
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

    // The board uses the first external VP/VM pair on each internal opamp.
    OPAMP1->CSR = OPAMP_CSR_HIGHSPEEDEN | OPAMP_CSR_OPAMPINTEN | OPAMP_CSR_OPAMPxEN;
    OPAMP2->CSR = OPAMP_CSR_HIGHSPEEDEN | OPAMP_CSR_OPAMPINTEN | OPAMP_CSR_OPAMPxEN;
    OPAMP3->CSR = OPAMP_CSR_HIGHSPEEDEN | OPAMP_CSR_OPAMPINTEN | OPAMP_CSR_OPAMPxEN;

    hal.scheduler->delay_microseconds(20);
#endif
}

bool MotorControl::init_current_sense()
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE && STM32_ADC_USE_ADC2 == TRUE
    _adc_cfg.difsel = 0U;

    if (adcStart(&ADCD1, &_adc_cfg) != HAL_RET_SUCCESS) {
        return false;
    }
    _adc1_started = true;

    if (adcStart(&ADCD2, &_adc_cfg) != HAL_RET_SUCCESS) {
        adcStop(&ADCD1);
        _adc1_started = false;
        return false;
    }
    _adc2_started = true;
    return true;
#else
    return false;
#endif
}

bool MotorControl::sample_phase_current_counts(uint16_t &phase_u, uint16_t &phase_v, uint16_t &phase_w)
{
#if HAL_USE_ADC == TRUE && STM32_ADC_USE_ADC1 == TRUE && STM32_ADC_USE_ADC2 == TRUE
    if (!_current_sense_initialized) {
        return false;
    }

    if (!wait_for_low_side_window(_phase_ticks[0])) {
        return false;
    }
    msg_t adc1_result = adcConvert(&ADCD1, &phase_current_adc1_group, _adc1_samples, 1);
    if (adc1_result != MSG_OK) {
        return false;
    }

    msg_t adc2_result = adcConvert(&ADCD2, &phase_current_adc2_group, _adc2_samples, 1);
    if (adc2_result != MSG_OK) {
        return false;
    }
    phase_u = uint16_t(_adc1_samples[0]);

    if (!wait_for_low_side_window(_phase_ticks[1])) {
        return false;
    }
    adc1_result = adcConvert(&ADCD1, &phase_current_adc1_group, _adc1_samples, 1);
    if (adc1_result != MSG_OK) {
        return false;
    }
    adc2_result = adcConvert(&ADCD2, &phase_current_adc2_group, _adc2_samples, 1);
    if (adc2_result != MSG_OK) {
        return false;
    }
    phase_v = uint16_t(_adc2_samples[0]);

    if (!wait_for_low_side_window(_phase_ticks[2])) {
        return false;
    }
    adc1_result = adcConvert(&ADCD1, &phase_current_adc1_group, _adc1_samples, 1);
    if (adc1_result != MSG_OK) {
        return false;
    }
    adc2_result = adcConvert(&ADCD2, &phase_current_adc2_group, _adc2_samples, 1);
    if (adc2_result != MSG_OK) {
        return false;
    }
    phase_w = uint16_t(_adc1_samples[1]);

    return true;
#else
    (void)phase_u;
    (void)phase_v;
    (void)phase_w;
    return false;
#endif
}

bool MotorControl::wait_for_low_side_window(uint16_t phase_width_ticks)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _driver == nullptr) {
        return false;
    }

    const uint16_t period = _pwm_cfg.period;
    if (period == 0 || phase_width_ticks >= period) {
        return false;
    }

    const uint16_t deadtime_margin = uint16_t(_deadtime_ticks) + 2U;
    const uint16_t margin = deadtime_margin > 4U ? deadtime_margin : 4U;
    if (phase_width_ticks + margin >= period) {
        return false;
    }

    const uint16_t target = phase_width_ticks + ((period - phase_width_ticks) / 2U);
    const uint32_t cycle_ticks = _center_aligned ? uint32_t(period) * 2U : period;
    const uint32_t cycle_us_calc = (uint64_t(cycle_ticks) * 1000000ULL) / _pwm_cfg.frequency;
    const uint32_t cycle_us = cycle_us_calc > 0U ? cycle_us_calc : 1U;
    const uint32_t start_us = AP_HAL::micros();
    bool target_armed = false;

    while ((AP_HAL::micros() - start_us) < (cycle_us * 3U)) {
        const bool counting_down = (_driver->tim->CR1 & STM32_TIM_CR1_DIR) != 0;
        const uint16_t counter = uint16_t(_driver->tim->CNT);

        if (counting_down || counter < target) {
            target_armed = true;
        }
        if (target_armed && !counting_down && counter >= target) {
            return true;
        }
    }

    return false;
#else
    (void)phase_width_ticks;
    return false;
#endif
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
