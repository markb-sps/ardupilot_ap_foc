#include "MotorControl.h"
#include "stm32_foc_motor_control.h"
#include "foc_transforms.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <math.h>

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS  = 3.3f / 4095.0f;
constexpr float IIR_CURRENT_ALPHA = 0.05f;
constexpr uint8_t ZERO_CAP_LOG2 = 6; // 2^6 = 64 samples
// Maps uint32 phase accumulator [0, 2^32) → angle [0, 2π)
constexpr float PHASE_TO_RAD = 2.0f * 3.14159265358979f / 4294967296.0f;
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
    if (cfg.pwm_clock_hz == 0U || cfg.pwm_frequency_hz == 0U) {
        return false;
    }

    _current_zero_raw[0] = 0U;
    _current_zero_raw[1] = 0U;
    _zero_accum[0] = 0U;
    _zero_accum[1] = 0U;
    _zero_count = 0;
    _filtered_volts[0] = 0.0f;
    _filtered_volts[1] = 0.0f;
    _filtered_volts[2] = 0.0f;
    _current_zero_valid = false;

    Stm32FocMotorControlSetup setup{};
    setup.pwm_clock_hz = cfg.pwm_clock_hz;
    setup.pwm_frequency_hz = cfg.pwm_frequency_hz;
    setup.current_sample_delay_ticks = cfg.current_sample_delay_ticks;
    setup.deadtime_ticks = cfg.deadtime_ticks;
    setup.center_aligned = cfg.center_aligned;
    setup.break_input_enabled = cfg.break_input_enabled;

    Stm32FocMotorControlCallbacks callbacks{};
    callbacks.ctx = this;
    callbacks.pwm_period = pwm_period_callback;
    callbacks.phase_current = current_sample_callback;

    const auto init_result = stm32_foc_motor_control_init(setup, callbacks);
    if (!init_result.ok) {
        return false;
    }
    _period_ticks = init_result.period_ticks;
    _pwm_update_rate_hz = init_result.update_rate_hz;
    _current_sense_initialized = init_result.current_sense_ok;

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
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_deinit();
    _current_sense_initialized = false;
    _current_zero_valid = false;
    _period_ticks = 0U;
    _pwm_update_rate_hz = 0U;
    _initialized = false;
#endif
}

void MotorControl::set_open_loop_target(float electrical_hz, float modulation, bool reset_phase)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _pwm_update_rate_hz == 0U) {
        return;
    }

    _open_loop_amplitude = (modulation <= 0.0f) ? 0.0f : (modulation > 1.0f ? 1.0f : modulation);

    if (reset_phase) {
        _open_loop_phase = 0;
    }
    if (electrical_hz <= 0.0f) {
        _open_loop_phase_step = 0;
        return;
    }

    const uint64_t step = uint64_t(electrical_hz * (4294967296.0 / double(_pwm_update_rate_hz)));
    _open_loop_phase_step = uint32_t(step);
#else
    (void)electrical_hz;
    (void)modulation;
    (void)reset_phase;
#endif
}

void MotorControl::enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_enable_outputs();
#endif
}

void MotorControl::disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_disable_outputs();
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
    if (!_initialized) {
        return;
    }

    _phase_ticks[0] = clamp_width(phase_u);
    _phase_ticks[1] = clamp_width(phase_v);
    _phase_ticks[2] = clamp_width(phase_w);
    stm32_foc_motor_control_set_phase_ticks(_phase_ticks[0], _phase_ticks[1], _phase_ticks[2]);
#else
    (void)phase_u;
    (void)phase_v;
    (void)phase_w;
#endif
}

bool MotorControl::get_filtered_phase_volts(float &u, float &v, float &w) const
{
    if (!_current_zero_valid) {
        return false;
    }
    osalSysLock();
    u = _filtered_volts[0];
    v = _filtered_volts[1];
    w = _filtered_volts[2];
    osalSysUnlock();
    return true;
}

uint16_t MotorControl::clamp_width(uint16_t width) const
{
    if (width > _period_ticks) {
        return _period_ticks;
    }
    return width;
}

void MotorControl::pwm_period_callback(void *ctx)
{
    auto *motor = static_cast<MotorControl *>(ctx);
    if (motor == nullptr) {
        return;
    }
    motor->update_open_loop_isr();
}

void MotorControl::current_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v)
{
    auto *motor = static_cast<MotorControl *>(ctx);
    if (motor == nullptr) {
        return;
    }
    motor->record_phase_current_sample_pair_isr(sample_u, sample_v);
}

void MotorControl::update_open_loop_isr()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    const float theta = float(_open_loop_phase) * PHASE_TO_RAD;
    const float amplitude = _open_loop_amplitude;
    const float sin_t = sinf(theta);
    const float cos_t = cosf(theta);

    float va, vb, vc;
    FOC::inv_clarke(amplitude * cos_t, amplitude * sin_t, va, vb, vc);

    float da, db, dc;
    FOC::svpwm(va, vb, vc, da, db, dc);

    const uint16_t period = _period_ticks;
    const uint16_t pu = uint16_t(da * float(period));
    const uint16_t pv = uint16_t(db * float(period));
    const uint16_t pw = uint16_t(dc * float(period));

    _phase_ticks[0] = pu;
    _phase_ticks[1] = pv;
    _phase_ticks[2] = pw;
    stm32_foc_motor_control_set_phase_ticks_isr(pu, pv, pw);
    _open_loop_phase += _open_loop_phase_step;
#endif
}

void MotorControl::record_phase_current_sample_pair_isr(uint16_t sample_u, uint16_t sample_v)
{
    if (!_current_zero_valid) {
        _zero_accum[0] += sample_u;
        _zero_accum[1] += sample_v;
        if (++_zero_count == (1U << ZERO_CAP_LOG2)) {
            _current_zero_raw[0] = uint16_t(_zero_accum[0] >> ZERO_CAP_LOG2);
            _current_zero_raw[1] = uint16_t(_zero_accum[1] >> ZERO_CAP_LOG2);
            _current_zero_valid = true;
        }
        return;
    }

    const float u = float(int32_t(sample_u) - int32_t(_current_zero_raw[0])) * ADC_LSB_VOLTS;
    const float v = float(int32_t(sample_v) - int32_t(_current_zero_raw[1])) * ADC_LSB_VOLTS;
    const float w = -(u + v);

    _filtered_volts[0] += IIR_CURRENT_ALPHA * (u - _filtered_volts[0]);
    _filtered_volts[1] += IIR_CURRENT_ALPHA * (v - _filtered_volts[1]);
    _filtered_volts[2] += IIR_CURRENT_ALPHA * (w - _filtered_volts[2]);
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
