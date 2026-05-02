#include "MotorControl.h"
#include "stm32_foc_motor_control.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <math.h>

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS = 3.3f / 4095.0f;
constexpr uint32_t OPEN_LOOP_PHASE_OFFSET_120 = 1431655765U;
constexpr uint32_t OPEN_LOOP_PHASE_OFFSET_240 = 2863311530U;
constexpr uint8_t OPEN_LOOP_SINE_BITS = 5;
constexpr uint8_t OPEN_LOOP_SINE_SIZE = 1U << OPEN_LOOP_SINE_BITS;
const int8_t open_loop_sine_lut[OPEN_LOOP_SINE_SIZE] = {
    0, 25, 49, 71, 90, 106, 117, 125,
    127, 125, 117, 106, 90, 71, 49, 25,
    0, -25, -49, -71, -90, -106, -117, -125,
    -127, -125, -117, -106, -90, -71, -49, -25
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
    if (cfg.pwm_clock_hz == 0U || cfg.pwm_frequency_hz == 0U) {
        return false;
    }

    _period_ticks = 0U;
    _phase_ticks[0] = 0;
    _phase_ticks[1] = 0;
    _phase_ticks[2] = 0;
    _pwm_update_rate_hz = 0U;
    _open_loop_amplitude_ticks = 0;
    _open_loop_phase = 0;
    _open_loop_phase_step = 0;
    _phase_current_zero_raw[0] = 0U;
    _phase_current_zero_raw[1] = 0U;
    _phase_current_raw_sum[0] = 0U;
    _phase_current_raw_sum[1] = 0U;
    _phase_current_last_counts[0] = 0;
    _phase_current_last_counts[1] = 0;
    _phase_current_last_counts[2] = 0;
    _phase_current_sum_counts[0] = 0;
    _phase_current_sum_counts[1] = 0;
    _phase_current_sum_counts[2] = 0;
    _phase_current_sum_sq_counts[0] = 0U;
    _phase_current_sum_sq_counts[1] = 0U;
    _phase_current_sum_sq_counts[2] = 0U;
    _phase_current_sample_count = 0U;
    _phase_current_zero_valid = false;

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
    _open_loop_center_ticks = uint16_t(_period_ticks / 2U);
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
    _period_ticks = 0U;
    _pwm_update_rate_hz = 0U;
    _initialized = false;
#endif
}

void MotorControl::set_open_loop_target(float electrical_hz, float modulation, bool reset_phase)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _period_ticks == 0U || _pwm_update_rate_hz == 0U) {
        return;
    }

    if (modulation <= 0.0f) {
        _open_loop_amplitude_ticks = 0;
    } else {
        const float max_amplitude = float(_open_loop_center_ticks);
        float scaled = modulation * 0.5f * float(_period_ticks);
        if (scaled > max_amplitude) {
            scaled = max_amplitude;
        }
        _open_loop_amplitude_ticks = uint16_t(scaled);
    }

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

uint16_t MotorControl::period_ticks() const
{
    return _initialized ? _period_ticks : 0U;
}

MotorControl::PhaseCurrentSense MotorControl::read_phase_current_voltages()
{
    PhaseCurrentSense sense;
    read_phase_current_voltages(sense);
    return sense;
}

bool MotorControl::read_phase_current_voltages(PhaseCurrentSense &sense)
{
    sense = {};
    int32_t counts[3];
    bool zero_valid = false;
    uint32_t sample_count = 0U;

    osalSysLock();
    counts[0] = _phase_current_last_counts[0];
    counts[1] = _phase_current_last_counts[1];
    counts[2] = _phase_current_last_counts[2];
    zero_valid = _phase_current_zero_valid;
    sample_count = _phase_current_sample_count;
    osalSysUnlock();

    if (!zero_valid || sample_count == 0U) {
        return false;
    }

    sense.u = float(counts[0]) * ADC_LSB_VOLTS;
    sense.v = float(counts[1]) * ADC_LSB_VOLTS;
    sense.w = float(counts[2]) * ADC_LSB_VOLTS;
    sense.raw_u = counts[0];
    sense.raw_v = counts[1];
    sense.raw_w = counts[2];
    sense.valid = true;
    return true;
}

bool MotorControl::read_phase_current_average_raw_voltages(PhaseCurrentSense &sense, bool reset)
{
    sense = {};
    uint64_t sum_raw[2];
    uint32_t sample_count = 0U;
    bool zero_valid = false;

    osalSysLock();
    sum_raw[0] = _phase_current_raw_sum[0];
    sum_raw[1] = _phase_current_raw_sum[1];
    sample_count = _phase_current_sample_count;
    zero_valid = _phase_current_zero_valid;
    const bool ready = !zero_valid && sample_count > 0U;
    if (ready && reset) {
        _phase_current_raw_sum[0] = 0U;
        _phase_current_raw_sum[1] = 0U;
        _phase_current_sample_count = 0U;
    }
    osalSysUnlock();

    if (!ready) {
        return false;
    }

    sense.raw_u = int32_t((sum_raw[0] + (sample_count / 2U)) / sample_count);
    sense.raw_v = int32_t((sum_raw[1] + (sample_count / 2U)) / sample_count);
    sense.raw_w = 0;
    sense.u = float(sum_raw[0]) * ADC_LSB_VOLTS / float(sample_count);
    sense.v = float(sum_raw[1]) * ADC_LSB_VOLTS / float(sample_count);
    sense.w = 0.0f;
    sense.valid = true;
    return true;
}

bool MotorControl::read_phase_current_average_voltages(PhaseCurrentSense &sense, bool reset)
{
    sense = {};
    int64_t sum_counts[3];
    uint64_t sum_sq_counts[3];
    uint32_t sample_count = 0U;
    bool zero_valid = false;

    osalSysLock();
    sum_counts[0] = _phase_current_sum_counts[0];
    sum_counts[1] = _phase_current_sum_counts[1];
    sum_counts[2] = _phase_current_sum_counts[2];
    sum_sq_counts[0] = _phase_current_sum_sq_counts[0];
    sum_sq_counts[1] = _phase_current_sum_sq_counts[1];
    sum_sq_counts[2] = _phase_current_sum_sq_counts[2];
    sample_count = _phase_current_sample_count;
    zero_valid = _phase_current_zero_valid;
    const bool ready = zero_valid && sample_count > 0U;
    if (ready && reset) {
        _phase_current_sum_counts[0] = 0;
        _phase_current_sum_counts[1] = 0;
        _phase_current_sum_counts[2] = 0;
        _phase_current_sum_sq_counts[0] = 0U;
        _phase_current_sum_sq_counts[1] = 0U;
        _phase_current_sum_sq_counts[2] = 0U;
        _phase_current_sample_count = 0U;
    }
    osalSysUnlock();

    if (!ready) {
        return false;
    }

    sense.raw_u = int32_t((sum_counts[0] + (sum_counts[0] >= 0 ? int64_t(sample_count / 2U) : -int64_t(sample_count / 2U))) / int64_t(sample_count));
    sense.raw_v = int32_t((sum_counts[1] + (sum_counts[1] >= 0 ? int64_t(sample_count / 2U) : -int64_t(sample_count / 2U))) / int64_t(sample_count));
    sense.raw_w = int32_t((sum_counts[2] + (sum_counts[2] >= 0 ? int64_t(sample_count / 2U) : -int64_t(sample_count / 2U))) / int64_t(sample_count));
    sense.u = float(sum_counts[0]) * ADC_LSB_VOLTS / float(sample_count);
    sense.v = float(sum_counts[1]) * ADC_LSB_VOLTS / float(sample_count);
    sense.w = float(sum_counts[2]) * ADC_LSB_VOLTS / float(sample_count);
    sense.rms_u = sqrtf(float(double(sum_sq_counts[0]) / double(sample_count))) * ADC_LSB_VOLTS;
    sense.rms_v = sqrtf(float(double(sum_sq_counts[1]) / double(sample_count))) * ADC_LSB_VOLTS;
    sense.rms_w = sqrtf(float(double(sum_sq_counts[2]) / double(sample_count))) * ADC_LSB_VOLTS;
    sense.valid = true;
    return true;
}

void MotorControl::set_phase_current_zero_offsets(const PhaseCurrentSense &sense)
{
    osalSysLock();
    _phase_current_zero_raw[0] = uint16_t(sense.raw_u);
    _phase_current_zero_raw[1] = uint16_t(sense.raw_v);
    _phase_current_sum_counts[0] = 0;
    _phase_current_sum_counts[1] = 0;
    _phase_current_sum_counts[2] = 0;
    _phase_current_sum_sq_counts[0] = 0U;
    _phase_current_sum_sq_counts[1] = 0U;
    _phase_current_sum_sq_counts[2] = 0U;
    _phase_current_sample_count = 0U;
    _phase_current_zero_valid = true;
    osalSysUnlock();
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

// NOTE: Called from interrupt.
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
    const uint16_t amplitude = _open_loop_amplitude_ticks;
    const uint16_t center = _open_loop_center_ticks;
    const auto phase_to_width = [amplitude, center](uint32_t phase) -> uint16_t {
        const uint8_t index = uint8_t(phase >> (32U - OPEN_LOOP_SINE_BITS));
        const int32_t scaled = int32_t(amplitude) * int32_t(open_loop_sine_lut[index]);
        return uint16_t(int32_t(center) + scaled / 127);
    };

    const uint16_t phase_u = phase_to_width(_open_loop_phase);
    const uint16_t phase_v = phase_to_width(_open_loop_phase - OPEN_LOOP_PHASE_OFFSET_120);
    const uint16_t phase_w = phase_to_width(_open_loop_phase - OPEN_LOOP_PHASE_OFFSET_240);

    _phase_ticks[0] = phase_u;
    _phase_ticks[1] = phase_v;
    _phase_ticks[2] = phase_w;
    stm32_foc_motor_control_set_phase_ticks_isr(phase_u, phase_v, phase_w);
    _open_loop_phase += _open_loop_phase_step;
#endif
}

void MotorControl::record_phase_current_sample_pair_isr(uint16_t sample_u, uint16_t sample_v)
{
    if (!_phase_current_zero_valid) {
        _phase_current_raw_sum[0] += sample_u;
        _phase_current_raw_sum[1] += sample_v;
        _phase_current_sample_count++;
        return;
    }

    const int32_t count_u = int32_t(sample_u) - int32_t(_phase_current_zero_raw[0]);
    const int32_t count_v = int32_t(sample_v) - int32_t(_phase_current_zero_raw[1]);
    const int32_t count_w = -(count_u + count_v);

    _phase_current_last_counts[0] = count_u;
    _phase_current_last_counts[1] = count_v;
    _phase_current_last_counts[2] = count_w;

    _phase_current_sum_counts[0] += count_u;
    _phase_current_sum_counts[1] += count_v;
    _phase_current_sum_counts[2] += count_w;

    _phase_current_sum_sq_counts[0] += uint64_t(count_u * count_u);
    _phase_current_sum_sq_counts[1] += uint64_t(count_v * count_v);
    _phase_current_sum_sq_counts[2] += uint64_t(count_w * count_w);
    _phase_current_sample_count++;
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
