#pragma once

#include <stdint.h>

namespace ChibiOS {

class MotorControl {
public:
    struct Config {
        uint32_t pwm_clock_hz = 20000000;
        uint32_t pwm_frequency_hz = 20000;
        uint16_t current_sample_delay_ticks = 2;
        uint8_t deadtime_ticks = 30;
        bool center_aligned = true;
        bool break_input_enabled = false;
    };

    MotorControl() = default;

    bool init();
    bool init(const Config &cfg);
    void deinit();

    bool is_initialized() const { return _initialized; }
    bool zero_valid() const { return _current_zero_valid; }

    void enable_outputs();
    void disable_outputs();

    void set_phase_duty(float phase_u, float phase_v, float phase_w);
    void set_phase_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);
    void set_open_loop_target(float electrical_hz, float modulation, bool reset_phase=false);

    uint16_t period_ticks() const { return _initialized ? _period_ticks : 0U; }
    bool current_sense_ready() const { return _current_sense_initialized; }
    bool get_filtered_phase_volts(float &u, float &v, float &w) const;

private:
    void update_open_loop_isr();
    static void pwm_period_callback(void *ctx);
    static void current_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v);
    void record_phase_current_sample_pair_isr(uint16_t sample_u, uint16_t sample_v);
    uint16_t clamp_width(uint16_t width) const;

    bool _initialized = false;
    bool _current_sense_initialized = false;
    uint16_t _period_ticks = 0;
    uint16_t _phase_ticks[3]{};
    volatile uint16_t _current_zero_raw[2]{};
    volatile uint32_t _zero_accum[2]{};
    volatile uint8_t _zero_count = 0;
    volatile float _filtered_volts[3]{};
    volatile bool _current_zero_valid = false;
    uint32_t _pwm_update_rate_hz = 0;
    volatile float _open_loop_amplitude = 0.0f; // normalized [0,1] = fraction of Vdc/2
    volatile uint32_t _open_loop_phase = 0;
    volatile uint32_t _open_loop_phase_step = 0;
};

} // namespace ChibiOS
