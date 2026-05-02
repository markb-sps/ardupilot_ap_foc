#pragma once

#include <stdint.h>

namespace ChibiOS {

class MotorControl {
public:
    struct PhaseCurrentSense {
        float u = 0.0f;
        float v = 0.0f;
        float w = 0.0f;
        float rms_u = 0.0f;
        float rms_v = 0.0f;
        float rms_w = 0.0f;
        int32_t raw_u = 0;
        int32_t raw_v = 0;
        int32_t raw_w = 0;
        bool valid = false;
    };

    struct Config {
        uint32_t pwm_clock_hz = 20000000;
        uint32_t pwm_frequency_hz = 20000;
        uint16_t current_sample_delay_ticks = 2;
        uint8_t deadtime_ticks = 40;
        bool center_aligned = true;
        bool break_input_enabled = false;
    };

    MotorControl() = default;

    bool init();
    bool init(const Config &cfg);
    void deinit();

    bool is_initialized() const { return _initialized; }

    void enable_outputs();
    void disable_outputs();

    void set_phase_duty(float phase_u, float phase_v, float phase_w);
    void set_phase_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);
    void set_open_loop_target(float electrical_hz, float modulation, bool reset_phase=false);

    uint16_t period_ticks() const;
    bool current_sense_ready() const { return _current_sense_initialized; }
    PhaseCurrentSense read_phase_current_voltages();
    bool read_phase_current_voltages(PhaseCurrentSense &sense);
    bool read_phase_current_average_raw_voltages(PhaseCurrentSense &sense, bool reset=true);
    bool read_phase_current_average_voltages(PhaseCurrentSense &sense, bool reset=true);
    void set_phase_current_zero_offsets(const PhaseCurrentSense &sense);

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
    volatile uint16_t _phase_current_zero_raw[2]{};
    volatile uint64_t _phase_current_raw_sum[2]{};
    volatile int32_t _phase_current_last_counts[3]{};
    volatile int64_t _phase_current_sum_counts[3]{};
    volatile uint64_t _phase_current_sum_sq_counts[3]{};
    volatile uint32_t _phase_current_sample_count = 0U;
    volatile bool _phase_current_zero_valid = false;
    uint16_t _open_loop_center_ticks = 0;
    uint32_t _pwm_update_rate_hz = 0;
    volatile uint16_t _open_loop_amplitude_ticks = 0;
    volatile uint32_t _open_loop_phase = 0;
    volatile uint32_t _open_loop_phase_step = 0;
};

} // namespace ChibiOS
