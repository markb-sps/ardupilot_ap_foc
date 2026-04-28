#pragma once

#include <hal.h>

namespace ChibiOS {

class MotorControl {
public:
    struct PhaseCurrentSense {
        float u = 0.0f;
        float v = 0.0f;
        float w = 0.0f;
    };

    struct Config {
        uint32_t pwm_clock_hz = 20000000;
        uint32_t pwm_frequency_hz = 20000;
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

    uint16_t period_ticks() const;
    bool current_sense_ready() const { return _current_sense_initialized; }
    PhaseCurrentSense read_phase_current_voltages();

private:
    void init_opamps();
    bool init_current_sense();
    bool sample_phase_current_counts(uint16_t &phase_u, uint16_t &phase_v, uint16_t &phase_w);
    bool wait_for_low_side_window(uint16_t phase_width_ticks);
    uint16_t clamp_width(uint16_t width) const;

    bool _initialized = false;
    bool _current_sense_initialized = false;
    bool _adc1_started = false;
    bool _adc2_started = false;
    bool _center_aligned = true;
    uint8_t _deadtime_ticks = 0;
    PWMDriver *_driver = nullptr;
    ADCConfig _adc_cfg{};
    adcsample_t _adc1_samples[2]{};
    adcsample_t _adc2_samples[1]{};
    uint16_t _phase_ticks[3]{};
    PWMConfig _pwm_cfg{};
};

} // namespace ChibiOS
