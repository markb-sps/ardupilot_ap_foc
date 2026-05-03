#pragma once

#include <stdint.h>

namespace ChibiOS {

class MotorControl {
public:
    struct Config {
        uint32_t pwm_clock_hz               = 20000000;
        uint32_t pwm_frequency_hz           = 20000;
        uint16_t current_sample_delay_ticks = 2;
        uint8_t  deadtime_ticks             = 30;
        bool     center_aligned             = true;
        bool     break_input_enabled        = false;
        // Motor electrical parameters — tune per motor
        float    motor_Rs                   = 0.1f;    // stator resistance [Ω]
        float    motor_Ls                   = 50e-6f;  // stator inductance [H]
        float    vbus                       = 12.0f;   // DC bus voltage [V]
        float    current_scale              = 36.5f;   // ADC volts → amps: 1/(Rshunt*atten*gain)
        // Sliding Mode Observer tuning
        float    smo_gain                   = 1.5f;    // switching gain k [A]
        float    smo_cutoff_hz              = 100.0f;  // back-EMF LPF cutoff [Hz]
    };

    MotorControl() = default;

    bool init();
    bool init(const Config &cfg);
    void deinit();

    bool is_initialized() const { return _initialized; }
    bool zero_valid()     const { return _current_zero_valid; }

    void enable_outputs();
    void disable_outputs();

    void set_open_loop_target(float electrical_hz, float modulation, bool reset_phase = false);

    uint16_t period_ticks()          const { return _initialized ? _period_ticks : 0U; }
    bool     current_sense_ready()   const { return _current_sense_initialized; }
    bool     get_filtered_phase_volts(float &u, float &v, float &w) const;
    float    get_estimated_angle()   const { return _smo_theta; }
    float    get_open_loop_hz()      const {
        if (_pwm_update_rate_hz == 0) return 0.0f;
        return float(_open_loop_phase_step) * float(_pwm_update_rate_hz) / 4294967296.0f;
    }
    float    get_open_loop_amplitude() const { return _open_loop_amplitude; }
    float    get_current_scale()     const { return _current_scale; }
    float    get_vbus()              const { return _vbus_half * 2.0f; }
    void     get_smo_state(float &e_alpha, float &e_beta) const {
        e_alpha = _smo_e_alpha;
        e_beta  = _smo_e_beta;
    }

private:
    static void adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v);
    void        adc_sample_isr(uint16_t sample_u, uint16_t sample_v);
    void        update_smo_isr(float i_alpha, float i_beta);

    bool     _initialized               = false;
    bool     _current_sense_initialized = false;
    uint16_t _period_ticks              = 0;
    uint32_t _pwm_update_rate_hz        = 0;

    // Zero calibration
    volatile uint32_t _zero_accum[2]{};
    volatile uint16_t _current_zero_raw[2]{};
    volatile uint8_t  _zero_count       = 0;
    volatile bool     _current_zero_valid = false;

    // Filtered phase voltages [V] for diagnostics (multiply by current_scale for amps)
    volatile float _filtered_volts[3]{};

    // Open-loop reference (written from thread context, read in ISR)
    volatile float    _open_loop_amplitude  = 0.0f;
    volatile uint32_t _open_loop_phase      = 0;
    volatile uint32_t _open_loop_phase_step = 0;

    // Commanded αβ voltages [V] fed to SMO — ISR-only, no volatile needed
    float _v_alpha_cmd = 0.0f;
    float _v_beta_cmd  = 0.0f;

    // SMO observer state — ISR-only except _smo_theta
    float          _smo_i_alpha_hat = 0.0f;
    float          _smo_i_beta_hat  = 0.0f;
    float          _smo_e_alpha     = 0.0f;
    float          _smo_e_beta      = 0.0f;
    volatile float _smo_theta       = 0.0f; // estimated rotor angle [rad], read externally

    // Precomputed motor / SMO constants (set in init, read-only in ISR)
    float _current_scale  = 36.5f;
    float _smo_gain       = 1.5f;
    float _dt_inv_Ls      = 0.0f;  // dt / Ls
    float _rs_dt_inv_Ls   = 0.0f;  // Rs * dt / Ls
    float _smo_omega_c_dt = 0.0f;  // 2π * smo_cutoff_hz * dt
    float _vbus_half      = 6.0f;  // vbus / 2
};

} // namespace ChibiOS
