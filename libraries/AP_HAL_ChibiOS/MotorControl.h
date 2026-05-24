#pragma once

#include <stdint.h>

namespace ChibiOS {

// Sensorless FOC motor controller modelled on vedderb/bldc (VESC).
//
//   * Always-closed dq current PI loop (Kp = L·ωbw, Ki = R·ωbw).
//   * Ortega flux-linkage observer for rotor angle (no HFI — surface motor).
//   * I/f open-loop startup: align → forced-angle ramp at fixed current →
//     blend into observer angle → fully sensorless (closed) control.
//   * Optional outer electrical-speed PI for VESC-Tool RPM commands.
//
// The whole control cycle runs in the ADC injected-EOC ISR, once per PWM
// period. Thread-side code only issues set-points and toggles the output stage.
class MotorControl {
public:
    enum class State : uint8_t { IDLE, ALIGN, OPENLOOP, BLEND, CLOSED, FAULT, DEBUG };

    // VESC fault codes (subset) reported through telemetry.
    enum Fault : uint8_t { FAULT_NONE = 0, FAULT_ABS_OVERCURRENT = 4 };

    struct Config {
        // ── PWM / current sense ────────────────────────────────────────────
        uint32_t pwm_clock_hz               = 20000000;
        uint32_t pwm_frequency_hz           = 20000;
        uint16_t current_sample_delay_ticks = 5;
        uint8_t  deadtime_ticks             = 3;
        bool     center_aligned             = true;
        bool     break_input_enabled        = false;
        float    current_scale              = 36.5f;   // ADC volts → amps

        // ── Motor (BDUAV 6374-170kv defaults; tune per motor) ──────────────
        float    motor_Rs    = 0.06f;    // phase resistance [Ω]
        float    motor_Ls    = 80e-6f;   // phase inductance [H]
        float    motor_flux  = 4.6e-3f;  // PM flux linkage λ [Wb]
        float    vbus        = 18.0f;    // DC bus [V] (fixed until ADC added)

        // ── Current loop / limits ──────────────────────────────────────────
        float    current_bw_rad    = 1000.0f; // current-loop bandwidth [rad/s]
        float    current_max       = 15.0f;   // iq command limit [A]
        float    overcurrent_trip  = 30.0f;   // per-phase hard trip [A]
        float    max_modulation    = 0.90f;   // SVPWM duty ceiling [0..~0.95]
        uint16_t command_timeout_ms = 1000;   // coast if no host packet within this (comms failsafe)

        // ── Sensorless I/f startup ─────────────────────────────────────────
        float    openloop_current      = 5.0f;    // forced-commutation current [A]
        float    openloop_erpm         = 600.0f;  // handover speed [electrical RPM]
        float    openloop_accel_erpm_s = 3000.0f; // ramp rate [eRPM/s]
        uint16_t align_ms              = 500;      // rotor pre-align time [ms]
        uint16_t blend_ms              = 100;      // open→observer angle blend [ms]

        // ── Observer / speed loop ──────────────────────────────────────────
        float    observer_gain = 9.0e6f;   // Ortega γ (bw ≈ γ·λ²); tune
        float    speed_kp      = 0.0005f;  // outer speed PI [A per eRPM]
        float    speed_ki      = 0.001f;   // outer speed PI [A per eRPM·s]

        // ── Open-loop voltage debug mode (current loop + observer bypassed) ─
        float    debug_openloop_hz    = 0.0f;   // fixed electrical rotation [Hz]
        float    debug_max_modulation = 0.02f;  // hard duty clamp (≈V/R bound, no heatsink)
    };

    MotorControl() = default;

    bool init();
    bool init(const Config &cfg);
    void deinit();

    bool is_initialized()   const { return _initialized; }
    bool zero_valid()       const { return _current_zero_valid; }
    // True when the bridge should be driving: calibrated, unfaulted, commanded.
    bool is_active()        const {
        return _current_zero_valid && _fault_code == FAULT_NONE && _mode != Mode::STOP;
    }
    bool current_sense_ready() const { return _current_sense_initialized; }
    uint16_t period_ticks() const { return _initialized ? _period_ticks : 0U; }

    void enable_outputs();
    void disable_outputs();

    // ── Set-points (thread context) ────────────────────────────────────────
    void set_current(float amps);  // torque control (signed iq)
    void set_rpm(float erpm);      // speed control (signed electrical RPM)
    void stop();                   // coast + clear latched fault

    // Host failsafe: coast if no host packet arrived within command_timeout_ms.
    // Call periodically from thread context with the current millis() timestamp.
    void check_command_timeout(uint32_t now_ms);
    // Mark the host as alive — call on receipt of any valid VESC packet
    // (set-points, GET_VALUES polling, COMM_ALIVE keepalives).
    void notify_host_alive();

    // Open-loop voltage bring-up: bypasses current loop and observer-control.
    // |duty|→modulation (clamped to debug_max_modulation), sign→direction.
    // Rotor turns at debug_openloop_hz; use it to verify current sign/scale and
    // observer tracking before closing the loop. duty≈0 stops.
    void set_debug_voltage(float duty);

    // ── Telemetry getters (lock-free snapshots) ────────────────────────────
    void  get_idq(float &id, float &iq) const { id = _t_id; iq = _t_iq; }
    void  get_vdq(float &vd, float &vq) const { vd = _t_vd; vq = _t_vq; }
    void  get_phase_currents(float &ia, float &ib, float &ic) const {
        ia = _t_ia; ib = _t_ib; ic = -(_t_ia + _t_ib);
    }
    float get_motor_current() const { return _t_iq; }      // q-axis ≈ torque current
    float get_duty()          const { return _t_duty; }    // modulation [0..1]
    float get_erpm()          const { return _t_erpm; }    // electrical RPM
    float get_estimated_angle() const { return _t_theta; }     // control angle [rad]
    float get_observer_angle()  const { return _t_obs_theta; } // observer angle [rad]
    float get_vbus()          const { return _vbus; }
    uint8_t get_fault()       const { return _fault_code; }
    uint8_t get_state()       const { return uint8_t(_state); }

    volatile uint32_t _adc_sample_cb_count{0};

private:
    static void adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v);
    void        adc_sample_isr(uint16_t sample_u, uint16_t sample_v);
    void        observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta);
    void        reset_control();
    void        trip_fault(uint8_t code);
    void        hold_off(State s);   // gate bridge off, park control state

    // ── Hardware / init ────────────────────────────────────────────────────
    bool     _initialized               = false;
    bool     _current_sense_initialized = false;
    uint16_t _period_ticks              = 0;
    uint32_t _pwm_update_rate_hz        = 0;
    float    _vbus                      = 18.0f;
    float    _current_scale             = 36.5f;

    // ── Zero-current calibration ───────────────────────────────────────────
    volatile uint32_t _zero_accum[2]{};
    volatile uint16_t _current_zero_raw[2]{};
    volatile uint16_t _zero_count       = 0;
    volatile bool     _current_zero_valid = false;

    // ── Commands (written from thread, read in ISR) ────────────────────────
    enum class Mode : uint8_t { STOP, CURRENT, SPEED, DEBUG_VOLTAGE };
    volatile Mode  _mode        = Mode::STOP;
    volatile float _cmd_current = 0.0f;   // [A]
    volatile float _cmd_erpm    = 0.0f;   // [electrical RPM]
    volatile uint32_t _last_cmd_ms = 0;   // millis() of last set-point (host failsafe)
    uint16_t       _cmd_timeout_ms = 500;
    volatile float _debug_mod   = 0.0f;   // debug-mode modulation amplitude
    volatile float _debug_dir   = 1.0f;   // debug-mode rotation direction (±1)

    // ── Precomputed constants (set in init, read-only in ISR) ──────────────
    float _dt              = 0.0f;
    float _cur_kp          = 0.0f;   // L·ωbw
    float _cur_ki_dt       = 0.0f;   // R·ωbw·dt
    float _current_max     = 15.0f;
    float _oc_trip         = 30.0f;
    float _v_max           = 0.0f;   // max |v_dq| = max_mod·vbus/√3
    float _inv_vbus_half   = 0.0f;   // 2/vbus
    float _open_current    = 5.0f;
    float _open_handover_w = 0.0f;   // handover electrical speed [rad/s]
    float _open_accel_w    = 0.0f;   // ramp step per cycle [rad/s]
    uint16_t _align_cycles = 0;
    uint16_t _blend_cycles = 0;
    float _obs_L           = 0.0f;   // 1.5·Ls
    float _obs_R           = 0.0f;   // 1.5·Rs
    float _obs_lambda2     = 0.0f;   // λ²
    float _obs_gamma_half  = 0.0f;   // γ/2
    float _spd_kp          = 0.0f;
    float _spd_ki_dt       = 0.0f;
    float _erpm_to_w       = 0.0f;   // 2π/60
    float _w_to_erpm       = 0.0f;   // 60/2π
    float _debug_phase_step = 0.0f;  // |Δθ| per cycle in debug mode
    float _debug_max_mod    = 0.10f; // debug modulation clamp

    // ── Control state (ISR-only) ───────────────────────────────────────────
    float    _open_theta   = 0.0f;
    float    _open_omega   = 0.0f;
    float    _integ_d      = 0.0f;
    float    _integ_q      = 0.0f;
    float    _integ_spd    = 0.0f;
    uint16_t _stage_cnt    = 0;
    float    _debug_theta  = 0.0f;
    float    _v_alpha_prev = 0.0f;   // applied αβ volts, fed to observer next cycle
    float    _v_beta_prev  = 0.0f;

    // ── Observer state (ISR-only except snapshots) ─────────────────────────
    float _obs_x1     = 0.0f;
    float _obs_x2     = 0.0f;
    float _obs_theta  = 0.0f;
    float _obs_omega  = 0.0f;   // filtered electrical speed [rad/s]

    // ── Telemetry snapshots (ISR → thread) ─────────────────────────────────
    volatile State   _state      = State::IDLE;
    volatile uint8_t _fault_code = FAULT_NONE;
    volatile float   _t_id    = 0.0f;
    volatile float   _t_iq    = 0.0f;
    volatile float   _t_vd    = 0.0f;
    volatile float   _t_vq    = 0.0f;
    volatile float   _t_ia    = 0.0f;
    volatile float   _t_ib    = 0.0f;
    volatile float   _t_duty  = 0.0f;
    volatile float   _t_erpm  = 0.0f;
    volatile float   _t_theta = 0.0f;
    volatile float   _t_obs_theta = 0.0f;
};

} // namespace ChibiOS
