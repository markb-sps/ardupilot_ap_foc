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
    // FAULT_STALL is not a VESC code — 30 is outside the VESC enum range.
    enum Fault : uint8_t {
        FAULT_NONE            = 0,
        FAULT_ABS_OVERCURRENT = 4,
        FAULT_OVER_TEMP_FET   = 5,
        FAULT_STALL           = 30,
    };

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
        // EPC23102 GaN HB: 100 V / 65 A pulsed, ~35 A continuous (cooling-bound).
        // Iq cap is approx peak phase current; keep below continuous with margin.
        float    current_max       = 30.0f;   // iq command limit [A]
        float    overcurrent_trip  = 50.0f;   // per-phase hard trip [A] (debounced)
        // Instant trip [A]: no debounce and active even inside the post-arm
        // blanking window, so an arm-into-a-short is caught within one sample.
        float    overcurrent_trip_hard = 60.0f;
        float    max_modulation    = 0.90f;   // SVPWM duty ceiling [0..~0.95]
        // Max braking/regen MOTOR current [A]. Caps the negative (decelerating)
        // iq in SPEED mode and the magnitude in BRAKE mode. Keep conservative:
        // braking energy returns to the bus, a bench PSU can't sink it, and
        // there is no bus-OV handling yet. Raise once OV clamp / brake resistor.
        float    regen_current_max = 5.0f;
        // Bus over-voltage foldback: braking/regen current is scaled down
        // proportionally as vbus rises through the band below vbus_max,
        // reaching zero at vbus_max — keeps regen from pumping the bus past
        // vbus_max even without a brake resistor / OV clamp.
        float    vbus_max          = 40.0f;   // regen fully cut at this bus voltage [V]
        float    vbus_fold_band    = 3.0f;    // foldback starts at vbus_max - band [V]
        // FET thermal limit (PCB NTC next to the bridge): iq limit derates
        // linearly from full at fet_temp_start to zero at fet_temp_max, where a
        // FAULT_OVER_TEMP_FET also trips; the trip releases 10°C lower.
        float    fet_temp_start    = 80.0f;   // derate onset [°C]
        float    fet_temp_max      = 100.0f;  // hard trip [°C]
        // Stall protection (CLOSED state only — open-loop has no real speed
        // estimate): |erpm| below stall_erpm with |iq| above stall_current for
        // stall_time_s → FAULT_STALL, bridge off.
        float    stall_erpm        = 250.0f;
        float    stall_current     = 2.0f;    // [A]
        float    stall_time_s      = 1.0f;
        // A CURRENT↔SPEED mode switch is refused while the peak phase current is
        // at/above this [A] — forces a coast (zero command) between active modes
        // so the loop never hot-swaps under load.
        float    mode_switch_current = 1.0f;
        // Dead-time voltage error to cancel, in volts (0 disables). This is the
        // ~fixed voltage the bridge loses to dead-time per phase; measure it with
        // the 2-point static debug method (slope fit gives V_dt). ~0.10V here.
        float    deadtime_comp_volts = 0.10f;
        uint16_t command_timeout_ms = 1000;   // coast if no host packet within this (comms failsafe)

        // ── Sensorless open-loop override (models VESC mcpwm_foc control_current)
        float    openloop_current      = 5.0f;    // boost current added during the override [A] (VESC boost_q)
        float    openloop_erpm         = 600.0f;  // open-loop speed threshold [eRPM] (VESC foc_openloop_rpm)
        float    openloop_rpm_low_frac = 0.0f;    // threshold at zero current, fraction of openloop_erpm (VESC rpm_low)
        float    openloop_hyst_s       = 0.1f;    // time below threshold before override fires [s] (VESC hyst)
        float    openloop_lock_s       = 0.0f;    // hold-angle lock time at sequence start [s] (VESC t_lock)
        float    openloop_ramp_s       = 0.1f;    // forced-speed ramp-up time [s] (VESC t_ramp)
        float    openloop_const_s      = 0.05f;   // forced-speed hold time after ramp [s] (VESC t_const)
        float    openloop_release_s    = 0.3f;    // boost fade-out after handover [s] (0 = step, old behaviour)
        // Fresh-start TRACK phase: hold iq=0 closed-loop for this long before
        // driving. Applied volts ≈ back-EMF, so the observer re-converges on a
        // still-freewheeling rotor (restart after stop) → seamless catch; from
        // standstill the open-loop hysteresis fires as usual ~openloop_hyst_s in.
        // Clamped in init to at least openloop_hyst_s + 20 ms.
        float    resync_time_s         = 0.15f;
        float    openloop_max_q        = 3.0f;    // open-loop iq cap [A] (VESC foc_sl_openloop_max_q) — limits startup heat

        // ── Observer / speed loop ──────────────────────────────────────────
        float    observer_gain = 2.5e7f;   // Ortega γ (bw ≈ γ·λ²); VESC-equivalent (9e7·(λ_vesc/λ)²)
        // VESC scales the observer gain with duty/speed (m_gamma_now duty map) so
        // it is gentle at low speed and full once back-EMF is large. Gain scales
        // linearly with modulation depth, reaching full `observer_gain` at
        // `observer_gain_mod_full` and floored at `observer_gain_slow_frac` below.
        // This is what keeps the open-loop hard-switch convergence from blipping.
        float    observer_gain_mod_full  = 0.4f;   // modulation at which gain hits full
        float    observer_gain_slow_frac = 0.25f;  // floor fraction at standstill (VESC foc_observer_gain_slow)
        float    speed_kp      = 0.0005f;  // outer speed PI [A per eRPM]
        float    speed_ki      = 0.001f;   // outer speed PI [A per eRPM·s]
        // VESC s_pid_ramp_erpms_s: slew the speed setpoint toward the command at
        // this accel limit. Keeps the post-handover acceleration controlled so the
        // observer/PLL can track it (no desync on large speed commands).
        float    speed_ramp_erpm_s = 5000.0f; // setpoint slew rate [eRPM/s]
        // PLL that tracks the observer angle to produce a clean speed estimate
        // (VESC foc_pll_run). Replaces noisy angle-differentiation. Tuned for
        // ωn≈500 rad/s (~80 Hz), ζ≈1: kp=2·ζ·ωn, ki=ωn². Fast enough to follow
        // the open-loop ramp without lag (slower gains push the floor UP), but
        // still smoother than differentiation.
        float    pll_kp        = 1000.0f;
        float    pll_ki        = 250000.0f;

        // ── Open-loop voltage debug mode (current loop + observer bypassed) ─
        float    debug_openloop_hz    = 0.0f;   // fixed electrical rotation [Hz]
        float    debug_max_modulation = 0.06f;  // hard duty clamp (≈V/R bound, no heatsink)
    };

    MotorControl() = default;

    bool init();
    bool init(const Config &cfg);

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
    void set_current(float amps);        // torque control (signed iq). 0 = hold iq=0 (smooth coast) if running, else idle.
    void set_brake_current(float amps);  // regen brake: iq opposite to rotation, |iq|=amps; auto-releases at low speed.
    void set_rpm(float erpm);            // speed control (signed electrical RPM)
    void stop();                         // coast + clear latched fault

    // Host failsafe: coast if no host packet arrived within command_timeout_ms.
    // Call periodically from thread context with the current millis() timestamp.
    void check_command_timeout(uint32_t now_ms);
    // FET thermal protection: reads the board NTC, updates the derate factor
    // applied to the iq limit and the over-temp trip flag. Thread context;
    // call periodically (internally throttled to 10 Hz).
    void update_thermal(uint32_t now_ms);
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
        ia = _t_ia; ib = _t_ib; ic = _t_ic;
    }
    // Sum of the three measured phase currents [A]. ≈0 in healthy operation;
    // a persistent non-zero value flags a dead phase / open shunt / bad amp.
    // Reads 0 exactly when the W channel isn't live (ic falls back to -(ia+ib)).
    float get_phase_residual() const { return _t_i_resid; }
    bool  w_sense_ok()         const { return _w_sense_valid; }
    float get_motor_current() const { return _t_iq; }      // q-axis ≈ torque current
    float get_duty()          const { return _t_duty; }    // modulation [0..1]
    float get_erpm()          const { return _t_erpm; }    // electrical RPM
    float get_estimated_angle() const { return _t_theta; }     // control angle [rad]
    float get_observer_angle()  const { return _t_obs_theta; } // observer angle [rad]
    float get_free_observer_angle() const { return _t_free_theta; } // unseeded shadow observer [rad]
    float get_vbus()          const { return _vbus; }
    // Filtered bus volts, maintained by the control ISR from the PA0 divider.
    float read_vbus() { return _vbus; }
    float get_fet_temp()      const { return _t_fet_temp; }  // board NTC [°C]
    uint8_t get_fault()       const { return _fault_code; }
    uint8_t get_state()       const { return uint8_t(_state); }

    volatile uint32_t _adc_sample_cb_count{0};

private:
    enum class Mode : uint8_t { STOP, CURRENT, SPEED, BRAKE, DEBUG_VOLTAGE };

    static void adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v, uint16_t sample_w);
    void        adc_sample_isr(uint16_t sample_u, uint16_t sample_v, uint16_t sample_w);
    void        observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta);
    // Largest |phase current| across U/V/W [A] (lock-free telemetry snapshot).
    float       peak_phase_current() const;
    // False if `target` would hot-swap between active drive modes (CURRENT/
    // SPEED) while peak current is above _mode_switch_i.
    bool        mode_change_allowed(Mode target) const;
    void        reset_control();
    void        trip_fault(uint8_t code);
    void        hold_off(State s);   // gate bridge off, park control state

    // ── Hardware / init ────────────────────────────────────────────────────
    bool     _initialized               = false;
    bool     _current_sense_initialized = false;
    uint16_t _period_ticks              = 0;
    uint32_t _pwm_update_rate_hz        = 0;
    volatile float _vbus                = 18.0f;  // filtered bus volts (ISR → thread)
    float    _current_scale             = 36.5f;

    // ── Zero-current calibration ───────────────────────────────────────────
    // [0]=U, [1]=V, [2]=W. U/V gate the calibration (control depends on them);
    // W is calibrated alongside but its absence never blocks start-up.
    volatile uint32_t _zero_accum[3]{};
    volatile uint16_t _current_zero_raw[3]{};
    volatile uint16_t _zero_count       = 0;
    volatile bool     _current_zero_valid = false;
    volatile bool     _w_sense_valid    = false;  // W amp/ref present → use measured ic

    // ── Commands (written from thread, read in ISR) ────────────────────────
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
    float _oc_trip_hard    = 60.0f;  // instant trip, active during blanking too [A]
    float _regen_max       = 5.0f;   // max braking/regen motor current [A]
    float _vbus_max        = 40.0f;  // regen folds to zero at this bus voltage [V]
    float _vbus_fold_inv   = 1.0f/3.0f; // 1 / vbus_fold_band
    float _mode_switch_i   = 1.0f;   // CURRENT↔SPEED switch blocked above this |Iphase| [A]
    // Overcurrent trip is debounced (needs OC_DEBOUNCE consecutive over-limit
    // samples) and blanked for OC_BLANK_SAMPLES samples after each output enable
    // to reject the switching-noise spike when the bridge first arms.
    volatile uint16_t _oc_over_count = 0;
    volatile uint16_t _oc_blank      = 0;
    // vbus-dependent constants: seeded from cfg.vbus in init, then recomputed
    // every cycle in the ISR from the measured, filtered bus voltage.
    float _v_max           = 0.0f;   // max |v_dq| = max_mod·vbus/√3
    float _inv_vbus_half   = 0.0f;   // 2/vbus
    float _dt_comp_duty    = 0.0f;   // dead-time comp expressed as a per-phase duty step
    float _vbus_flt        = 18.0f;  // ISR-side filtered bus volts
    float _mod_to_vmax     = 0.0f;   // max_modulation/√3
    float _dt_comp_volts   = 0.0f;   // cfg.deadtime_comp_volts
    // VESC-style open-loop override constants
    float _ol_boost_q        = 0.0f; // boost current during override [A]
    float _ol_max_q          = 3.0f; // open-loop iq cap [A]
    float _open_handover_erpm = 0.0f;// open-loop speed threshold [eRPM]
    float _ol_rpm_low        = 0.0f; // threshold-at-zero-current fraction
    float _ol_hyst           = 0.1f; // time below threshold to trigger [s]
    float _ol_t_lock         = 0.0f; // lock phase duration [s]
    float _ol_t_ramp         = 0.1f; // forced-speed ramp duration [s]
    float _ol_t_total        = 0.15f;// lock + ramp + const [s]
    float _ol_t_release      = 0.3f; // post-handover boost fade-out [s]
    float _resync_t          = 0.15f;// fresh-start TRACK phase duration [s]
    float _obs_lambda        = 0.0f; // PM flux linkage λ [Wb] (observer seed)
    float _obs_L           = 0.0f;   // 1.5·Ls
    float _obs_R           = 0.0f;   // 1.5·Rs
    float _obs_lambda2     = 0.0f;   // λ²
    float _obs_gamma_half  = 0.0f;   // γ/2 (base, scaled by modulation each cycle)
    float _obs_gain_mod_inv = 0.0f;  // 1 / observer_gain_mod_full
    float _obs_gain_slow_frac = 0.25f; // gain floor fraction at low speed
    float _spd_kp          = 0.0f;
    float _spd_ki_dt       = 0.0f;
    float _spd_ramp_erpm_s = 0.0f;   // speed-setpoint slew rate [eRPM/s]
    float _spd_set_erpm    = 0.0f;   // ramped speed setpoint (follows _cmd_erpm)
    float _pll_kp          = 0.0f;   // observer-angle PLL gains (speed estimate)
    float _pll_ki          = 0.0f;
    float _erpm_to_w       = 0.0f;   // 2π/60
    float _w_to_erpm       = 0.0f;   // 60/2π
    float _debug_phase_step = 0.0f;  // |Δθ| per cycle in debug mode
    float _debug_max_mod    = 0.10f; // debug modulation clamp

    // ── Thermal protection (thread computes, ISR applies) ─────────────────
    float _fet_t_start     = 80.0f;
    float _fet_t_max       = 100.0f;
    uint32_t _last_temp_ms = 0;
    volatile float _i_derate     = 1.0f;   // thermal iq-limit scale [0..1]
    volatile bool  _thermal_trip = false;  // latched-by-temp over-temp trip

    // ── Stall protection (ISR-only) ────────────────────────────────────────
    float _stall_w         = 0.0f;   // |ω| threshold [rad/s]
    float _stall_i         = 2.0f;   // |iq| threshold [A]
    float _stall_t         = 1.0f;   // dwell [s]
    float _stall_timer     = 0.0f;

    // ── Control state (ISR-only) ───────────────────────────────────────────
    float    _integ_d      = 0.0f;
    float    _integ_q      = 0.0f;
    float    _integ_spd    = 0.0f;
    Mode     _prev_mode    = Mode::STOP; // ISR view of _mode last cycle (mode-entry detect)
    float    _override_ang = 0.0f;   // forced open-loop angle [rad]
    float    _hyst_timer   = 0.0f;   // time spent below open-loop speed [s]
    float    _ol_timer     = 0.0f;   // remaining open-loop override time [s]
    float    _ol_release   = 0.0f;   // remaining post-handover boost fade [s]
    float    _track_timer  = 0.0f;   // remaining fresh-start TRACK time [s]
    uint16_t _lock_count   = 0;      // consecutive in-band flux samples during TRACK
    uint16_t _lock_need    = 1500;   // samples required to call the observer locked
    uint16_t _ol_lock_count = 0;     // shadow-observer lock samples during OPENLOOP
    float    _ol_run_time  = 0.0f;   // cumulative time in this OL sequence [s]
    float    _debug_theta  = 0.0f;
    float    _v_alpha_prev = 0.0f;   // applied αβ volts, fed to observer next cycle
    float    _v_beta_prev  = 0.0f;

    // ── Observer state (ISR-only except snapshots) ─────────────────────────
    float _obs_x1     = 0.0f;
    float _obs_x2     = 0.0f;
    float _obs_theta  = 0.0f;
    float _obs_omega  = 0.0f;   // electrical speed [rad/s] — PLL output
    float _pll_theta  = 0.0f;   // PLL tracked angle (follows _obs_theta)
    float _free_x1    = 0.0f;   // shadow observer — never seeded/clobbered (diagnostic)
    float _free_x2    = 0.0f;

    // ── Telemetry snapshots (ISR → thread) ─────────────────────────────────
    volatile State   _state      = State::IDLE;
    volatile uint8_t _fault_code = FAULT_NONE;
    volatile float   _t_id    = 0.0f;
    volatile float   _t_iq    = 0.0f;
    volatile float   _t_vd    = 0.0f;
    volatile float   _t_vq    = 0.0f;
    volatile float   _t_ia    = 0.0f;
    volatile float   _t_ib    = 0.0f;
    volatile float   _t_ic    = 0.0f;   // measured phase-W current (or -(ia+ib) fallback)
    volatile float   _t_i_resid = 0.0f; // ia+ib+ic residual (0 when W not live)
    volatile float   _t_duty  = 0.0f;
    volatile float   _t_erpm  = 0.0f;
    volatile float   _t_theta = 0.0f;
    volatile float   _t_obs_theta = 0.0f;
    volatile float   _t_free_theta = 0.0f;
    volatile float   _t_fet_temp = 0.0f;   // board NTC [°C] (thread-written)
};

} // namespace ChibiOS
