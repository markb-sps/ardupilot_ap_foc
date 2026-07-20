#pragma once

#include <stdint.h>
#include <math.h>

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
    enum class State : uint8_t { IDLE, ALIGN, OPENLOOP, BLEND, CLOSED, FAULT, DEBUG, BEEP,
                                 HALL, HALL_DETECT };

    // VESC fault codes (subset) reported through telemetry.
    // FAULT_STALL / FAULT_HALL_SENSOR are not VESC codes — 30+ is outside the
    // VESC enum range, so they can't be confused with a real VESC fault.
    enum Fault : uint8_t {
        FAULT_NONE            = 0,
        FAULT_ABS_OVERCURRENT = 4,
        FAULT_OVER_TEMP_FET   = 5,
        FAULT_STALL           = 30,
        FAULT_HALL_SENSOR     = 31,
    };

    // Rotor-angle source. SENSORLESS = Ortega observer + I/f startup (original).
    // HALL = digital hall sensors (J304), commutates from standstill, blends to
    // the observer angle at high speed.
    enum class SensorMode : uint8_t { SENSORLESS, HALL };

    struct Config {
        // ── Rotor-angle sensing ────────────────────────────────────────────
        // Default HALL for hardware bring-up of a sensored motor; switch to
        // SENSORLESS (or set param) to run the observer-only path.
        SensorMode sensor_mode = SensorMode::HALL;
        // Electrical angle [deg] per 3-bit hall state (bit0 = A/PB11, bit1 = B/PB7,
        // bit2 = C/PB10). NaN = unmapped. Motor/wiring specific AND convention
        // specific (a 60° motor uses states {0,1,2,5,6,7}, a 120° motor {1..6}) —
        // there is no safe universal guess, so the default is ALL-INVALID. HALL
        // mode refuses to drive until a real table is loaded (from params) or
        // produced by a HALL_DETECT spin. See MotorControl::hall_table_valid().
        float hall_table_deg[8] = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN };
        // Hall→observer angle blend band [eRPM] (VESC foc_sl_erpm). Below _lo the
        // commutation angle is pure hall; above _hi it is pure observer; linear in
        // between. Keep _hi below the sensorless floor is unnecessary — the
        // observer is reliable well above _lo here.
        float hall_blend_erpm_lo = 3000.0f;
        float hall_blend_erpm_hi = 6000.0f;
        // Break-away current cap [A] applied in HALL mode until the rotor has
        // demonstrably moved (>= HALL_BREAKAWAY_N hall transitions). Bounds the
        // current dumped into a stationary rotor if the table is wrong/mis-
        // calibrated, so a bad angle can't hold near-DC current in one leg long
        // enough to cook a FET before the HALL stall trip fires. See run loop.
        float hall_breakaway_a   = 4.0f;
        // Hall-table detection spin current [A], d-axis, current-regulated (VESC
        // mcpwm_foc_hall_detect uses current control, not fixed voltage, so an
        // unloaded low-R motor can't draw a large spin current). Clamped to
        // current_max in init.
        float hall_detect_a      = 5.0f;

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
        // Power stage (v2 PCB): EPC2305 eGaN FETs — 150 V, 80 A pulsed, 3.2 mΩ,
        // MP1918 gate driver, 1 mΩ shunt + INA181A1. eGaN has NO avalanche rating
        // and a fragile gate (V_GS abs-max ~+6/-4 V): it fails from fast transients
        // (gate ring / dv/dt shoot-through), not slow heat — keep current + di/dt
        // conservative. Iq cap is approx peak phase current; margin below pulsed.
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
        // Torque-command slew limit [A/s]: the CURRENT-mode setpoint is ramped
        // toward each new command at this rate so a step throttle input (e.g. a
        // PWM RC channel snapped to full) can't apply an instant iq reference
        // jump. 0 disables (instant). VESC l_current_ramp equivalent.
        float    current_slew_a_s   = 0.0f;

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
        // Sensorless-start supervision. The observer free-runs (never seeded) during
        // the forced lock→ramp→const sequence and its angle is USED only at the
        // transfer: handover happens iff the observer has converged (|flux| held in
        // band for _lock_need samples) by the transfer speed. If it hasn't, the
        // start is abandoned, the bridge coasts for openloop_cooldown_s, then it
        // retries. Retries are capped because there is no motor-temp sensor and a
        // motor that never locks would otherwise be force-driven (and heated)
        // indefinitely — after openloop_max_attempts it latches FAULT_STALL.
        uint8_t  openloop_max_attempts = 3;       // forced-start tries before FAULT
        float    openloop_cooldown_s   = 0.5f;    // high-Z coast between tries [s]

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

    // Play an audible tone THROUGH the motor: a fixed-axis voltage vector whose
    // magnitude is modulated at freq_hz, so the windings vibrate as a speaker
    // without net rotation (a symmetric AC on one axis makes no net torque).
    // amplitude is a modulation fraction (0..1 of the SVPWM range), internally
    // capped; keep freq_hz audible (≥~500 Hz) so winding inductance limits the
    // current. Auto-releases the bridge (coast) when the tone finishes. Use it
    // for a power-on chime; do not call while driving. is_beeping() is true until
    // the tone (and its release) completes.
    void play_tone(float freq_hz, float amplitude, uint16_t duration_ms);
    bool is_beeping() const { return _mode == Mode::BEEP; }

    // ── Hall sensors ────────────────────────────────────────────────────────
    // Start a hall-table detection spin: forces a slow open-loop electrical
    // rotation and records the forced angle seen in each hall state, then stores
    // the result into the live hall table and coasts. Call only at standstill.
    void start_hall_detect();
    // True once a detection spin has completed; fills out[8] with the detected
    // sector-centre angles [deg] (NaN for unseen/invalid states).
    bool hall_detect_result(float out_deg[8]) const;
    // Like hall_detect_result() but consumes the "fresh" flag: returns true only
    // once per completed detection (for a one-shot save to storage).
    bool take_hall_detect_result(float out_deg[8]);
    // Latest decoded hall state (1..6; 0 = invalid/not yet read).
    uint8_t get_hall_state() const { return _t_hall_state; }
    // True once the hall table maps all six real states (i.e. a valid detected
    // table is loaded). HALL mode refuses to drive while this is false.
    bool hall_table_valid() const { return _hall_table_valid; }
    // Copy the live hall table out as electrical degrees [0..360), NaN = unmapped.
    void get_hall_table_deg(float out_deg[8]) const {
        for (uint8_t k = 0; k < 8; k++) {
            float d = _hall_table[k] * 57.2957795f;   // rad → deg
            if (!isnan(d) && d < 0.0f) d += 360.0f;
            out_deg[k] = d;
        }
    }

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
    float get_vbus()          const { return _vbus; }
    // Filtered bus volts, maintained by the control ISR from the PA0 divider.
    float read_vbus() { return _vbus; }
    float get_fet_temp()      const { return _t_fet_temp; }  // board NTC [°C]
    uint8_t get_fault()       const { return _fault_code; }
    uint8_t get_state()       const { return uint8_t(_state); }
    float   current_limit()   const { return _current_max; }   // iq command ceiling [A]

    // ── Config read-back (for the VESC-Tool COMM_GET_MCCONF responder) ──────
    SensorMode get_sensor_mode() const { return _sensor_mode; }
    void  get_hall_blend_erpm(float &lo, float &hi) const { lo = _hall_blend_lo; hi = _hall_blend_hi; }
    float get_current_kp() const { return _cur_kp; }
    // Per-phase motor params (unscaled — the observer holds L,R pre-scaled ×1.5).
    void  get_motor_lrflux(float &L, float &R, float &flux) const {
        L    = (_obs_L > 0.0f) ? _obs_L * (1.0f / 1.5f) : 0.0f;
        R    = _obs_R * (1.0f / 1.5f);
        flux = _obs_lambda;
    }

    volatile uint32_t _adc_sample_cb_count{0};

private:
    enum class Mode : uint8_t { STOP, CURRENT, SPEED, BRAKE, DEBUG_VOLTAGE, BEEP, HALL_DETECT };

    static void adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v, uint16_t sample_w);
    void        adc_sample_isr(uint16_t sample_u, uint16_t sample_v, uint16_t sample_w);
    void        observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta);
    // Sensorless (observer + I/f) angle/torque state machine. Sets theta/id_set/
    // iq_set for the shared current loop; returns false if it aborted the cycle
    // (bridge already gated off — the caller must return without writing PWM).
    bool        run_sensorless(Mode mode, float dir, float iq_cmd,
                               float i_alpha, float i_beta, float i_max, bool coasting,
                               float &theta, float &id_set, float &iq_set);
    // Recompute _hall_table_valid (true iff all six real hall states are mapped).
    void        update_hall_table_valid();
    // Read the three hall GPIOs → raw 3-bit state (0..7).
    uint8_t     read_hall_state() const;
    // Decode/debounce halls, update _hall_theta (interpolated commutation angle)
    // and _hall_omega (speed from transition timing). Returns false on an invalid
    // (0/7) or unmapped state.
    bool        update_hall();
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
    volatile float _cmd_current = 0.0f;   // [A] applied (slew-limited) CURRENT setpoint / BRAKE magnitude
    volatile float _cmd_current_target = 0.0f; // [A] raw CURRENT command; _cmd_current slews toward this
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
    float _i_slew_per_tick = 0.0f;   // CURRENT-setpoint slew per ISR tick [A] (0 = instant)
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
    // Sensorless-start supervision (see Config). _ol_attempts / _ol_cooldown are
    // deliberately NOT touched by reset_control() so the retry budget survives the
    // inter-attempt coast; they are cleared on stop() and on a successful handover.
    uint8_t _ol_max_attempts = 3;    // forced-start tries before FAULT
    float   _ol_cooldown_t   = 0.5f; // coast between tries [s]
    uint8_t _ol_attempts     = 0;    // failed tries this spin-up
    float   _ol_cooldown     = 0.0f; // remaining inter-try coast [s]
    // Latched once _ol_attempts hits the cap: the bridge stays off (FAULT_STALL)
    // and — unlike other trips — a nonzero current command does NOT clear it, so
    // the arbiter re-commanding every loop can't defeat the thermal cap. Cleared
    // only by a zero/stop command (throttle released). NOT reset by reset_control.
    volatile bool _ol_locked_out = false;
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

    // ── Audio beep (Mode::BEEP): fixed-axis amplitude-modulated tone ──────────
    volatile uint32_t _beep_ticks_left = 0;    // ISR ticks remaining in the tone
    float _beep_phase      = 0.0f;   // tone phase [rad] (ISR)
    float _beep_phase_step = 0.0f;   // 2π·freq·dt per tick (thread → ISR)
    float _beep_amp        = 0.0f;   // modulation amplitude [0..1] (thread → ISR)

    // ── Thermal protection (thread computes, ISR applies) ─────────────────
    float _fet_t_start     = 80.0f;
    float _fet_t_max       = 100.0f;
    uint32_t _last_temp_ms = 0;
    volatile float _i_derate     = 1.0f;   // thermal iq-limit scale [0..1]
    volatile bool  _thermal_trip = false;  // latched-by-temp over-temp trip

    // ── Hall sensors ──────────────────────────────────────────────────────
    SensorMode _sensor_mode = SensorMode::HALL;
    float _hall_table[8]   = {0};   // sector-centre electrical angle [rad]; NaN if invalid
    volatile bool _hall_table_valid = false; // all six real states mapped → HALL may drive
    float _hall_blend_lo   = 3000.0f; // pure-hall below this [eRPM]
    float _hall_blend_hi   = 6000.0f; // pure-observer above this [eRPM]
    float _hall_breakaway_a = 4.0f;   // iq cap until motion confirmed [A]
    uint8_t _hall_move_count = 0;     // committed transitions since drive re-arm (saturating)
    // Runtime decode state (ISR-only except _t_hall_state snapshot).
    uint8_t  _hall_state    = 0xFF; // committed state (0..7); 0xFF = none yet (0 is a valid state)
    uint8_t  _hall_raw_prev = 0;    // last raw read (debounce)
    uint8_t  _hall_deb      = 0;    // consecutive identical raw reads
    float    _hall_dir      = 1.0f; // rotation sign from the last transition
    float    _hall_base     = 0.0f; // interpolation origin (sector entry edge) [rad]
    float    _hall_theta    = 0.0f; // interpolated commutation angle [rad]
    float    _hall_omega    = 0.0f; // electrical speed from hall timing [rad/s]
    uint32_t _hall_ticks    = 0;    // ISR ticks since the last committed transition
    uint16_t _hall_fault    = 0;    // consecutive invalid-read samples
    // Hall-table detection accumulators (circular mean of the forced angle seen
    // in each state). _hd_* are only touched during a HALL_DETECT spin.
    float    _hd_sin[8]     = {0};
    float    _hd_cos[8]     = {0};
    uint32_t _hd_n[8]       = {0};
    float    _hd_angle      = 0.0f; // forced electrical angle [rad]
    uint32_t _hd_ticks      = 0;    // spin duration counter
    float    _hd_current    = 5.0f; // detection d-axis current target [A]
    bool     _hall_detect_done = false;
    volatile bool _hall_detect_fresh = false; // set on completion, cleared by take_hall_detect_result()
    float    _hall_detect_deg[8] = {0}; // last detection result [deg]

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
    uint16_t _lock_need    = 1500;   // samples required to call the observer locked (TRACK/restart)
    float    _debug_theta  = 0.0f;
    float    _v_alpha_prev = 0.0f;   // applied αβ volts, fed to observer next cycle
    float    _v_beta_prev  = 0.0f;

    // ── Observer state (ISR-only except snapshots) ─────────────────────────
    float _obs_x1     = 0.0f;
    float _obs_x2     = 0.0f;
    float _obs_theta  = 0.0f;
    float _obs_omega  = 0.0f;   // electrical speed [rad/s] — PLL output
    float _pll_theta  = 0.0f;   // PLL tracked angle (follows _obs_theta)

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
    volatile float   _t_fet_temp = 0.0f;   // board NTC [°C] (thread-written)
    volatile uint8_t _t_hall_state = 0;    // decoded hall state (1..6)
};

} // namespace ChibiOS
