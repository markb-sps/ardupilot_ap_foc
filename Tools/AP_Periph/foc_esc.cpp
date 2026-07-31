#include "foc_esc.h"

#ifdef HAL_PERIPH_ENABLE_FOC_ESC

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL_ChibiOS/stm32_pwm_input.h>

extern const AP_HAL::HAL& hal;

// ── Current-sense scale (v2 PCB hardware) ───────────────────────────────────
// Exact scale = 1 / (Rshunt · input_attenuation · amp_gain).
namespace {
constexpr float phase_current_shunt_ohms = 0.001f;  // R407 = 1 mΩ
// External INA181A1 (gain 20 V/V) sits directly across the shunt — no external
// input divider. Its REF = 1.8 V zero-current bias is removed by the firmware's
// startup auto-calibration, so it doesn't enter the scale.
constexpr float phase_current_amp_gain = 20.0f;
constexpr float phase_current_shunt_input_attenuation = 1.0f;

// ── J305 RC PWM throttle calibration (single-channel servo pulse) ───────────
constexpr uint16_t THR_PWM_MIN_US       = 1000;  // 0% throttle pulse
constexpr uint16_t THR_PWM_MAX_US       = 2000;  // 100% throttle pulse
constexpr uint16_t THR_PWM_DEADZONE_US  = 30;    // ignore this much above min (0% band)
constexpr uint16_t THR_PWM_RANGE_TOL_US = 100;   // accept 900..2100; beyond → signal invalid
// Arming gate: the throttle must be seen at/below this (fully closed) before it
// may command torque, so a power-up/reconnect at a raised stick won't spin the
// motor until it has passed through the low end at least once.
constexpr uint16_t THR_PWM_ARM_MAX_US   = 1000;

// A source is "fresh" for this long after its last valid command — covers a
// couple of dropped 50 Hz RC frames / DroneCAN commands before it ages to coast.
constexpr uint32_t THR_SOURCE_TIMEOUT_MS = 200;

// ── Power-on chime (played through the motor windings) ──────────────────────
struct ChimeNote { uint16_t freq_hz; uint16_t ms; };
constexpr ChimeNote CHIME[] = { {1047, 120}, {1319, 120}, {1568, 200} };  // C6–E6–G6
constexpr uint8_t   CHIME_LEN       = sizeof(CHIME) / sizeof(CHIME[0]);
constexpr float     CHIME_AMPLITUDE = 0.08f;  // modulation fraction (capped in play_tone)
constexpr uint16_t  CHIME_GAP_MS    = 40;     // silence between notes
// How long to wait for the controller to become ready before giving up and
// skipping the chime. Must not be indefinite — see update_startup_chime().
constexpr uint32_t  CHIME_READY_TIMEOUT_MS = 3000;
}

const AP_Param::GroupInfo FOC_ESC::var_info[] = {
    // @Param: SENSOR
    // @DisplayName: FOC rotor-angle sensor mode
    // @Description: Rotor angle source for the FOC ESC.
    // @Values: 0:Sensorless,1:Hall
    // @User: Standard
    AP_GROUPINFO("SENSOR", 1, FOC_ESC, _p_sensor_mode, 1),

    // @Param: HALL0
    // @DisplayName: Hall table angle for state 0
    // @Description: Electrical angle for hall state 0, or -1 if this state is unused. Set by hall detection.
    // @Units: deg
    // @Range: -1 359
    // @User: Advanced
    AP_GROUPINFO("HALL0", 2, FOC_ESC, _p_hall[0], -1),
    // @Param: HALL1
    // @Description: Hall table angle for state 1 (deg, -1 = unused).
    AP_GROUPINFO("HALL1", 3, FOC_ESC, _p_hall[1], -1),
    // @Param: HALL2
    // @Description: Hall table angle for state 2 (deg, -1 = unused).
    AP_GROUPINFO("HALL2", 4, FOC_ESC, _p_hall[2], -1),
    // @Param: HALL3
    // @Description: Hall table angle for state 3 (deg, -1 = unused).
    AP_GROUPINFO("HALL3", 5, FOC_ESC, _p_hall[3], -1),
    // @Param: HALL4
    // @Description: Hall table angle for state 4 (deg, -1 = unused).
    AP_GROUPINFO("HALL4", 6, FOC_ESC, _p_hall[4], -1),
    // @Param: HALL5
    // @Description: Hall table angle for state 5 (deg, -1 = unused).
    AP_GROUPINFO("HALL5", 7, FOC_ESC, _p_hall[5], -1),
    // @Param: HALL6
    // @Description: Hall table angle for state 6 (deg, -1 = unused).
    AP_GROUPINFO("HALL6", 8, FOC_ESC, _p_hall[6], -1),
    // @Param: HALL7
    // @Description: Hall table angle for state 7 (deg, -1 = unused).
    AP_GROUPINFO("HALL7", 9, FOC_ESC, _p_hall[7], -1),

    // ── Motor identity (VESC Tool "Write Motor Configuration" writes these) ──
    // @Param: M_RS
    // @DisplayName: Motor phase resistance
    // @Units: Ohm
    // @User: Standard
    AP_GROUPINFO("M_RS", 10, FOC_ESC, _p_motor_rs, 0.055f),
    // @Param: M_LS
    // @DisplayName: Motor phase inductance
    // @Units: H
    // @User: Standard
    AP_GROUPINFO("M_LS", 11, FOC_ESC, _p_motor_ls, 80e-6f),
    // @Param: M_FLUX
    // @DisplayName: Motor PM flux linkage
    // @Units: Wb
    // @User: Standard
    AP_GROUPINFO("M_FLUX", 12, FOC_ESC, _p_motor_flux, 4.6e-3f),
    // @Param: M_POLES
    // @DisplayName: Motor pole pairs
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("M_POLES", 13, FOC_ESC, _p_motor_poles, 7),

    // ── Current loop / limits ────────────────────────────────────────────────
    // @Param: I_MAX
    // @DisplayName: Max motor current (iq ceiling)
    // @Units: A
    // @User: Standard
    AP_GROUPINFO("I_MAX", 14, FOC_ESC, _p_i_max, 15.0f),
    // @Param: I_OC
    // @DisplayName: Overcurrent trip (debounced, per phase)
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_OC", 15, FOC_ESC, _p_i_oc, 30.0f),
    // @Param: I_OCHARD
    // @DisplayName: Instant overcurrent trip (per phase)
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_OCHARD", 16, FOC_ESC, _p_i_oc_hard, 45.0f),
    // @Param: I_REGEN
    // @DisplayName: Max braking/regen current
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_REGEN", 17, FOC_ESC, _p_i_regen, 5.0f),
    // @Param: I_SLEW
    // @DisplayName: Torque-command slew limit
    // @Units: A/s
    // @User: Advanced
    AP_GROUPINFO("I_SLEW", 18, FOC_ESC, _p_i_slew, 150.0f),
    // @Param: I_SCALE
    // @DisplayName: Current-sense scale (ADC volts to amps)
    // @User: Advanced
    AP_GROUPINFO("I_SCALE", 19, FOC_ESC, _p_i_scale, 50.0f),

    // ── Bus / thermal / stall protections ────────────────────────────────────
    // @Param: V_MAX
    // @DisplayName: Bus voltage at which regen is fully cut
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_MAX", 20, FOC_ESC, _p_v_max, 40.0f),
    // @Param: V_FOLD
    // @DisplayName: Bus over-voltage foldback band
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_FOLD", 21, FOC_ESC, _p_v_fold, 3.0f),
    // @Param: T_START
    // @DisplayName: FET temperature derate onset
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("T_START", 22, FOC_ESC, _p_t_start, 80.0f),
    // @Param: T_MAX
    // @DisplayName: FET over-temperature hard trip
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("T_MAX", 23, FOC_ESC, _p_t_max, 100.0f),
    // @Param: STL_RPM
    // @DisplayName: Stall speed threshold
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("STL_RPM", 24, FOC_ESC, _p_stall_rpm, 250.0f),
    // @Param: STL_I
    // @DisplayName: Stall current threshold
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("STL_I", 25, FOC_ESC, _p_stall_i, 2.0f),
    // @Param: STL_T
    // @DisplayName: Stall dwell before trip
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("STL_T", 26, FOC_ESC, _p_stall_t, 1.0f),
    // @Param: V_MIN
    // @DisplayName: Bus under-voltage floor
    // @Description: Bridge is gated off below this bus voltage on a single reading (no debounce); motoring current folds back over V_FOLD above it. Protects the bus-derived gate-drive rail from a current-limited supply collapsing while driving. Keep above the gate driver supply's dropout.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_MIN", 27, FOC_ESC, _p_v_min, 14.0f),
    // @Param: V_UVFOLD
    // @DisplayName: Bus under-voltage foldback band
    // @Description: Motoring current scales from full at (V_MIN + this) down to zero at V_MIN. Separate from V_FOLD, which is the over-voltage regen band. Keep V_MIN + V_UVFOLD comfortably below the lowest bus voltage you actually run at, or normal supply sag will silently derate torque. VESC's equivalent (l_battery_cut_start/end) sits far below normal running voltage.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_UVFOLD", 28, FOC_ESC, _p_v_uvfold, 2.0f),
    // @Param: M_OBSG
    // @DisplayName: Ortega observer gain (gamma)
    // @Description: Sensorless flux observer gain. VESC Tool's FOC tab shows and writes this as "Observer Gain (x1M)", and its detection wizard computes it as 1e3/lambda^2 after measuring the flux linkage. Scales as 1/lambda^2, so it must be re-derived whenever M_FLUX changes.
    // @User: Advanced
    AP_GROUPINFO("M_OBSG", 29, FOC_ESC, _p_obs_gain, 2.5e7f),
    // @Param: M_CKP
    // @DisplayName: Current-loop proportional gain
    // @Description: Direct current-PI Kp in volts per amp (VESC foc_current_kp). 0 = derive from the internal bandwidth and M_LS, which is the behaviour if VESC Tool has never written it.
    // @User: Advanced
    AP_GROUPINFO("M_CKP", 30, FOC_ESC, _p_cur_kp, 0.0f),
    // @Param: M_CKI
    // @DisplayName: Current-loop integral gain
    // @Description: Direct current-PI Ki in volts per amp-second (VESC foc_current_ki). 0 = derive from the internal bandwidth and M_RS.
    // @User: Advanced
    AP_GROUPINFO("M_CKI", 31, FOC_ESC, _p_cur_ki, 0.0f),

    AP_GROUPEND
};

void FOC_ESC::init(AP_HAL::UARTDriver *vesc_uart)
{
    ChibiOS::MotorControl::Config motor_cfg;
    motor_cfg.pwm_clock_hz = 20000000;
    motor_cfg.pwm_frequency_hz = 20000;
    motor_cfg.current_sample_delay_ticks = 5;
    // Bridge dead time, in real nanoseconds — the driver encodes it against the
    // timer kernel clock. This was previously "8", intended as 8 PWM counter
    // ticks (400 ns) but applied as 8 DTG steps of the 160 MHz kernel clock =
    // 50 ns. Verify the achieved value with motor_control.deadtime_ns() and on
    // a scope before running the bridge hard.
    motor_cfg.deadtime_ns = 200;
    motor_cfg.center_aligned = true;
    motor_cfg.break_input_enabled = false;
    // BDUAV 6374-170kv electrical parameters (tune on hardware via VESC Tool).
    motor_cfg.motor_Rs   = 0.055f;   // phase resistance [Ω] (Step-1 I–V slope: ≈0.055)
    motor_cfg.motor_Ls   = 80e-6f;   // phase inductance [H]
    motor_cfg.motor_flux = 4.6e-3f;  // PM flux linkage λ [Wb] (≈60/(√3·π·Kv·poles))
    motor_cfg.vbus       = 18.0f;    // DC bus [V] (fixed until bus ADC added)
    // Per-phase duty ceiling. The MP1918 high side is bootstrapped, so 100% duty
    // is not a supported state, and the low-side window it reserves is also what
    // the shunt ADC samples in. 0.80 is deliberately conservative for bring-up
    // (10 µs of low-side conduction per period at 20 kHz — ~2.8x the ADC needs);
    // it caps the modulation index at 2*(0.80-0.5) = 0.60, i.e. about a third
    // less top speed than the old 0.90. Raise toward 0.92 once the stage is
    // trusted and the sampling has been checked on a scope.
    motor_cfg.duty_max         = 0.80f;
    // Conservative limits for first bring-up — raise once verified.
    motor_cfg.current_max      = 15.0f;
    motor_cfg.overcurrent_trip = 30.0f;
    // Instant trip (no debounce, not blanked on arm) — catches arming into
    // a short within one sample. Keep well above overcurrent_trip, below
    // the EPC2305 80 A pulse rating.
    motor_cfg.overcurrent_trip_hard = 45.0f;
    // Bus-OV regen foldback: braking current scales to zero as vbus rises
    // from (vbus_max - band) to vbus_max.
    motor_cfg.vbus_max       = 40.0f;
    motor_cfg.vbus_fold_band = 3.0f;
    // Startup torque boost added on top of the commanded iq during the forced
    // I/f sequence. This is the single highest-stress routine in the firmware:
    // it holds this current on a FORCED angle, into a rotor that may not be
    // following, for openloop_lock_s + ramp_s + const_s (~1.1 s) — so even a
    // 1 A command drives this much. Five gate drivers have died on this bench;
    // start conservative and only raise it if the rotor demonstrably fails to
    // spin up (watch `lag` shrinking as back-EMF appears). Was 10 A.
    motor_cfg.openloop_current = 4.0f;
    // Max braking/regen motor current [A]. Deliberately low: braking energy
    // returns to the bus, a bench PSU can't sink it, and there is no bus-OV
    // clamp yet. Raise once OV handling / a brake resistor exists.
    motor_cfg.regen_current_max = 5.0f;
    // Refuse a CURRENT↔SPEED hot-swap while peak phase current ≥ this [A];
    // the host must coast (command 0) between active modes.
    motor_cfg.mode_switch_current = 1.0f;
    // Startup spin-up: the 6374 rotor can't follow a 0.1 s ramp on only 3 A,
    // so it slips and the observer never sees coherent back-EMF. Give it real
    // torque and a gentle, long ramp/hold so we can confirm it actually spins
    // (watch `lag`→small as back-EMF appears) before worrying about handover.
    motor_cfg.openloop_max_q  = 15.0f;  // = current_max: full budget for startup torque
    // Capture phase: hold the vector static (current ramped in over the
    // first ~75 ms) until the rotor's settle oscillation dies, THEN
    // accelerate — otherwise capture happens mid-ramp and a bad draw
    // slips poles backward before catching (backward-run-then-jerk start).
    motor_cfg.openloop_lock_s = 0.4f;
    motor_cfg.openloop_ramp_s = 0.5f;   // was 0.1 s — gentler so the rotor can keep up
    motor_cfg.openloop_const_s = 0.2f;  // hold forced rotation long enough to observe
    // Hand over to the observer at HIGHER speed so back-EMF is large enough for
    // a clean lock. Note this is NOT the actual handover eRPM: like VESC
    // (mcpwm_foc utils_map on iq vs current_max), the effective threshold is
    //   map(|iq|+boost, 0, current_max, rpm_low_frac*openloop_erpm, openloop_erpm)
    // At boost_q=10 A of 15 A, a zero-throttle start hands over near 0.8·this;
    // full throttle at this ceiling — both well above the observer's speed floor.
    motor_cfg.openloop_erpm = 2000.0f;
    // Outer speed PI — strong enough to reject load (tune: ↑ if sluggish, ↓ if hunting).
    motor_cfg.speed_kp = 0.005f;   // [A per eRPM]
    motor_cfg.speed_ki = 0.05f;    // [A per eRPM·s]
    motor_cfg.current_scale = 1.0f / (phase_current_shunt_ohms *
                                       phase_current_shunt_input_attenuation *
                                       phase_current_amp_gain);
    // Bring-up: debug-voltage mode rotates the applied vector at this rate so
    // the observer has real back-EMF to lock onto (Step 2 sign/tracking check).
    motor_cfg.debug_openloop_hz = 1.0f;   // ≈60 eRPM forced rotation (pulls in from rest)
    // Torque-command slew: full-scale (current_max) reached in ~100 ms so a
    // PWM RC throttle step can't apply an instant iq jump. See MotorControl.
    motor_cfg.current_slew_a_s = 150.0f;   // [A/s]

    // ── Apply persisted config (AP_Periph storage) over the defaults ────────
    // Motor identity + limits + protections are now params (tunable over
    // DroneCAN/MAVLink, and the identity/limit subset over VESC Tool). These
    // override the compile-time defaults set above.
    motor_cfg.motor_Rs             = _p_motor_rs.get();
    motor_cfg.motor_Ls             = _p_motor_ls.get();
    motor_cfg.motor_flux           = _p_motor_flux.get();
    motor_cfg.current_max          = _p_i_max.get();
    motor_cfg.overcurrent_trip     = _p_i_oc.get();
    motor_cfg.overcurrent_trip_hard = _p_i_oc_hard.get();
    motor_cfg.regen_current_max    = _p_i_regen.get();
    motor_cfg.current_slew_a_s     = _p_i_slew.get();
    motor_cfg.current_scale        = _p_i_scale.get();
    motor_cfg.vbus_max             = _p_v_max.get();
    motor_cfg.vbus_min             = _p_v_min.get();
    motor_cfg.vbus_fold_band       = _p_v_fold.get();
    motor_cfg.vbus_uv_fold_band    = _p_v_uvfold.get();
    motor_cfg.observer_gain        = _p_obs_gain.get();
    motor_cfg.current_kp           = _p_cur_kp.get();
    motor_cfg.current_ki           = _p_cur_ki.get();
    motor_cfg.fet_temp_start       = _p_t_start.get();
    motor_cfg.fet_temp_max         = _p_t_max.get();
    motor_cfg.stall_erpm           = _p_stall_rpm.get();
    motor_cfg.stall_current        = _p_stall_i.get();
    motor_cfg.stall_time_s         = _p_stall_t.get();

    motor_cfg.sensor_mode = (_p_sensor_mode.get() == 0)
                                ? ChibiOS::MotorControl::SensorMode::SENSORLESS
                                : ChibiOS::MotorControl::SensorMode::HALL;
    // Use the stored hall table only once it has been populated by a detection
    // run (any entry >= 0); a virgin board keeps the compile-time default guess.
    bool have_hall_table = false;
    for (uint8_t k = 0; k < 8; k++) {
        if (_p_hall[k].get() >= 0) { have_hall_table = true; break; }
    }
    if (have_hall_table) {
        for (uint8_t k = 0; k < 8; k++) {
            const int16_t v = _p_hall[k].get();
            motor_cfg.hall_table_deg[k] = (v < 0) ? nanf("") : float(v);
        }
    }

    motor_control.init(motor_cfg);

    // Give the VESC link the param-derived values it can't read from
    // MotorControl (for the GET_MCCONF read-out) and register the write-back
    // sink for VESC Tool's "Write Motor Configuration".
    ChibiOS::VescTelemetry::ConfSnapshot snap;
    snap.poles           = uint8_t(_p_motor_poles.get() * 2);  // count = 2×pairs
    snap.max_vin         = _p_v_max.get();
    snap.temp_fet_start  = _p_t_start.get();
    snap.temp_fet_end    = _p_t_max.get();
    snap.abs_current_max = _p_i_oc_hard.get();
    snap.observer_gain   = _p_obs_gain.get();
    vesc_telem.set_conf_snapshot(snap);
    vesc_telem.set_mcconf_sink(this, &FOC_ESC::mcconf_write_trampoline);
    vesc_telem.set_detect_sink(this, &FOC_ESC::detect_trampoline);

    vesc_telem.init(vesc_uart);

    // J305 PWM RC throttle input (TIM3_CH1 / PC6) — polled in read_pwm_throttle().
    ChibiOS::stm32_pwm_input_init();
}

// VESC Tool wrote a motor config (COMM_SET_MCCONF). Persist the standard fields
// that map to our params, then schedule a reboot so the controller re-inits with
// the new values (reboot-to-apply — matching how the params load at boot). Only
// the fields VESC exposes are here; the custom protections stay DroneCAN-only.
void FOC_ESC::on_mcconf_write(const ChibiOS::VescTelemetry::McconfIn &in)
{
    _p_motor_rs.set_and_save(in.motor_r);
    _p_motor_ls.set_and_save(in.motor_l);
    _p_motor_flux.set_and_save(in.motor_flux);
    if (in.poles >= 2) {
        _p_motor_poles.set_and_save(int8_t(in.poles / 2));   // count → pairs
    }
    _p_i_max.set_and_save(in.current_max);
    _p_i_oc_hard.set_and_save(in.abs_current_max);
    _p_v_max.set_and_save(in.max_vin);
    _p_t_start.set_and_save(in.temp_fet_start);
    _p_t_max.set_and_save(in.temp_fet_end);

    // foc_sensor_mode. VESC: 0 = SENSORLESS, 1 = ENCODER, 2 = HALL, 3 = HFI; we
    // implement only the first and third of those. Deliberately NOT a plain
    // "== 2 ? hall : sensorless": ENCODER/HFI are modes this firmware cannot run,
    // and silently reading either as SENSORLESS would drop a sensored motor onto
    // the observer path without the operator ever asking for it. Anything we do
    // not implement leaves the param untouched. 0xFF = field absent (see McconfIn).
    // Ortega observer gain. VESC Tool's detection derives this as 1e3/λ² and it
    // is meaningless at zero, so an absent/zero field leaves the param alone.
    if (in.observer_gain > 0.0f) {
        _p_obs_gain.set_and_save(in.observer_gain);
    }
    // Current-loop gains. Both must be positive to be believed: a zero Kp is a
    // dead current loop, and taking one without the other would pair a new
    // proportional term with a stale integral one.
    if (in.current_kp > 0.0f && in.current_ki > 0.0f) {
        _p_cur_kp.set_and_save(in.current_kp);
        _p_cur_ki.set_and_save(in.current_ki);
    }
    if (in.sensor_mode == 0 || in.sensor_mode == 2) {
        _p_sensor_mode.set_and_save(int8_t(in.sensor_mode == 2 ? 1 : 0));
    }

    // set_and_save() only QUEUES the write for the background IO thread
    // (AP_Param::save_queue). The deferred reboot below fires 400 ms later,
    // which on this G431's flash-emulated EEPROM is not long enough to drain
    // nine of them — a single page erase is tens of ms. The reboot then threw
    // every write away, so NO VESC-Tool config write ever persisted: the board
    // kept reporting param defaults after a confirmed write-and-reboot cycle.
    //
    // Coast first: flush() blocks this (the periph main) thread for as long as
    // the queue takes, up to 2 s, and the motor must not be left under command
    // while the loop that feeds the throttle arbiter is stalled.
    motor_control.set_current(0.0f);
    AP_Param::flush();               // blocks until saved; uses expect_delay_ms

    _reboot_ms = AP_HAL::millis();   // deferred reboot (see update())
}

// ── Motor-parameter detection ───────────────────────────────────────────────
//
// Answers VESC Tool's "Measure R/L", "Measure λ" and the motor wizard. The
// FLUX measurement follows conf_general_measure_flux_linkage_openloop():
// lock the rotor with d-axis current, ramp the commanded speed until the duty
// reaches the requested value, then average and solve
//     λ = (|v| − R·|i|)/ω_e − |i|·L
//
// R and L do NOT use VESC's mcpwm_foc_measure_res_ind() (voltage-step injection
// at the PWM level). They use this firmware's own two measurements, which are
// already validated on this hardware and reuse machinery that exists:
//   R  — DC d-axis injection at zero speed, R = vd/id. Runs through the normal
//        modulation path, so dead-time compensation applies and the reading is
//        not inflated by the dead band the way an uncompensated one is.
//   L  — the tone sweep. |Z| = V_tone/(1.5·Ipk) at each frequency (the 1.5
//        removes the B‖C parallel path), then a least-squares fit of
//        |Z|² = R² + ω²L². The SLOPE gives L and is insensitive to the fixed
//        voltage offset that dead time contributes, which is why L is taken
//        from the sweep and R is not.
// Different method, same quantity, reported through the standard command so
// VESC Tool is none the wiser.
namespace {
// Tone frequencies for the inductance sweep [Hz]. Above ~500 Hz so ωL is a
// meaningful share of |Z| (at ~9 µH, ωL at 500 Hz is only ~28 mΩ against ~50 mΩ
// of R), and at most 4 kHz because a 20 kHz PWM samples a 4 kHz tone just five
// times per cycle. Matches the bench sweep in bench_debug.py.
constexpr float    DET_L_FREQS[4]  = { 800.0f, 1500.0f, 2500.0f, 4000.0f };
constexpr float    DET_L_AMP       = 0.06f;   // modulation, capped by debug_max_modulation
constexpr uint32_t DET_L_TONE_MS   = 300;     // tone length per point
constexpr uint32_t DET_L_GAP_MS    = 120;     // silence between points
constexpr uint32_t DET_R_LOCK_MS   = 400;     // current ramp-in + settle before averaging
constexpr uint32_t DET_R_MEAS_MS   = 400;
// VESC's resistance search starts at 2 A and steps x1.5 until the resistive
// drop reaches ~1 V (mcpwm_foc_measure_res_ind), capped at l_current_max/2.
constexpr float    DET_R_START_A   = 2.0f;
constexpr uint32_t DET_F_LOCK_MS   = 500;     // VESC ramps in over 200 ms then dwells
constexpr uint32_t DET_F_STILL_MS  = 600;     // standstill duty average (VESC: 1000 ms)
constexpr uint32_t DET_F_RAMP_MAX_MS = 15000; // VESC max_time
constexpr uint32_t DET_F_SETTLE_MS = 1000;
constexpr uint32_t DET_F_MEAS_MS   = 1000;
constexpr float    DET_F_ERPM_MAX  = 12000.0f;// VESC stops ramping here
}

void FOC_ESC::on_detect_request(const ChibiOS::VescTelemetry::DetectReq &req)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    if (_det_state != DetState::IDLE) {
        return;   // VESC discards a blocking command while another is running
    }
    if (!motor_control.is_initialized() || motor_control.get_fault() != 0) {
        // Refuse rather than clear a latched trip — detection drives real
        // current, so the operator must release the fault first. Reply with the
        // failure value so VESC Tool reports an error instead of hanging.
        switch (req.kind) {
        case DetectKind::R_L:           vesc_telem.send_detect_r_l(0, 0, 0); break;
        case DetectKind::FLUX_OPENLOOP: vesc_telem.send_detect_flux(0);      break;
        case DetectKind::APPLY_ALL_FOC: vesc_telem.send_detect_apply_all(0); break;
        default: break;
        }
        return;
    }
    _det_req       = req;
    _det_r         = 0.0f;
    _det_l         = 0.0f;
    _det_freq_idx  = 0;
    _det_r_try     = DET_R_START_A;
    _det_acc_a     = _det_acc_b = 0.0f;
    _det_ticks     = 0;
    _det_erpm      = 0.0f;
    _det_duty_max  = 0.0f;
    _det_duty_still = 0.0f;
    _det_ms        = AP_HAL::millis();
    // APPLY_ALL_FOC is R/L followed by flux, so both entry points start at R.
    // A bare FLUX request skips straight to the spin and uses the R and L the
    // host supplied (or, if it supplied none, the configured values).
    _det_state = (req.kind == DetectKind::FLUX_OPENLOOP) ? DetState::FLUX_LOCK
                                                         : DetState::RES_LOCK;
    if (_det_state == DetState::FLUX_LOCK) {
        _det_r = (req.resistance > 0.0f) ? req.resistance : _p_motor_rs.get();
        _det_l = (req.inductance > 0.0f) ? req.inductance : _p_motor_ls.get();
    }
}

// Send the reply for whichever command is running and stand the motor down.
void FOC_ESC::detect_finish(float r, float l, float linkage, bool ok)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    motor_control.set_current(0.0f);
    motor_control.stop();
    switch (_det_req.kind) {
    case DetectKind::R_L:
        vesc_telem.send_detect_r_l(ok ? r : 0.0f, ok ? (l * 1e6f) : 0.0f, 0.0f);
        break;
    case DetectKind::FLUX_OPENLOOP:
        // VESC passes negative sentinels straight through as the linkage value;
        // preserve that so VESC Tool can show its own diagnostic text.
        vesc_telem.send_detect_flux(linkage);
        break;
    case DetectKind::APPLY_ALL_FOC:
        if (ok && linkage > 0.0f) {
            _p_motor_rs.set_and_save(r);
            _p_motor_ls.set_and_save(l);
            _p_motor_flux.set_and_save(linkage);
            // Observer gain scales as 1/λ², so a new flux linkage invalidates the
            // old gain. Derive it the way VESC Tool's wizard does (1e3/λ²) rather
            // than leaving a gain tuned for a different machine in place. Note
            // VESC's own firmware uses 0.5e3/λ² in
            // conf_general_measure_flux_linkage_openloop() — the tool's number is
            // 2× the firmware's, and we follow the TOOL so what the FOC tab shows
            // after a detection is what the board actually runs.
            _p_obs_gain.set_and_save(1.0e3f / (linkage * linkage));
            // Re-derive the current-loop gains from the freshly measured R and L,
            // mirroring VESC conf_general_calc_values(): kp = L·bw, ki = R·bw.
            // Without this, gains previously written by VESC Tool would survive as
            // explicit overrides tuned for the OLD motor parameters — the "0 =
            // derive" fallback only protects a board that has never had them set.
            // bw is our configured current-loop bandwidth (motor_cfg.current_bw_rad).
            constexpr float CUR_BW_RAD = 1000.0f;
            _p_cur_kp.set_and_save(l * CUR_BW_RAD);
            _p_cur_ki.set_and_save(r * CUR_BW_RAD);
            AP_Param::flush();
            vesc_telem.send_detect_apply_all(1);
            _reboot_ms = AP_HAL::millis();   // reboot-to-apply, same as MCCONF write
        } else {
            vesc_telem.send_detect_apply_all(0);
        }
        break;
    default:
        break;
    }
    _det_state = DetState::IDLE;
}

bool FOC_ESC::update_detect(uint32_t now_ms)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    if (_det_state == DetState::IDLE) {
        return false;
    }
    // Any trip during a measurement aborts it. The ISR has already gated the
    // bridge; all we do is report the failure so the tool does not hang.
    if (motor_control.get_fault() != 0) {
        detect_finish(0.0f, 0.0f, 0.0f, false);
        return false;
    }
    const uint32_t dt_ms = now_ms - _det_ms;
    float vd = 0.0f, vq = 0.0f, id = 0.0f, iq = 0.0f;
    motor_control.get_vdq(vd, vq);
    motor_control.get_idq(id, iq);

    switch (_det_state) {
    // ── Resistance: DC d-axis injection, rotor locked ───────────────────────
    case DetState::RES_LOCK:
        motor_control.set_openloop_current(_det_r_try, 0.0f);
        if (dt_ms >= DET_R_LOCK_MS) {
            _det_acc_a = _det_acc_b = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::RES_MEAS;
            _det_ms    = now_ms;
        }
        break;

    case DetState::RES_MEAS:
        motor_control.set_openloop_current(_det_r_try, 0.0f);
        // VESC averages the MAGNITUDES |v| and |i| (mcpwm_foc.c:4131 accumulates
        // NORM2(vd,vq) and NORM2(id,iq)) and takes R = |v|/|i|, rather than the
        // d-axis ratio. Same thing at DC on a locked rotor, but it does not
        // assume the whole response landed on the axis we drove.
        _det_acc_a += sqrtf(vd * vd + vq * vq);
        _det_acc_b += sqrtf(id * id + iq * iq);
        _det_ticks++;
        if (dt_ms >= DET_R_MEAS_MS) {
            const float i_avg = (_det_ticks > 0) ? (_det_acc_b / float(_det_ticks)) : 0.0f;
            const float v_avg = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            if (i_avg < 0.5f) {
                detect_finish(0.0f, 0.0f, 0.0f, false);   // no current flowed
                return false;
            }
            const float r_try = v_avg / i_avg;
            // THE point of VESC's search (mcpwm_foc_measure_res_ind): keep raising
            // the current, ×1.5 each time from 2 A, until `i > 1/r` — i.e. until
            // the resistive drop reaches ~1 V. Dead-time error is a roughly fixed
            // voltage (~0.1 V here), so measuring at a drop of ~1 V pushes it down
            // to a few percent, where at 4 A and 20 mΩ it was the same order as
            // the entire signal. This is why the first implementation read a third
            // of the real value.
            const float i_cap = _p_i_max.get() * 0.5f;   // VESC: l_current_max / 2
            if (r_try > 0.0f && _det_r_try > (1.0f / r_try)) {
                _det_r = r_try;                 // drop is big enough — accept it
            } else if ((_det_r_try * 1.5f) < i_cap) {
                _det_r_try *= 1.5f;             // step up and repeat
                _det_r      = r_try;            // keep the best so far
                _det_state  = DetState::RES_LOCK;
                _det_ms     = now_ms;
                break;
            } else {
                // Ran out of current headroom. VESC falls back to l_current_max/2
                // for one last measurement; we take the highest we reached, which
                // is the same reading without another ramp.
                _det_r = r_try;
            }
            motor_control.set_current(0.0f);
            motor_control.stop();
            _det_freq_idx = 0;
            _det_state    = DetState::IND_GAP;
            _det_ms       = now_ms;
        }
        break;

    // ── Inductance: tone sweep, |Z|² = R² + ω²L² ────────────────────────────
    case DetState::IND_GAP:
        if (dt_ms >= DET_L_GAP_MS) {
            motor_control.play_tone(DET_L_FREQS[_det_freq_idx], DET_L_AMP,
                                    uint16_t(DET_L_TONE_MS));
            _det_state = DetState::IND_TONE;
            _det_ms    = now_ms;
        }
        break;

    case DetState::IND_TONE:
        if (dt_ms >= DET_L_TONE_MS) {
            const float ipk  = motor_control.get_tone_ipk();
            const float vbus = motor_control.get_vbus();
            // Commanded phase-A amplitude. m_alpha = v·2/vbus, so v = m·vbus/2.
            const float v_cmd = DET_L_AMP * vbus * 0.5f;
            const float w     = 2.0f * float(M_PI) * DET_L_FREQS[_det_freq_idx];
            if (ipk > 0.05f) {
                const float z = v_cmd / (1.5f * ipk);
                _det_zz[_det_freq_idx] = z * z;
                _det_ww[_det_freq_idx] = w * w;
            } else {
                _det_zz[_det_freq_idx] = 0.0f;   // no response — excluded by the fit
                _det_ww[_det_freq_idx] = w * w;
            }
            _det_freq_idx++;
            if (_det_freq_idx >= 4) {
                // Least squares on (ω², |Z|²): slope = L², intercept = R².
                float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
                for (uint8_t k = 0; k < 4; k++) {
                    if (_det_zz[k] <= 0.0f) {
                        continue;
                    }
                    sx += _det_ww[k];  sy += _det_zz[k];
                    sxx += _det_ww[k] * _det_ww[k];
                    sxy += _det_ww[k] * _det_zz[k];
                    n += 1.0f;
                }
                const float den = n * sxx - sx * sx;
                const float slope = (n >= 2.0f && fabsf(den) > 1e-12f)
                                        ? ((n * sxy - sx * sy) / den) : 0.0f;
                _det_l = (slope > 0.0f) ? sqrtf(slope) : 0.0f;
                if (_det_l <= 0.0f) {
                    detect_finish(0.0f, 0.0f, 0.0f, false);
                    return false;
                }
                if (_det_req.kind == DetectKind::R_L) {
                    detect_finish(_det_r, _det_l, 0.0f, true);
                    return false;
                }
                // APPLY_ALL_FOC: carry R and L into the flux measurement.
                _det_req.resistance = _det_r;
                _det_req.inductance = _det_l;
                if (_det_req.current <= 0.0f) {
                    // Size the injection from the requested power loss budget:
                    // P = 1.5·R·I² for a three-phase machine.
                    const float pl = (_det_req.max_power_loss > 0.0f)
                                         ? _det_req.max_power_loss : 20.0f;
                    _det_req.current = sqrtf(pl / (1.5f * _det_r));
                }
                if (_det_req.duty <= 0.0f)         { _det_req.duty = 0.3f; }
                if (_det_req.erpm_per_sec <= 0.0f) { _det_req.erpm_per_sec = 1500.0f; }
                _det_state = DetState::FLUX_LOCK;
                _det_ms    = now_ms;
            } else {
                _det_state = DetState::IND_GAP;
                _det_ms    = now_ms;
            }
        }
        break;

    // ── Flux linkage: VESC conf_general_measure_flux_linkage_openloop() ─────
    case DetState::FLUX_LOCK: {
        // Ramp the current in rather than stepping it: a step into an unknown
        // rotor angle maximally excites the swing into the I/f well, which is
        // the backward-run-then-jerk start. VESC ramps over 200 ms.
        const float ramp = (dt_ms < 200) ? (float(dt_ms) / 200.0f) : 1.0f;
        motor_control.set_openloop_current(_det_req.current * ramp, 0.0f);
        if (dt_ms >= DET_F_LOCK_MS) {
            _det_acc_a = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::FLUX_STILL;
            _det_ms    = now_ms;
        }
        break;
    }

    case DetState::FLUX_STILL:
        motor_control.set_openloop_current(_det_req.current, 0.0f);
        _det_acc_a += motor_control.get_duty_vesc();
        _det_ticks++;
        if (dt_ms >= DET_F_STILL_MS) {
            _det_duty_still = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            _det_erpm     = 0.0f;
            _det_duty_max = 0.0f;
            _det_state    = DetState::FLUX_RAMP;
            _det_ms       = now_ms;
        }
        break;

    case DetState::FLUX_RAMP: {
        // Speed ramp at the requested eRPM/s. VESC accumulates a step every 1 ms;
        // deriving it from elapsed phase time instead is exact, independent of the
        // main-loop rate, and carries no state that could survive an aborted run
        // into the next one.
        _det_erpm = _det_req.erpm_per_sec * float(dt_ms) * 1e-3f;
        motor_control.set_openloop_current(_det_req.current, _det_erpm);

        const float duty_now = motor_control.get_duty_vesc();
        if (duty_now > _det_duty_max) {
            _det_duty_max = duty_now;
        }
        // VESC's three abort sentinels, reported as the linkage value so VESC
        // Tool shows its own explanation for each.
        if (dt_ms >= DET_F_RAMP_MAX_MS) {
            detect_finish(0, 0, -1.0f, false);   // never reached the target duty
            return false;
        }
        if (dt_ms > 4000 && duty_now < (_det_duty_max * 0.7f)) {
            detect_finish(0, 0, -2.0f, false);   // duty collapsed → lost the rotor
            return false;
        }
        if (dt_ms > 4000 && _det_req.duty < (_det_duty_still * 1.1f)) {
            detect_finish(0, 0, -3.0f, false);   // target below the standstill duty
            return false;
        }
        if (duty_now >= _det_req.duty || _det_erpm >= DET_F_ERPM_MAX) {
            _det_state = DetState::FLUX_SETTLE;
            _det_ms    = now_ms;
        }
        break;
    }

    case DetState::FLUX_SETTLE:
        motor_control.set_openloop_current(_det_req.current, _det_erpm);
        if (dt_ms >= DET_F_SETTLE_MS) {
            _det_acc_a = _det_acc_b = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::FLUX_MEAS;
            _det_ms    = now_ms;
        }
        break;

    case DetState::FLUX_MEAS:
        motor_control.set_openloop_current(_det_req.current, _det_erpm);
        _det_acc_a += sqrtf(vd * vd + vq * vq);   // |v|
        _det_acc_b += sqrtf(id * id + iq * iq);   // |i|
        _det_ticks++;
        if (dt_ms >= DET_F_MEAS_MS) {
            const float v_mag = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            const float i_mag = (_det_ticks > 0) ? (_det_acc_b / float(_det_ticks)) : 0.0f;
            // ω from the COMMANDED speed — in forced-angle injection that is the
            // rotor speed by construction, and it carries no estimator noise.
            const float rad_s = _det_erpm * (2.0f * float(M_PI) / 60.0f);
            float linkage = 0.0f;
            if (rad_s > 1.0f) {
                linkage = (v_mag - _det_r * i_mag) / rad_s - i_mag * _det_l;
            }
            if (!(linkage > 0.0f)) {
                detect_finish(0, 0, 0.0f, false);
                return false;
            }
            detect_finish(_det_r, _det_l, linkage, true);
            return false;
        }
        break;

    case DetState::DONE_STOP:
    case DetState::IDLE:
        _det_state = DetState::IDLE;
        return false;
    }
    return true;   // still ours — the arbiter stands off
}

// Poll the TIM3 capture and, on a valid in-range frame, refresh the PWM source.
// Enforces a boot-low arming gate: the throttle must be seen fully closed once
// before it may command torque, so a power-up (or reconnect) at high stick can't
// spin the motor. An out-of-range pulse re-arms the gate.
void FOC_ESC::read_pwm_throttle(uint32_t now_ms)
{
    uint16_t pulse_us, period_us;
    if (!ChibiOS::stm32_pwm_input_read(&pulse_us, &period_us)) {
        return;   // no fresh frame — leave _pwm_ms to age out into coast
    }
    // Plausibility: pulse in the RC band and a sane ~50–500 Hz frame period.
    if (pulse_us < THR_PWM_MIN_US - THR_PWM_RANGE_TOL_US ||
        pulse_us > THR_PWM_MAX_US + THR_PWM_RANGE_TOL_US ||
        period_us < 2000 || period_us > 25000) {
        _pwm_armed = false;   // bad signal → require a fresh low before driving
        _pwm_amps  = 0.0f;
        return;               // and let the source go stale → coast
    }
    // Frame is valid → the PWM source is present (prevents coast even at 0%).
    _pwm_ms = now_ms;
    const uint16_t low = THR_PWM_MIN_US + THR_PWM_DEADZONE_US;
    if (!_pwm_armed) {
        // Must pass through the fully-closed end (≤1000 µs) before it can drive:
        // a boot/reconnect at raised throttle stays coasted until stick is low.
        if (pulse_us <= THR_PWM_ARM_MAX_US) {
            _pwm_armed = true;
        }
        _pwm_amps = 0.0f;     // hold coast until armed through low
        return;
    }
    const float pct = constrain_float(float(pulse_us - low) /
                                      float(THR_PWM_MAX_US - low), 0.0f, 1.0f);
    _pwm_amps = pct * motor_control.current_limit();
}

void FOC_ESC::set_can_throttle(float frac)
{
    _can_amps = constrain_float(frac, 0.0f, 1.0f) * motor_control.current_limit();
    _can_ms   = AP_HAL::millis();
}

// Torque set-points come from three sources: DroneCAN ESC RawCommand (CAN), a
// VESC-Tool COMM_SET_CURRENT over USB serial (USB), and the J305 PWM RC input
// (PWM). Priority CAN > USB > PWM; as long as one source is fresh the motor is
// driven, and it coasts only when all are stale. A VESC-Tool bench override
// (rpm / brake / duty-debug) drives MotorControl directly and, while active,
// takes exclusive control — the arbiter stands off so it isn't stomped.
// Power-on chime: play the note table through the motor windings once the
// controller has finished zero-current calibration. Returns true while the chime
// owns the motor (so the arbiter stands off); aborts immediately if a real
// command arrives or a fault trips, so it can never sound over a live throttle.
bool FOC_ESC::update_startup_chime(uint32_t now_ms, bool any_command)
{
    if (_chime_state == ChimeState::DONE) {
        return false;
    }
    if (any_command || motor_control.get_fault() != 0) {
        // Abort the chime. Never stop() on a fault — stop() is the operator-level
        // reset and would clear the latched trip we are aborting for. The ISR
        // already holds the bridge off, so just stand down.
        if (motor_control.is_beeping() && motor_control.get_fault() == 0) {
            motor_control.stop();
        }
        _chime_state = ChimeState::DONE;
        return false;
    }
    if (_chime_state == ChimeState::WAIT) {
        // Two independent readiness conditions, and the chime needs BOTH before it
        // may drive the bridge:
        //   zero_valid()  — phase-current baseline captured (INA181 ref rail up)
        //   vbus_ready()  — bus measurement has settled through its RC filter
        // They settle on very different timescales: the INA reference comes up
        // fast, the VBUS divider is 357k||10k against 100 nF (τ ≈ 1 ms). Waiting
        // only on the first one armed the bridge — and the under-voltage trip
        // with it — against a bus reading that had not arrived yet, which is what
        // killed the chime part-way through.
        if (!motor_control.zero_valid() || !motor_control.vbus_ready()) {
            // Don't wait forever: returning true holds the arbiter off, so a bus
            // that never reads ready (e.g. V_MIN set above the actual supply)
            // would silently block throttle entirely. Give up and skip the chime
            // instead — losing a beep is preferable to losing the motor.
            if (_chime_step_ms == 0) {
                _chime_step_ms = now_ms + CHIME_READY_TIMEOUT_MS;
            } else if (int32_t(now_ms - _chime_step_ms) >= 0) {
                _chime_state = ChimeState::DONE;
                return false;
            }
            return true;   // not ready — hold the arbiter off, stay silent
        }
        _chime_idx = 0;
        motor_control.play_tone(CHIME[0].freq_hz, CHIME_AMPLITUDE, CHIME[0].ms);
        _chime_step_ms = now_ms + CHIME[0].ms + CHIME_GAP_MS;
        _chime_state   = ChimeState::PLAY;
        return true;
    }
    // PLAY: let the current note + trailing gap elapse, then start the next.
    if (int32_t(now_ms - _chime_step_ms) < 0) {
        return true;
    }
    if (++_chime_idx >= CHIME_LEN) {
        _chime_state = ChimeState::DONE;   // finished → hand control to the arbiter
        return false;
    }
    motor_control.play_tone(CHIME[_chime_idx].freq_hz, CHIME_AMPLITUDE, CHIME[_chime_idx].ms);
    _chime_step_ms = now_ms + CHIME[_chime_idx].ms + CHIME_GAP_MS;
    return true;
}

void FOC_ESC::update(uint32_t now_ms)
{
    if (motor_control.is_initialized()) {
        read_pwm_throttle(now_ms);
        motor_control.update_thermal(now_ms);     // FET NTC → iq derate / over-temp trip

        const bool can_fresh = _can_ms != 0 && (now_ms - _can_ms) < THR_SOURCE_TIMEOUT_MS;
        const bool pwm_fresh = _pwm_ms != 0 && (now_ms - _pwm_ms) < THR_SOURCE_TIMEOUT_MS;
        float usb_amps;
        const bool usb_fresh = vesc_telem.usb_current(now_ms, THR_SOURCE_TIMEOUT_MS, usb_amps);
        const bool override  = vesc_telem.override_active(now_ms, THR_SOURCE_TIMEOUT_MS);

        if (update_detect(now_ms)) {
            // A parameter measurement owns the motor: it drives forced-angle
            // current and any throttle input mid-run would corrupt the result.
        } else if (update_startup_chime(now_ms, can_fresh || usb_fresh || pwm_fresh || override)) {
            // Power-on chime owns the motor until it finishes or is pre-empted.
        } else if (override) {
            // VESC bench override owns the motor — leave it be.
        } else if (can_fresh) {
            motor_control.set_current(_can_amps);
        } else if (usb_fresh) {
            motor_control.set_current(usb_amps);
        } else if (pwm_fresh) {
            motor_control.set_current(_pwm_amps);
        } else {
            motor_control.set_current(0.0f);          // all sources stale → coast
        }
    }

    vesc_telem.update();

    // Persist a freshly detected hall table so it survives reboot (triggered via
    // VESC COMM_DETECT_HALL_FOC / hall_detect.py). One-shot per detection.
    float hd[8];
    if (motor_control.take_hall_detect_result(hd)) {
        for (uint8_t k = 0; k < 8; k++) {
            int16_t v = -1;   // unused state
            if (!isnan(hd[k])) {
                int a = int(lroundf(hd[k])) % 360;
                if (a < 0) a += 360;
                v = int16_t(a);
            }
            _p_hall[k].set_and_save(v);
        }
    }

    // Deferred reboot after a VESC-Tool config write: give the COMM_SET_MCCONF
    // ack time to go out and the param storage flush to settle, then re-init the
    // controller with the new values. Coast the motor first for safety.
    if (_reboot_ms != 0 && (now_ms - _reboot_ms) > 400) {
        _reboot_ms = 0;
        motor_control.set_current(0.0f);
        motor_control.disable_outputs();
        hal.scheduler->reboot(false);
    }
}

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
