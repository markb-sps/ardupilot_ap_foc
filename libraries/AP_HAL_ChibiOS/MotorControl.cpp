#include "MotorControl.h"
#include "stm32_foc_motor_control.h"
#include "foc_transforms.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL.h>
#include <hal.h>

// Needs the PWM/ADC HAL; builds without them (e.g. the bootloader) compile to
// an empty translation unit.
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && (HAL_USE_PWM == TRUE)

#include <math.h>

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS   = 3.3f / 4095.0f;
constexpr uint16_t ZERO_SAMPLES = 64;       // zero-current calibration window
constexpr uint16_t OC_DEBOUNCE      = 3;    // consecutive over-limit samples before tripping
constexpr uint16_t OC_BLANK_SAMPLES = 16;   // trip-blank window after the bridge arms
// Under-voltage samples before the bus trip fires: ONE. There is no safe amount
// of time to keep driving a bridge whose gate-drive rail is collapsing, so this
// deliberately does not debounce — the first reading below the floor latches the
// fault and gates the bridge.
//
// What makes a lone sample trustworthy is that the measurement is real by the
// time the check is armed: the divider is RC filtered in hardware (357k||10k
// against 100 nF), and _vbus_ready below withholds the protection until that
// filter has demonstrably settled. Arming against an unsettled node was what
// produced spurious trips, not the lack of a debounce.
constexpr uint16_t UV_DEBOUNCE = 1;
// Consecutive healthy bus samples required before the under-voltage protection
// arms at all (~50 ms at 20 kHz).
//
// The VBUS divider is RC filtered — 357k||10k ≈ 9.7 kOhm against 100 nF, so
// τ ≈ 1 ms and the node needs ~5 ms to settle. Nothing else in the start-up
// sequence waits for that: the chime gates on zero_valid(), which depends on the
// INA181 reference rail, a different and much faster node. So the bridge could
// arm — and the UV check go live — while the bus measurement was still charging
// through its own filter, tripping on a reading that had simply not arrived yet.
// This is a measurement-readiness gate, not a debounce; 50 ms is ~50τ.
constexpr uint16_t VBUS_READY_SAMPLES = 1000;
// ── Trip latch / repeat-trip escalation ─────────────────────────────────────
// Minimum bridge-off time after a trip before the latch may be released, even
// once the host has commanded zero. Bounds how often a persistent fault can be
// re-entered: the old behaviour (any command clears the fault) re-armed at the
// ~1 kHz command rate, turning every protection into a duty-cycle limiter on a
// destructive condition instead of a shutdown.
constexpr uint32_t TRIP_REARM_COOLDOWN_MS = 500;
// A trip landing within this long of the previous one counts as a repeat: the
// drive is not recovering, it is being re-armed into the same fault.
constexpr uint32_t TRIP_FORGET_MS         = 5000;
// Repeats before the cooldown escalates to TRIP_LOCKOUT_MS. Deliberately
// recoverable rather than absolute — a hard latch needing a power cycle would
// be worse in the field — but slow enough that a persistent fault can no longer
// cook the bridge. stop() clears the history immediately.
constexpr uint8_t  TRIP_MAX_CONSEC        = 3;
constexpr uint32_t TRIP_LOCKOUT_MS        = 10000;
// How long a cleared trip keeps being reported to the host (see reported_fault).
constexpr uint32_t FAULT_REPORT_HOLD_MS   = 5000;
// At zero current the INA181 outputs sit at the ~1.8 V reference (~2233 counts).
// If the ref rail (VBUS-derived) is not up yet — e.g. the board booted on USB
// before the motor supply was applied — the ADC reads ~0. Only begin the zero
// calibration once both channels are clearly above this floor, otherwise a ~0
// baseline would offset every reading by −1.8 V (≈ −90 A) and trip overcurrent.
constexpr uint16_t SENSE_ALIVE_COUNTS = 1000;
// Upper bound of the plausible INA zero-reference band (~1.8 V ≈ 2233 counts).
// A W baseline above this reads as a floating/unpopulated input → don't trust it.
constexpr uint16_t SENSE_REF_MAX      = 3200;
constexpr float TWO_PI          = 6.28318530718f;
constexpr float PI_F            = 3.14159265359f;
// dq-current low-pass coefficient, used only to pick the dead-time correction
// sign. Matches VESC's MCCONF_FOC_CURRENT_FILTER_CONST.
constexpr float CURRENT_FILT_K  = 0.1f;
constexpr float OL_IQ_RAMP_S    = 0.2f;   // capture soft-start: OL current ramp-in time

// ── Hall sensors ────────────────────────────────────────────────────────────
constexpr uint8_t  HALL_NONE          = 0xFF; // "no committed state yet" sentinel (0 is a valid state)
constexpr uint8_t  HALL_DEBOUNCE      = 2;    // identical raw reads before a state commit
constexpr uint16_t HALL_FAULT_SAMPLES = 200;  // consecutive invalid reads → FAULT (~10 ms @20 kHz)
constexpr float    HALL_STOP_S        = 0.05f;// no transition for this long → treat speed as 0
constexpr float    HALL_SECTOR        = 1.04719755f;   // 60° electrical [rad]
constexpr float    HALL_HALF_SECTOR   = 0.52359878f;   // 30° electrical [rad]
constexpr uint8_t  HALL_BREAKAWAY_N   = 2;    // hall transitions confirming motion → release full current
// Hall-table detection spin: slow current-controlled forced rotation, forward
// then reverse (averaging both directions cancels the hall hysteresis bias).
// Forced electrical rotation [Hz]. The whole method assumes the rotor TRACKS the
// commanded vector like a stepper — the recorded sector centre is the circular
// mean of the COMMANDED angle, so any lag between command and rotor lands
// straight in the table. 3.0 Hz was too fast for a 6374 rotor to follow against
// its own cogging and inertia: it snapped detent-to-detent, and the resulting
// table had 60° sectors scattered from 36° to 94° (one entry ~30° out of place),
// failing update_hall_table_valid() and silently blocking all HALL drive.
// VESC steps 1° every 5 ms = 1.8 s per electrical revolution.
constexpr float    HD_HZ              = 0.556f;  // was 3.0 — matched to VESC
// Detect-current ramp-in / initial align [s]. This is the rotor's only chance to
// settle onto the forced vector BEFORE angle recording starts — a rotor still
// swinging when sampling begins biases every recorded sector angle, and a
// uniformly biased table passes the geometry check silently (a rotated table has
// perfect 60° spacing). VESC ramps its detect current over ~1000 ms; 0.2 s here
// was five times shorter with no reason behind it. Matched to VESC.
constexpr float    HD_RAMP_S          = 1.0f;
constexpr float    HD_REVS            = 3.0f;  // electrical revolutions per direction to average over

inline float wrap_pi(float a)
{
    while (a >  PI_F) a -= TWO_PI;
    while (a < -PI_F) a += TWO_PI;
    return a;
}

inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

inline float step_towards(float v, float target, float step)
{
    if (v < target) return (v + step > target) ? target : v + step;
    if (v > target) return (v - step < target) ? target : v - step;
    return v;
}

// Linear map x:[in_lo,in_hi] → [out_lo,out_hi], clamped to the output range
// (matches VESC utils_map usage here).
inline float mapf(float x, float in_lo, float in_hi, float out_lo, float out_hi)
{
    if (fabsf(in_hi - in_lo) < 1e-9f) return out_lo;
    const float t = clampf((x - in_lo) / (in_hi - in_lo), 0.0f, 1.0f);
    return out_lo + (out_hi - out_lo) * t;
}
} // namespace

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

    _zero_accum[0] = _zero_accum[1] = 0U;
    _zero_count = 0;
    _current_zero_valid = false;

    Stm32FocMotorControlSetup setup{};
    setup.pwm_clock_hz               = cfg.pwm_clock_hz;
    setup.pwm_frequency_hz           = cfg.pwm_frequency_hz;
    setup.current_sample_delay_ticks = cfg.current_sample_delay_ticks;
    setup.deadtime_ns                = cfg.deadtime_ns;
    setup.center_aligned             = cfg.center_aligned;
    setup.break_input_enabled        = cfg.break_input_enabled;

    Stm32FocMotorControlCallbacks callbacks{};
    callbacks.ctx           = this;
    callbacks.phase_current = adc_sample_callback;

    const auto r = stm32_foc_motor_control_init(setup, callbacks);
    if (!r.ok) {
        return false;
    }
    _period_ticks              = r.period_ticks;
    _pwm_update_rate_hz        = r.update_rate_hz;
    _current_sense_initialized = r.current_sense_ok;
    _deadtime_ns               = r.deadtime_ns_actual;

    // ── Precompute control constants ───────────────────────────────────────
    _dt            = 1.0f / float(_pwm_update_rate_hz);
    _vbus          = cfg.vbus;
    _current_scale = cfg.current_scale;

    _cur_kp    = cfg.current_bw_rad * cfg.motor_Ls;
    _cur_ki_dt = cfg.current_bw_rad * cfg.motor_Rs * _dt;
    _current_max   = cfg.current_max;
    _i_slew_per_tick = cfg.current_slew_a_s * _dt;   // 0 → instant (slew disabled)
    _oc_trip       = cfg.overcurrent_trip;
    _oc_trip_hard  = cfg.overcurrent_trip_hard;
    _regen_max     = cfg.regen_current_max;
    _vbus_max      = cfg.vbus_max;
    // Under-voltage floor, held below vbus_max by at least the foldback band so
    // the two limits can never invert (which would fold the drive to zero at
    // every bus voltage).
    _vbus_min      = (cfg.vbus_min < cfg.vbus_max - cfg.vbus_fold_band)
                         ? cfg.vbus_min : (cfg.vbus_max - cfg.vbus_fold_band);
    _vbus_fold_inv   = (cfg.vbus_fold_band > 0.1f) ? (1.0f / cfg.vbus_fold_band) : 10.0f;
    _vbus_uvfold_inv = (cfg.vbus_uv_fold_band > 0.1f) ? (1.0f / cfg.vbus_uv_fold_band) : 10.0f;
    _mode_switch_i = cfg.mode_switch_current;
    // vbus-dependent constants: seeded from cfg.vbus here, recomputed each ISR
    // cycle from the measured, filtered bus voltage.
    _vbus_flt      = cfg.vbus;
    // Duty ceiling first: the linear modulation range must not be able to demand
    // more duty than the hard clamp allows, or write_duties() would be saturating
    // continuously and the current PI would wind up against a limit it can't see.
    // With min-max zero-sequence injection the peak phase duty is
    //   d_max = 0.5 + 0.5·max_modulation
    // so cap max_modulation at 2·(duty_max − 0.5) and the two stay consistent.
    _duty_max = clampf(cfg.duty_max, 0.55f, 0.98f);
    const float mod_ceiling = 2.0f * (_duty_max - 0.5f);
    const float eff_max_mod = (cfg.max_modulation < mod_ceiling) ? cfg.max_modulation : mod_ceiling;
    _mod_to_vmax   = eff_max_mod * 0.57735026919f; // /√3
    _dt_comp_volts   = cfg.deadtime_comp_volts;
    _dt_comp_on_duty = cfg.deadtime_comp_on_duty;
    _v_max         = _mod_to_vmax * cfg.vbus;
    _inv_vbus_half = 2.0f / cfg.vbus;

    _fet_t_start = cfg.fet_temp_start;
    _fet_t_max   = (cfg.fet_temp_max > cfg.fet_temp_start + 1.0f)
                       ? cfg.fet_temp_max : cfg.fet_temp_start + 1.0f;
    _stall_i     = cfg.stall_current;
    _stall_t     = cfg.stall_time_s;

    _erpm_to_w = TWO_PI / 60.0f;
    _w_to_erpm = 60.0f / TWO_PI;
    _stall_w   = cfg.stall_erpm * _erpm_to_w;

    _ol_boost_q         = cfg.openloop_current;
    _ol_max_q           = cfg.openloop_max_q;
    _ol_max_attempts    = cfg.openloop_max_attempts;
    _ol_cooldown_t      = cfg.openloop_cooldown_s;
    _open_handover_erpm = cfg.openloop_erpm;
    _ol_rpm_low         = cfg.openloop_rpm_low_frac;
    _ol_hyst            = cfg.openloop_hyst_s;
    _ol_t_lock          = cfg.openloop_lock_s;
    _ol_t_ramp          = cfg.openloop_ramp_s;
    _ol_t_total         = cfg.openloop_lock_s + cfg.openloop_ramp_s + cfg.openloop_const_s;
    _ol_t_release       = cfg.openloop_release_s;
    // TRACK must outlast the OL hysteresis: if it expired first, a standstill
    // start would drive full iq on a garbage angle until the hysteresis fires.
    _resync_t           = (cfg.resync_time_s > cfg.openloop_hyst_s + 0.02f)
                              ? cfg.resync_time_s : cfg.openloop_hyst_s + 0.02f;
    // Lock dwell = half the TRACK window: long enough that an unconverged
    // observer's offset circle (sweeps out of the flux band each electrical
    // cycle down to ~500 eRPM) can't stay in band that long by accident.
    _lock_need          = uint16_t(0.5f * _resync_t / _dt);

    _obs_L          = 1.5f * cfg.motor_Ls;
    _obs_R          = 1.5f * cfg.motor_Rs;
    _obs_lambda     = cfg.motor_flux;
    _obs_lambda2    = cfg.motor_flux * cfg.motor_flux;
    _obs_gamma_half = 0.5f * cfg.observer_gain;
    _obs_gain_mod_inv   = (cfg.observer_gain_mod_full > 1e-3f) ? (1.0f / cfg.observer_gain_mod_full) : 1e6f;
    _obs_gain_slow_frac = cfg.observer_gain_slow_frac;

    _spd_kp    = cfg.speed_kp;
    _spd_ki_dt = cfg.speed_ki * _dt;
    _spd_ramp_erpm_s = cfg.speed_ramp_erpm_s;
    _pll_kp    = cfg.pll_kp;
    _pll_ki    = cfg.pll_ki;
    _cmd_timeout_ms = cfg.command_timeout_ms;
    _last_cmd_ms    = 0;

    _debug_phase_step = TWO_PI * cfg.debug_openloop_hz * _dt;
    _debug_max_mod    = cfg.debug_max_modulation;

    // ── Hall sensor config ─────────────────────────────────────────────────
    _sensor_mode   = cfg.sensor_mode;
    _hall_blend_lo = cfg.hall_blend_erpm_lo;
    _hall_blend_hi = (cfg.hall_blend_erpm_hi > cfg.hall_blend_erpm_lo + 1.0f)
                         ? cfg.hall_blend_erpm_hi : cfg.hall_blend_erpm_lo + 1.0f;
    for (uint8_t s = 0; s < 8; s++) {
        _hall_table[s] = isnan(cfg.hall_table_deg[s])
                             ? NAN : wrap_pi(cfg.hall_table_deg[s] * (PI_F / 180.0f));
    }
    _hall_breakaway_a = cfg.hall_breakaway_a;
    _hd_current       = (cfg.hall_detect_a < cfg.current_max) ? cfg.hall_detect_a : cfg.current_max;
    update_hall_table_valid();

    reset_control();
    _mode       = Mode::STOP;
    _fault_code = FAULT_NONE;
    _state      = State::IDLE;

    _initialized = true;
    return true;
#else
    (void)cfg;
    return false;
#endif
}

void MotorControl::reset_control()
{
    _integ_d = _integ_q = _integ_spd = 0.0f;
    _override_ang = 0.0f;
    _hyst_timer = 0.0f;
    _ol_timer = 0.0f;
    _ol_release = 0.0f;
    _track_timer = 0.0f;
    _lock_count = 0;
    _v_alpha_prev = _v_beta_prev = 0.0f;
    _id_filt = _iq_filt = 0.0f;
    _obs_x1 = _obs_x2 = _obs_theta = _obs_omega = 0.0f;
    _pll_theta = 0.0f;
    _stall_timer = 0.0f;
    // Hall decode re-fixes from scratch on the next valid read.
    _hall_state = HALL_NONE;
    _hall_raw_prev = _hall_deb = 0;
    _hall_omega = 0.0f;
    _hall_ticks = 0;
    _hall_fault = 0;
    // Motion must be re-confirmed after any stop/hold-off before full current is
    // released again (break-away clamp re-arms). reset_control() runs from
    // hold_off(), so every coast/idle/fault re-arms it.
    _hall_move_count = 0;
}

// Gate the output stage off (bare MOE clear) and mark the bridge disarmed so
// the next arm_bridge() re-applies the overcurrent blanking window. ISR-safe.
void MotorControl::gate_off()
{
    stm32_foc_motor_control_disable_outputs_isr();
    _outputs_on = false;
}

// Enforce the per-phase duty ceiling, then write the CCRs. Every drive path
// funnels through here so none can bypass the limit.
//
// The ceiling exists because the high side is bootstrapped: the MP1918 refills
// BST only while SW is pulled low, and its BST-SW ESD clamp actively bleeds the
// cap down, so a phase parked near 100% duty loses its high-side rail and drops
// the FET on UVLO with no warning to the firmware. The low-side conduction
// window it guarantees is also exactly what the shunt ADC needs to sample in.
//
// Excess is removed as a COMMON-MODE shift rather than by clipping the offending
// phase: the zero sequence is free in a 3-wire motor, so shifting all three
// equally preserves every line-to-line voltage — and therefore the applied
// vector — exactly, where per-phase clipping would distort it and make the
// current PI fight a disturbance it can't observe.
void MotorControl::write_duties(float da, float db, float dc)
{
    const float d_hi = fmaxf(da, fmaxf(db, dc));
    if (d_hi > _duty_max) {
        const float shift = d_hi - _duty_max;
        da -= shift;
        db -= shift;
        dc -= shift;
    }
    // Backstop. The vector is already bounded by _v_max (derived from _duty_max
    // in init), so a shift big enough to drive a phase negative shouldn't be
    // reachable — clip rather than hand a wrapped value to the CCR cast.
    da = clampf(da, 0.0f, _duty_max);
    db = clampf(db, 0.0f, _duty_max);
    dc = clampf(dc, 0.0f, _duty_max);

    const float pf = float(_period_ticks);
    stm32_foc_motor_control_write_pwm(uint16_t(da * pf), uint16_t(db * pf), uint16_t(dc * pf));
}

void MotorControl::arm_bridge()
{
    if (_outputs_on) {
        return;   // already driving — do NOT re-blank the overcurrent trip
    }
    _oc_over_count = 0;
    _oc_blank      = OC_BLANK_SAMPLES;
    _outputs_on    = true;
    stm32_foc_motor_control_enable_outputs();
}

// Thread context. See the header for the contract.
bool MotorControl::fault_gate(bool release, uint32_t now_ms)
{
    if (!_fault_latched) {
        return true;
    }
    if (!_trip_seen) {
        // Book the trip once, thread-side (the ISR only sets the latch).
        _trip_seen = true;
        const bool repeat = (_last_trip_ms != 0) && ((now_ms - _last_trip_ms) < TRIP_FORGET_MS);
        _trip_count   = repeat ? uint8_t((_trip_count < 255) ? _trip_count + 1 : 255) : 1;
        _last_trip_ms = now_ms;
        _rearm_ok_ms  = now_ms + ((_trip_count >= TRIP_MAX_CONSEC)
                                      ? TRIP_LOCKOUT_MS
                                      : TRIP_REARM_COOLDOWN_MS * _trip_count);
    }
    // A live (nonzero) command must NEVER clear a trip, and even a release only
    // counts once the cooldown has run — so a host that streams zero between
    // throttle applications still can't shorten the bridge-off time.
    if (!release || int32_t(now_ms - _rearm_ok_ms) < 0) {
        return false;
    }
    _fault_latched = false;
    _fault_code    = FAULT_NONE;
    _trip_seen     = false;
    // Park the drive so the re-arm after this release starts from zero rather
    // than from the set-point that was live when the fault tripped.
    _mode               = Mode::STOP;
    _cmd_current        = 0.0f;
    _cmd_current_target = 0.0f;
    return false;   // released on this call; the next command may arm
}

void MotorControl::trip_fault(uint8_t code)
{
    // ISR context: cut the output stage immediately via a bare MOE clear. The
    // latch keeps it off until the host releases (see fault_gate).
    gate_off();
    _fault_code    = code;
    _fault_latched = true;
    _fault_last    = code;                 // sticky, for host reporting
    _fault_last_ms = AP_HAL::millis();     // ISR-safe (systick read)
    _state         = State::FAULT;
    reset_control();
}

// See header. Holds a cleared trip in the reported field long enough for a
// polling host to actually see it.
uint8_t MotorControl::reported_fault() const
{
    if (_fault_code != FAULT_NONE) {
        return _fault_code;
    }
    if (_fault_last != FAULT_NONE &&
        (AP_HAL::millis() - _fault_last_ms) < FAULT_REPORT_HOLD_MS) {
        return _fault_last;
    }
    return FAULT_NONE;
}

// Bridge held off (STOP / latched fault): truly high-Z the outputs (MOE=0) so
// the rotor freewheels rather than being short-braked by PWM(0,0,0) — which
// would tie all three phases to GND via the low-side FETs and damp/jitter the
// rotor against cogging. Outputs are re-enabled by the next spin-up command.
void MotorControl::hold_off(State s)
{
    reset_control();
    gate_off();
    stm32_foc_motor_control_write_pwm(0, 0, 0);
    _state = s;
    _t_id = _t_iq = _t_vd = _t_vq = _t_duty = _t_erpm = 0.0f;
    _v_alpha_prev = _v_beta_prev = 0.0f;
    _id_filt = _iq_filt = 0.0f;
}

void MotorControl::enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (_initialized) {
        arm_bridge();
    }
#endif
}

void MotorControl::disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (_initialized) {
        _outputs_on = false;
        stm32_foc_motor_control_disable_outputs();
    }
#endif
}

// FET thermal protection off the board NTC (3V3 → 10k → PB12 → NTC 10k B3435
// → GND). Linearly derates the iq limit from full at fet_temp_start to zero at
// fet_temp_max, where an over-temp fault also trips (released 10°C lower).
// Thread context; the logf is throttled to 10 Hz.
void MotorControl::update_thermal(uint32_t now_ms)
{
    if (!_current_sense_initialized || (now_ms - _last_temp_ms) < 100U) {
        return;
    }
    _last_temp_ms = now_ms;

    const float ratio = stm32_foc_temp_read_ratio();  // Vpin/Vref
    if (ratio < 0.01f || ratio > 0.99f) {
        // Open/shorted sensor (e.g. NTC unpopulated → pulled to Vref): can't
        // measure, don't derate — protection is advisory, absence must not
        // brick the drive.
        _i_derate     = 1.0f;
        _thermal_trip = false;
        return;
    }
    // Low-side NTC: ratio = Rntc/(Rntc+10k) → Rntc/10k = ratio/(1-ratio),
    // then the Beta equation against 25°C/10k.
    const float r_rel  = ratio / (1.0f - ratio);
    const float temp_c = 1.0f / (1.0f / 298.15f + logf(r_rel) / 3435.0f) - 273.15f;
    _t_fet_temp = temp_c;

    _i_derate = clampf((_fet_t_max - temp_c) / (_fet_t_max - _fet_t_start), 0.0f, 1.0f);
    if (temp_c >= _fet_t_max) {
        _thermal_trip = true;
    } else if (temp_c < _fet_t_max - 10.0f) {
        _thermal_trip = false;
    }
}

float MotorControl::peak_phase_current() const
{
    const float a = fabsf(_t_ia), b = fabsf(_t_ib), c = fabsf(_t_ic);
    const float ab = a > b ? a : b;
    return ab > c ? ab : c;
}

// A switch between the two active setpoint controllers (CURRENT/SPEED) is only
// safe near zero torque — otherwise the incoming loop can step-demand a large
// (possibly braking) current against a spinning rotor, which is exactly the
// event that destroyed a leg. Arming from STOP and same-mode setpoint updates
// are always allowed; BRAKE is not routed through here (it is regen-limited).
bool MotorControl::mode_change_allowed(Mode target) const
{
    if (_mode == target || _mode == Mode::STOP) {
        return true;
    }
    return peak_phase_current() < _mode_switch_i;
}

void MotorControl::set_current(float amps)
{
    const uint32_t now_ms = AP_HAL::millis();
    _last_cmd_ms = now_ms;            // still a live host even when held off
    const bool release = fabsf(amps) < 0.01f;

    // Clear the start lockout HERE, before the fault gate, not in the release
    // block further down. fault_gate() returns false on EVERY release path —
    // including the one that successfully clears the fault — so set_current()
    // returns early and never reached the clear below. The result was a
    // permanently unclearable lockout: the fault code cleared (telemetry then
    // reported fault=0), but every subsequent nonzero command was still
    // silently refused at the _ol_locked_out check, leaving the controller in
    // IDLE with no torque, no fault, and no way out but a power cycle.
    if (release) {
        _ol_locked_out = false;
        _ol_attempts   = 0;
    }

    // A trip is a shutdown, not a blip. Only a zero/stop command clears one, and
    // only after the re-arm cooldown — a nonzero command never does. Clearing on
    // every command re-armed the bridge into live faults at the ~1 kHz command
    // rate, which is what destroyed the previous power stages.
    if (!fault_gate(release, now_ms)) {
        return;
    }
    // Sensorless-start lockout: after too many failed forced starts the bridge is
    // latched off (thermal cap). A nonzero command must NOT clear it — otherwise
    // the arbiter re-commanding every loop would retry forever. Only a zero/stop
    // command (below) releases it.
    if (_ol_locked_out && !release) {
        return;
    }
    if (release) {
        // (lockout already cleared above, before the fault gate)
        // Zero command: while running keep the loop alive at iq=0 (current loop
        // drives vd/vq to hold zero current → smooth coast, no bridge cliff-cut).
        // From idle/fault, stay idle so a zero command can't spin the motor up.
        // Leave _cmd_current to slew down to 0 for a controlled release; only
        // force it to 0 when fully stopping so a later re-arm starts from zero.
        _cmd_current_target = 0.0f;
        _mode = (_state == State::CLOSED || _state == State::OPENLOOP) ? Mode::CURRENT : Mode::STOP;
        if (_mode == Mode::STOP) {
            _cmd_current = 0.0f;
        }
        return;
    }
    // Refuse a hot swap from another active controller under load; the host
    // must coast (command 0) first so current decays before re-arming.
    if (!mode_change_allowed(Mode::CURRENT)) {
        return;
    }
    if (_mode != Mode::CURRENT) {
        _cmd_current = 0.0f;                     // (re)entry: slew up from zero, not from a stale value
    }
    _cmd_current_target = clampf(amps, -_current_max, _current_max);
    _mode = Mode::CURRENT;                      // set mode first so a racing ISR sees CURRENT not STOP
    arm_bridge();                               // re-arm bridge if previously released
}

// VESC COMM_SET_CURRENT_BRAKE: apply iq opposite to rotation for regenerative
// braking. Loop stays active so back-EMF energy returns to the bus in a controlled
// way (no high-Z body-diode rectification). Auto-releases the bridge once speed
// drops below the safe-release threshold (see adc_sample_isr).
void MotorControl::set_brake_current(float amps)
{
    const uint32_t now_ms = AP_HAL::millis();
    _last_cmd_ms = now_ms;
    const float mag = fabsf(amps);
    if (mag < 0.01f) {           // zero brake ≡ coast (and releases a latched trip)
        set_current(0.0f);
        return;
    }
    if (!fault_gate(false, now_ms)) {
        return;                  // latched trip: a live brake command must not clear it
    }
    _cmd_current = (mag > _regen_max) ? _regen_max : mag;  // magnitude, capped to regen limit
    _mode = Mode::BRAKE;
}

void MotorControl::set_rpm(float erpm)
{
    const uint32_t now_ms = AP_HAL::millis();
    _last_cmd_ms = now_ms;
    if (fabsf(erpm) < 1.0f) {    // zero command = explicit stop (also clears a latched fault)
        stop();
        return;
    }
    // A live RPM command must not clear a latched trip: doing so armed the bridge
    // for one cycle per packet before the fault path cut it again — a short-brake
    // blip at the command rate. An RPM-streaming host recovers by commanding zero
    // (or stop), which releases the latch once the cooldown has run.
    if (!fault_gate(false, now_ms)) {
        return;
    }
    // Refuse a hot swap from another active controller under load (see set_current).
    if (!mode_change_allowed(Mode::SPEED)) {
        return;
    }
    _cmd_erpm = erpm;
    _mode = Mode::SPEED;
    arm_bridge();
}

void MotorControl::check_command_timeout(uint32_t now_ms)
{
    if (_mode != Mode::STOP && (now_ms - _last_cmd_ms) > _cmd_timeout_ms) {
        _mode = Mode::STOP;   // host went quiet → coast (fault state left untouched)
    }
}

void MotorControl::notify_host_alive()
{
    _last_cmd_ms = AP_HAL::millis();
}

// Explicit host stop: the one command that clears everything immediately —
// latched trip, re-arm cooldown and the repeat-trip history. This is the
// deliberate operator-level reset; ordinary throttle release goes through
// fault_gate() and still has to wait out the cooldown.
void MotorControl::stop()
{
    _mode = Mode::STOP;
    _cmd_current        = 0.0f;
    _cmd_current_target = 0.0f;
    _fault_code    = FAULT_NONE;
    _fault_latched = false;
    _fault_last    = FAULT_NONE;   // explicit stop also clears the sticky report
    _trip_seen     = false;
    _trip_count    = 0;
    _last_trip_ms  = 0;
    _rearm_ok_ms   = 0;
    _ol_attempts   = 0;         // fresh sensorless-start retry budget
    _ol_cooldown   = 0.0f;
    _ol_locked_out = false;
}

void MotorControl::set_debug_voltage(float duty)
{
    const uint32_t now_ms = AP_HAL::millis();
    _last_cmd_ms = now_ms;
    const float a = fabsf(duty);
    // Same rule as the real set-points: a zero-duty command releases a latched
    // trip (after the cooldown), a live one never does.
    if (!fault_gate(a < 0.001f, now_ms)) {
        return;
    }
    if (a < 0.001f) {
        _mode = Mode::STOP;
        return;
    }
    _debug_mod = (a > _debug_max_mod) ? _debug_max_mod : a;
    _debug_dir = (duty >= 0.0f) ? 1.0f : -1.0f;
    _mode = Mode::DEBUG_VOLTAGE;
    arm_bridge();
}

// Play a tone through the motor (see header). A fixed-axis (α) voltage vector
// modulated at freq_hz vibrates the windings without net rotation. Amplitude is
// capped and the frequency floored so a mistaken low/DC tone can't dump current
// through the ~55 mΩ winding resistance (the hard OC trip still backstops it).
void MotorControl::play_tone(float freq_hz, float amplitude, uint16_t duration_ms)
{
    // A tone is never a reason to re-arm a tripped bridge — refuse while a fault
    // is latched rather than clearing it (the power-on chime in particular must
    // not put current back into a faulted power stage).
    if (!_initialized || duration_ms == 0 || _fault_latched) {
        return;
    }
    if (freq_hz < 200.0f) freq_hz = 200.0f;   // keep inductive reactance meaningful
    _beep_phase      = 0.0f;
    _beep_phase_step = TWO_PI * freq_hz * _dt;
    _beep_amp        = clampf(amplitude, 0.0f, _debug_max_mod);
    _beep_ticks_left = uint32_t(duration_ms) * (_pwm_update_rate_hz / 1000U);
    _mode            = Mode::BEEP;
    arm_bridge();
}

// ── ISR path ────────────────────────────────────────────────────────────────

void MotorControl::adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v, uint16_t sample_w)
{
    static_cast<MotorControl *>(ctx)->adc_sample_isr(sample_u, sample_v, sample_w);
}

// Full FOC cycle, once per PWM period (ADC injected-EOC ISR).
void MotorControl::adc_sample_isr(uint16_t sample_u, uint16_t sample_v, uint16_t sample_w)
{
    // Live raw hall state for diagnostics — updated every cycle regardless of
    // drive state, so the state can be watched while turning the rotor by hand
    // (motor stopped). HALL-run mode overwrites this with the debounced state.
    _t_hall_state = read_hall_state();

    // ── Zero-current calibration (outputs gated, no current flowing) ────────
    if (!_current_zero_valid) {
        // Don't latch a baseline until the sense front-end (INA181 ref) is
        // powered; restart accumulation if the rail dips mid-calibration.
        // Only U/V gate the loop (control depends on them); W is baselined in
        // parallel but a missing W amp must not stall start-up.
        if (sample_u < SENSE_ALIVE_COUNTS || sample_v < SENSE_ALIVE_COUNTS) {
            _zero_accum[0] = _zero_accum[1] = _zero_accum[2] = 0;
            _zero_count = 0;
            return;
        }
        _zero_accum[0] += sample_u;
        _zero_accum[1] += sample_v;
        _zero_accum[2] += sample_w;
        if (++_zero_count == ZERO_SAMPLES) {
            _current_zero_raw[0] = uint16_t(_zero_accum[0] / ZERO_SAMPLES);
            _current_zero_raw[1] = uint16_t(_zero_accum[1] / ZERO_SAMPLES);
            _current_zero_raw[2] = uint16_t(_zero_accum[2] / ZERO_SAMPLES);
            // W is only trusted if its baseline sits in the INA-ref band; a
            // floating/unpopulated input (rails near 0 or full-scale) → fall
            // back to the derived -(ia+ib) and report a zero residual.
            _w_sense_valid = (_current_zero_raw[2] > SENSE_ALIVE_COUNTS &&
                              _current_zero_raw[2] < SENSE_REF_MAX);
            _current_zero_valid  = true;
        }
        return;
    }
    _adc_sample_cb_count++;

    // ── Bus voltage tracking ────────────────────────────────────────────────
    // Recompute the vbus-dependent constants each cycle so the volts→duty
    // conversion and the observer's assumed applied voltage stay correct
    // whatever the actual supply is.
    //
    // The filter tracks ASYMMETRICALLY: slow up (τ ≈ 5 ms, for noise immunity on
    // the modulation math) but fast down (τ ≈ 0.5 ms). A supply hitting its
    // current limit collapses in well under 5 ms, and lagging that sag means
    // computing duties against a bus that is no longer there. Erring toward the
    // lower reading is also the safe direction — it under-modulates.
    const float vraw = stm32_foc_vbus_read_volts();
    {
        _vbus_raw_v = vraw;
        if (vraw > 6.0f) {
            if (!_vbus_valid) {
                // First plausible reading: SNAP the filter to it. _vbus_flt is
                // seeded with the configured nominal, so ramping from that seed
                // would drag the derived limits (_v_max, uv_scale) through a
                // transient that never physically happened.
                _vbus_valid = true;
                _vbus_flt   = vraw;
            } else {
                _vbus_flt += (vraw - _vbus_flt) * ((vraw < _vbus_flt) ? 0.1f : 0.01f);
            }
        }
        // Measurement-readiness latch: the RC-filtered divider needs ~5 ms to
        // settle, and nothing else in start-up waits for it (see
        // VBUS_READY_SAMPLES). Until the bus has read healthy continuously, a low
        // sample means "not measured yet" and must not arm the protections.
        if (!_vbus_ready) {
            if (vraw >= _vbus_min) {
                if (++_vbus_ready_count >= VBUS_READY_SAMPLES) {
                    _vbus_ready = true;
                }
            } else {
                _vbus_ready_count = 0;
            }
        }
        const float inv_vbus = 1.0f / _vbus_flt;
        _inv_vbus_half = 2.0f * inv_vbus;
        _v_max         = _mod_to_vmax * _vbus_flt;
        _vbus          = _vbus_flt;
    }
    // Bus-OV regen foldback: scales any decelerating (bus-charging) current
    // from full at (vbus_max - band) to zero at vbus_max.
    const float ov_scale = clampf((_vbus_max - _vbus_flt) * _vbus_fold_inv, 0.0f, 1.0f);
    // Bus-UV foldback: the mirror image, on MOTORING current — that is what
    // loads the supply. As the bus sags toward vbus_min the torque command is
    // scaled back, which unloads the supply and lets it recover: a negative
    // feedback that settles at whatever the PSU can actually deliver, instead of
    // collapsing into the hard trip below and oscillating trip/re-arm. Uses its
    // own band (see Config::vbus_uv_fold_band) — sharing the OV band put the
    // onset within a volt of nominal and quietly derated torque on any sag.
    // Gated on _vbus_ready for the same reason as the trip: before the divider
    // has settled, derating against an unsettled reading would silently crush the
    // torque limit toward zero on a bus that is actually fine.
    const float uv_scale = _vbus_ready
                               ? clampf((_vbus_flt - _vbus_min) * _vbus_uvfold_inv, 0.0f, 1.0f)
                               : 1.0f;
    // Thermal derate of the iq limit (thread-computed from the board NTC).
    const float i_max = _current_max * _i_derate * uv_scale;

    // ── Phase currents ──────────────────────────────────────────────────────
    // Low-side shunt polarity: positive phase current pulls the amplified ADC
    // reading BELOW the zero reference, so current = (zero - sample).
    const float ia = float(int32_t(_current_zero_raw[0]) - int32_t(sample_u)) * ADC_LSB_VOLTS * _current_scale;
    const float ib = float(int32_t(_current_zero_raw[1]) - int32_t(sample_v)) * ADC_LSB_VOLTS * _current_scale;
    // Prefer the directly-measured W current; the sum ia+ib+ic then becomes an
    // independent health check. Without a live W amp, reconstruct as before.
    float ic;
    if (_w_sense_valid) {
        const float ic_meas = float(int32_t(_current_zero_raw[2]) - int32_t(sample_w)) * ADC_LSB_VOLTS * _current_scale;
        _t_i_resid = ia + ib + ic_meas;
        ic = ic_meas;
    } else {
        _t_i_resid = 0.0f;
        ic = -(ia + ib);
    }
    _t_ia = ia;
    _t_ib = ib;
    _t_ic = ic;

    // ── Hard overcurrent trip ───────────────────────────────────────────────
    // Two tiers. The instant tier (_oc_trip_hard) has no debounce and is NOT
    // blanked — arming straight into a short must be caught within one sample,
    // not after the 800 µs blank window. The debounced tier (_oc_trip) rejects
    // single noisy samples: it skips the first few samples after the bridge
    // arms (switching-transient inrush on the sense line), then requires
    // OC_DEBOUNCE consecutive over-limit samples before tripping.
    const float ia_abs = fabsf(ia), ib_abs = fabsf(ib), ic_abs = fabsf(ic);
    const float i_pk   = ia_abs > ib_abs ? (ia_abs > ic_abs ? ia_abs : ic_abs)
                                         : (ib_abs > ic_abs ? ib_abs : ic_abs);
    if (i_pk > _oc_trip_hard) {
        trip_fault(FAULT_ABS_OVERCURRENT);
    } else if (_oc_blank > 0) {
        _oc_blank--;
        _oc_over_count = 0;
    } else if (i_pk > _oc_trip) {
        if (++_oc_over_count >= OC_DEBOUNCE) {
            trip_fault(FAULT_ABS_OVERCURRENT);
        }
    } else {
        _oc_over_count = 0;
    }

    // ── Bus under-voltage trip ──────────────────────────────────────────────
    // Checked only while the bridge is armed, and against the RAW sample rather
    // than _vbus_flt: a collapse outruns even the fast-tracking filter, and this
    // is the one protection that must not lag. Debounced a few samples so a
    // single noisy conversion can't gate the drive.
    //
    // There is no lower guard on vraw — a reading near zero while driving is
    // itself the fault (either the bus is gone or the divider has failed), and
    // in both cases the safe response is the same: stop driving.
    if (_outputs_on && _vbus_ready) {
        // Record what the trip actually saw — the single most useful number when
        // an under-voltage fires unexpectedly, since the filtered value reported
        // to the host can differ from this by a lot.
        if (vraw < _vbus_min_seen) {
            _vbus_min_seen = vraw;
        }
        if (vraw < _vbus_min) {
            if (++_uv_count >= UV_DEBOUNCE) {
                trip_fault(FAULT_UNDER_VOLTAGE);
                return;
            }
        } else {
            _uv_count = 0;
        }
    } else {
        _uv_count = 0;
    }

    float i_alpha, i_beta;
    FOC::clarke(ia, ib, i_alpha, i_beta);

    // Observer always runs (using the voltage applied last cycle).
    observer_update(_v_alpha_prev, _v_beta_prev, i_alpha, i_beta);

    const Mode mode = _mode;
    // Detect a fresh entry into SPEED (e.g. CURRENT→SPEED while spinning) so the
    // setpoint can be seeded to the actual speed below. Without this the speed
    // error starts from a stale setpoint and the loop step-demands a hard brake.
    const bool speed_entry = (mode == Mode::SPEED && _prev_mode != Mode::SPEED);
    _prev_mode = mode;

    // ── Over-temp: re-trips every cycle while hot, so a streaming host that
    //    clears the fault code with each command can't keep the bridge armed.
    if (_thermal_trip && _fault_code == FAULT_NONE) {
        trip_fault(FAULT_OVER_TEMP_FET);
    }
    // ── Latched fault → bridge stays off until cleared by a new command ─────
    if (_fault_code != FAULT_NONE) {
        hold_off(State::FAULT);
        return;
    }
    // ── Stopped / coasting ──────────────────────────────────────────────────
    if (mode == Mode::STOP) {
        hold_off(State::IDLE);
        return;
    }

    // ── Audio beep: fixed-axis (α) voltage modulated at the tone frequency ────
    // The windings vibrate as a speaker; a symmetric AC on one axis makes no net
    // torque so the rotor doesn't spin. No current loop / observer involvement.
    if (mode == Mode::BEEP) {
        if (_beep_ticks_left == 0) {
            _mode = Mode::STOP;        // tone done → coast (silent) next cycle
            hold_off(State::IDLE);
            return;
        }
        _beep_ticks_left--;
        _beep_phase = wrap_pi(_beep_phase + _beep_phase_step);
        const float m_alpha = _beep_amp * sinf(_beep_phase);
        const float m_beta  = 0.0f;

        float va, vb, vc;
        FOC::inv_clarke(m_alpha, m_beta, va, vb, vc);
        float da, db, dc;
        FOC::svpwm(va, vb, vc, da, db, dc);
        write_duties(da, db, dc);

        const float vbus_half = _vbus * 0.5f;
        _v_alpha_prev = m_alpha * vbus_half;   // applied volts → observer next cycle
        _v_beta_prev  = m_beta  * vbus_half;
        _state  = State::BEEP;
        _t_duty = _beep_amp;
        _t_erpm = 0.0f;
        return;
    }

    // ── Open-loop voltage debug mode (current loop + observer-control off) ───
    if (mode == Mode::DEBUG_VOLTAGE) {
        _debug_theta = wrap_pi(_debug_theta + _debug_dir * _debug_phase_step);
        const float st = sinf(_debug_theta);
        const float ct = cosf(_debug_theta);
        const float m_alpha = _debug_mod * ct;
        const float m_beta  = _debug_mod * st;

        float va, vb, vc;
        FOC::inv_clarke(m_alpha, m_beta, va, vb, vc);
        float da, db, dc;
        FOC::svpwm(va, vb, vc, da, db, dc);
        write_duties(da, db, dc);

        const float vbus_half = _vbus * 0.5f;
        _v_alpha_prev = m_alpha * vbus_half;   // applied volts → observer next cycle
        _v_beta_prev  = m_beta  * vbus_half;

        // Currents resolved against the *commanded* angle: applying voltage on
        // +d should drive id>0. Wrong sign/scale shows up here immediately.
        float id, iq;
        FOC::park(i_alpha, i_beta, st, ct, id, iq);
        _integ_d = _integ_q = _integ_spd = 0.0f; // keep loop clean for later
        _state   = State::DEBUG;
        _t_id    = id;
        _t_iq    = iq;
        _t_vd    = _debug_mod * vbus_half;
        _t_vq    = 0.0f;
        _t_duty  = _debug_mod;
        _t_erpm  = _obs_omega * _w_to_erpm;
        _t_theta = _debug_theta;
        return;
    }

    // ── Hall-table detection (VESC mcpwm_foc_hall_detect style) ─────────────
    //    Current-controlled forced rotation: hold a d-axis current at a forced
    //    electrical angle and sweep the angle slowly — forward for the first
    //    half, reverse for the second. The rotor's PM follows the field like a
    //    stepper; averaging both directions cancels the hall switching-hysteresis
    //    bias so each recorded sector centre is the true mid-point. Current is
    //    regulated (not fixed voltage) so an unloaded low-R motor can't draw a
    //    large spin current. Fills the live hall table and coasts when finished.
    if (mode == Mode::HALL_DETECT) {
        const uint32_t half       = uint32_t(HD_REVS / (HD_HZ * _dt));
        const uint32_t ramp_ticks = uint32_t(HD_RAMP_S / _dt);
        // Ramp the detect current in over the first ramp_ticks (with the angle
        // held) so the rotor first aligns to angle 0 without a torque step.
        const float ramp      = clampf(float(_hd_ticks) / float(ramp_ticks), 0.0f, 1.0f);
        const float id_target = _hd_current * ramp;
        if (_hd_ticks >= ramp_ticks) {
            const float dir_hd = (_hd_ticks < ramp_ticks + half) ? 1.0f : -1.0f;
            _hd_angle = wrap_pi(_hd_angle + dir_hd * TWO_PI * HD_HZ * _dt);
        }
        const float st = sinf(_hd_angle);
        const float ct = cosf(_hd_angle);

        // d-axis current PI on the forced angle (iq target 0), same gains and
        // anti-windup / vector clamp as the main current loop below.
        float id_m, iq_m;
        FOC::park(i_alpha, i_beta, st, ct, id_m, iq_m);
        _id_filt += (id_m - _id_filt) * CURRENT_FILT_K;
        _iq_filt += (iq_m - _iq_filt) * CURRENT_FILT_K;
        _integ_d = clampf(_integ_d + (id_target - id_m) * _cur_ki_dt, -_v_max, _v_max);
        _integ_q = clampf(_integ_q + (0.0f      - iq_m) * _cur_ki_dt, -_v_max, _v_max);
        float vd = clampf((id_target - id_m) * _cur_kp + _integ_d,
                          -_v_max * 0.7071068f, _v_max * 0.7071068f);
        float vq = (0.0f - iq_m) * _cur_kp + _integ_q;
        const float vqr = sqrtf(_v_max * _v_max - vd * vd);
        vq = clampf(vq, -vqr, vqr);

        float v_alpha, v_beta;
        FOC::inv_park(vd, vq, st, ct, v_alpha, v_beta);
        const float m_alpha = v_alpha * _inv_vbus_half;
        const float m_beta  = v_beta  * _inv_vbus_half;

        float va, vb, vc, da, db, dc;
        FOC::inv_clarke(m_alpha, m_beta, va, vb, vc);
        FOC::svpwm(va, vb, vc, da, db, dc);
        // Same dead-time treatment as the main loop — VESC's hall detect reuses
        // control_current() outright, so it gets this for free; ours duplicates
        // the modulation, and the two must not drift apart. This sweep runs at
        // the lowest modulation of any mode, so it is where the uncompensated
        // dead-band distorts the applied vector most.
        dt_comp_apply_duties(st, ct, da, db, dc);
        write_duties(da, db, dc);
        float dv_alpha_hd, dv_beta_hd;
        dt_comp_alpha_beta(st, ct, dv_alpha_hd, dv_beta_hd);
        _v_alpha_prev = v_alpha - dv_alpha_hd;  // keep the observer input sane during the spin
        _v_beta_prev  = v_beta  - dv_beta_hd;

        // Record ALL states 0..7 (the valid six are motor-specific — this motor
        // uses {0,1,2,5,6,7}, others {1..6} — we don't presume which), but only
        // after the current has ramped in, to skip the initial alignment step.
        if (ramp >= 1.0f) {
            const uint8_t s = read_hall_state();
            _hd_sin[s] += st;
            _hd_cos[s] += ct;
            _hd_n[s]++;
        }
        _state = State::HALL_DETECT;

        if (++_hd_ticks >= ramp_ticks + 2 * half) {
            // A real state is dwelt in for a full 60° sector each rev; a glitch
            // state gets only a handful of samples. Keep states with a meaningful
            // share of the busiest state's count (threshold = max/4).
            uint32_t nmax = 0;
            for (uint8_t k = 0; k < 8; k++) {
                if (_hd_n[k] > nmax) nmax = _hd_n[k];
            }
            const uint32_t nmin = nmax / 4;
            // Circular mean of the forced angle per state = sector centre.
            for (uint8_t k = 0; k < 8; k++) {
                const bool valid = (_hd_n[k] > nmin);
                const float ang  = valid ? atan2f(_hd_sin[k], _hd_cos[k]) : NAN;
                _hall_table[k]        = ang;
                _hall_detect_deg[k]   = valid ? (ang * (180.0f / PI_F)) : NAN;
            }
            update_hall_table_valid();   // newly detected table may now be drivable
            _hall_detect_done  = true;
            _hall_detect_fresh = true;   // one-shot: consumed by take_hall_detect_result()
            _mode = Mode::STOP;
            hold_off(State::IDLE);
        }
        return;
    }

    // Slew the CURRENT-mode torque setpoint toward its target (VESC l_current_ramp
    // equivalent) so a step throttle command can't apply an instant iq reference
    // jump. Done before the coast test below, which reads _cmd_current: a
    // commanded 0 ramps down to true zero, then coasts/releases. BRAKE writes
    // _cmd_current directly as a magnitude and SPEED ignores it, so slew CURRENT
    // only. i_max clamp at the iq_cmd site still bounds the derated ceiling.
    if (mode == Mode::CURRENT) {
        _cmd_current = (_i_slew_per_tick > 0.0f)
                           ? step_towards(_cmd_current, _cmd_current_target, _i_slew_per_tick)
                           : _cmd_current_target;
    }

    // ── Hall sensors (sensored mode): decode + speed every cycle ────────────
    // The observer still free-runs above (for the high-speed blend), but the
    // control speed comes from hall transition timing, which is valid from the
    // first revolution — no sensorless floor.
    if (_sensor_mode == SensorMode::HALL) {
        // Fail-safe arming: never commutate on an unvalidated table. Without all
        // six real states mapped (fresh board, or params never populated), the
        // angle would be wrong → jerky drive + hard fault-cuts that stress the
        // GaN stage. Refuse to drive (high-Z idle) until a HALL_DETECT run (or
        // loaded params) supplies a real table. Reached only for drive modes;
        // HALL_DETECT itself returns earlier, so detection is unaffected.
        if (!_hall_table_valid) {
            hold_off(State::IDLE);
            return;
        }
        if (update_hall()) {
            _hall_fault = 0;
        } else if (++_hall_fault > HALL_FAULT_SAMPLES) {
            trip_fault(FAULT_HALL_SENSOR);
            return;
        }
    }
    // Unified control-speed estimate: hall timing when sensored, observer PLL
    // otherwise. Used by the shared coast/regen/speed-loop logic below.
    const float omega_ctrl = (_sensor_mode == SensorMode::HALL) ? _hall_omega : _obs_omega;

    // ── Coast (CURRENT @ iq=0) / Brake: hold the loop alive, never let OL
    //    re-arm and re-spin the motor, auto-release once safely slow. Without
    //    this, a coasting motor that drops below the OL hysteresis threshold
    //    would be kicked back up by the boost sequence → endless OL/CLOSED loop.
    const bool coasting = (mode == Mode::BRAKE) ||
                          (mode == Mode::CURRENT && _cmd_current == 0.0f);
    if (coasting) {
        _ol_cooldown = 0.0f;                             // command released → drop
        _ol_attempts = 0;                                // any pending retry/cooldown
        constexpr float RELEASE_OMEGA = 10.47f;          // ~100 eRPM (rad/s)
        if (fabsf(omega_ctrl) < RELEASE_OMEGA) {
            hold_off(State::IDLE);                       // safe to high-Z now
            return;
        }
        _ol_timer = 0.0f;
        _ol_release = 0.0f;
        _hyst_timer = 0.0f;
    }

    // Inter-attempt cooldown after an abandoned sensorless start: hold the bridge
    // high-Z for openloop_cooldown_s so the windings shed heat before another
    // forced try (no motor-temp sensor). The retry counters survive reset_control()
    // so the budget carries across; when the timer expires we fall through and the
    // TRACK/hysteresis path below re-arms the next forced attempt.
    if (_ol_cooldown > 0.0f) {
        _ol_cooldown -= _dt;
        gate_off();
        stm32_foc_motor_control_write_pwm(0, 0, 0);
        _state = State::IDLE;
        _t_id = _t_iq = _t_vd = _t_vq = _t_duty = 0.0f;
        return;
    }

    // First running cycle after STOP / fault / debug → enter the TRACK phase
    // (State::ALIGN): closed loop at iq=0. The rotor state is unknown here — a
    // stop is a high-Z coast and hold_off() kept the observer zeroed, so the
    // rotor may still be freewheeling fast. Forcing open loop against it (the
    // old behaviour) kicked and tripped overcurrent. With iq nulled the applied
    // volts ≈ back-EMF and the observer converges to the true angle/speed:
    // spinning → ω rises above the OL threshold and closed loop catches
    // seamlessly; standstill → the hysteresis fires the OL sequence as before.
    // (Sensorless startup bookkeeping — hall mode commutates from standstill and
    // skips the whole I/f sequence, so this only arms in SENSORLESS mode.)
    if (_sensor_mode == SensorMode::SENSORLESS &&
        _state != State::OPENLOOP && _state != State::CLOSED &&
        _state != State::ALIGN && !coasting) {
        _override_ang = _obs_theta;
        _integ_spd    = 0.0f;
        _hyst_timer   = 0.0f;
        _track_timer  = _resync_t;
        _lock_count   = 0;
        _spd_set_erpm = _obs_omega * _w_to_erpm;  // VESC: init setpoint to current speed
    }

    const float dir = (mode == Mode::SPEED)
                          ? (_cmd_erpm >= 0.0f ? 1.0f : -1.0f)
                          : (_cmd_current >= 0.0f ? 1.0f : -1.0f);

    // ── Outer command: torque current (speed PI for RPM mode, direct otherwise)
    float iq_cmd;
    if (mode == Mode::SPEED) {
        // On a fresh SPEED entry, seed the ramped setpoint to the current speed
        // (and clear the integrator) so the takeover error ≈ 0 — no step brake.
        if (speed_entry) {
            _spd_set_erpm = omega_ctrl * _w_to_erpm;
            _integ_spd    = 0.0f;
        }
        // VESC-style ramped setpoint (foc_run_pid_control_speed): slew toward the
        // command at an accel limit, and while still in open loop clamp it to the
        // handover speed. So at handover the setpoint ≈ actual speed (no error step
        // → no kick) and afterwards it ramps up under control (observer/PLL keep
        // up → no desync on big speed commands).
        _spd_set_erpm = step_towards(_spd_set_erpm, _cmd_erpm, _spd_ramp_erpm_s * _dt);
        float set_erpm = _spd_set_erpm;
        if (_ol_timer > 0.0f) {
            set_erpm = clampf(set_erpm, -_open_handover_erpm, _open_handover_erpm);
        }
        const float erpm_err = set_erpm - omega_ctrl * _w_to_erpm;
        _integ_spd = clampf(_integ_spd + erpm_err * _spd_ki_dt, -i_max, i_max);
        iq_cmd = clampf(erpm_err * _spd_kp + _integ_spd, -i_max, i_max);
        // Regen limit: when iq opposes rotation (decelerating) the braking energy
        // returns to the bus, which a bench PSU can't sink — cap the braking
        // current hard, folded toward zero as vbus approaches vbus_max.
        if (iq_cmd * omega_ctrl < 0.0f) {
            const float rl = _regen_max * ov_scale;
            iq_cmd = clampf(iq_cmd, -rl, rl);
        }
    } else if (mode == Mode::BRAKE) {
        // _cmd_current holds the brake magnitude (already capped to _regen_max);
        // sign opposes rotation. Clamp again here as the single enforcement
        // point, folded by the bus-OV scale.
        const float rl = _regen_max * ov_scale;
        iq_cmd = clampf((omega_ctrl >= 0.0f ? -1.0f : 1.0f) * _cmd_current, -rl, rl);
    } else {
        iq_cmd = clampf(_cmd_current, -i_max, i_max);
        // CURRENT mode has no steady regen cap by design, but a decelerating
        // command must still fold back rather than pump the bus past vbus_max.
        if (iq_cmd * omega_ctrl < 0.0f) {
            const float rl = i_max * ov_scale;
            iq_cmd = clampf(iq_cmd, -rl, rl);
        }
    }

    // ── Angle + torque source ───────────────────────────────────────────────
    // HALL commutates from the hall angle (interpolated), blending to the
    // observer at speed. SENSORLESS runs the I/f-start state machine, which may
    // gate the bridge off (failed start) — then it returns false and we abort.
    float theta;
    float id_set = 0.0f;
    float iq_set = iq_cmd;

    if (_sensor_mode == SensorMode::HALL) {
        // Blend hall → observer angle across [_hall_blend_lo, _hall_blend_hi]
        // eRPM (VESC foc_sl_erpm): pure hall from standstill, pure observer at
        // speed, short-way interpolation of the wrapped angle difference between.
        const float erpm_abs = fabsf(_hall_omega) * _w_to_erpm;
        const float k = mapf(erpm_abs, _hall_blend_lo, _hall_blend_hi, 0.0f, 1.0f);
        theta = (k <= 0.0f)
                    ? _hall_theta
                    : wrap_pi(_hall_theta + k * wrap_pi(_obs_theta - _hall_theta));
        iq_set = iq_cmd;   // torque straight through; no open-loop boost needed
        // Break-away clamp: until the rotor has demonstrably moved (>= a couple
        // of hall transitions) cap torque current to a low value. A wrong/mis-
        // calibrated table or a jam then can't dump full current into a stationary
        // rotor — the current that can be held at a bad angle is bounded, and the
        // HALL stall trip below cuts it entirely within _stall_t if it never moves.
        if (_hall_move_count < HALL_BREAKAWAY_N) {
            iq_set = clampf(iq_set, -_hall_breakaway_a, _hall_breakaway_a);
        }
        _state = State::HALL;
    } else if (!run_sensorless(mode, dir, iq_cmd, i_alpha, i_beta, i_max, coasting,
                               theta, id_set, iq_set)) {
        return;   // sensorless start abandoned this cycle — bridge already gated off
    }

    // ── dq current PI (Kp = L·ωbw, Ki = R·ωbw) with anti-windup ─────────────
    const float sin_t = sinf(theta);
    const float cos_t = cosf(theta);
    float id, iq;
    FOC::park(i_alpha, i_beta, sin_t, cos_t, id, iq);
    // Low-pass the dq currents purely to give the dead-time correction a stable
    // sign (VESC id_filter/iq_filter). The control loop itself still uses the
    // unfiltered id/iq — this must not add lag to the current regulator.
    _id_filt += (id - _id_filt) * CURRENT_FILT_K;
    _iq_filt += (iq - _iq_filt) * CURRENT_FILT_K;

    // ── Stall protection ────────────────────────────────────────────────────
    // SENSORLESS/CLOSED: during the open-loop override the observer is seeded to
    // the forced angle, so its speed doesn't reflect the rotor (an OL stall shows
    // up as the hysteresis retrigger loop instead). A locked rotor in closed loop
    // = no back-EMF, ω→0, torque current held → heat with no cooling.
    //
    // HALL: a locked or wrong-angle rotor never transitions the halls, so
    // _hall_omega collapses to 0 while torque current is still commanded — the
    // exact case that holds near-DC current in one leg and cooks a low-side FET
    // (this is what killed a FET on a mis-set 120°-vs-60° table). Unlike VESC,
    // which has no stall trip and holds torque at standstill by design, the eGaN
    // stage can't sit at DC current, so we trip. The break-away clamp above bounds
    // the current during the _stall_t dwell before the trip fires.
    bool stalled;
    if (_sensor_mode == SensorMode::HALL) {
        stalled = (_state == State::HALL) && (fabsf(_hall_omega) < 1.0f) && (fabsf(iq_set) > _stall_i);
    } else {
        stalled = (_state == State::CLOSED) && (fabsf(_obs_omega) < _stall_w) && (fabsf(iq) > _stall_i);
    }
    if (stalled) {
        _stall_timer += _dt;
        if (_stall_timer > _stall_t) {
            trip_fault(FAULT_STALL);
            return;   // MOE already cut; don't write another PWM cycle
        }
    } else {
        _stall_timer = 0.0f;
    }

    const float err_d = id_set - id;
    const float err_q = iq_set - iq;
    _integ_d = clampf(_integ_d + err_d * _cur_ki_dt, -_v_max, _v_max);
    _integ_q = clampf(_integ_q + err_q * _cur_ki_dt, -_v_max, _v_max);
    float vd = err_d * _cur_kp + _integ_d;
    float vq = err_q * _cur_kp + _integ_q;

    // Clamp the voltage vector to the SVPWM limit, prioritising vq (torque).
    const float vd_lim = _v_max * 0.7071068f;
    vd = clampf(vd, -vd_lim, vd_lim);
    const float vq_room = sqrtf(_v_max * _v_max - vd * vd);
    vq = clampf(vq, -vq_room, vq_room);

    // ── αβ voltages → SVPWM duties ──────────────────────────────────────────
    float v_alpha, v_beta;
    FOC::inv_park(vd, vq, sin_t, cos_t, v_alpha, v_beta);

    const float m_alpha = v_alpha * _inv_vbus_half;
    const float m_beta  = v_beta  * _inv_vbus_half;

    float va, vb, vc;
    FOC::inv_clarke(m_alpha, m_beta, va, vb, vc);

    float da, db, dc;
    FOC::svpwm(va, vb, vc, da, db, dc);

    dt_comp_apply_duties(sin_t, cos_t, da, db, dc);
    write_duties(da, db, dc);

    // ── Dead-time handling ──────────────────────────────────────────────────
    // During the bridge's dead time (both FETs briefly off at each switch-over)
    // the phase current — not the PWM — sets the output: a phase sourcing
    // current (i>0) gets pulled low and so delivers LESS voltage than commanded;
    // a phase sinking current (i<0) gets pulled high and delivers MORE.
    //
    // Two places that error can be dealt with, selected by
    // cfg.deadtime_comp_on_duty:
    //
    //   duty feed-forward (default) — dt_comp_apply_duties() above already
    //     nudged the duties so the bridge delivers what was asked. The observer
    //     then wants the COMMANDED voltage, and dt_comp_alpha_beta() returns
    //     zero so nothing is subtracted twice.
    //
    //   observer-side (VESC) — the duties go out untouched and the estimate of
    //     what was applied is corrected instead. This is what vedderb/bldc does:
    //     mod_alpha_raw/mod_beta_raw reach foc_svm() unmodified, while
    //     update_valpha_vbeta() subtracts the dead-time term only from the
    //     modulation used to derive state->v_alpha/v_beta ("Note that these are
    //     not used to control the switching times", mcpwm_foc.c).
    //
    // VESC gets away with the second because its current loop closes on measured
    // current and absorbs the error. Ours has to push through a low-current hall
    // break-away phase where V_dt is a large fraction of the total drive, so the
    // feed-forward earns its keep — leaving it out measurably cost torque here.
    float dv_alpha, dv_beta;
    dt_comp_alpha_beta(sin_t, cos_t, dv_alpha, dv_beta);
    _v_alpha_prev = v_alpha - dv_alpha;   // applied αβ volts → observer next cycle
    _v_beta_prev  = v_beta  - dv_beta;

    // ── Telemetry snapshots ─────────────────────────────────────────────────
    _t_id    = id;
    _t_iq    = iq;
    _t_vd    = vd;
    _t_vq    = vq;
    _t_duty  = sqrtf(m_alpha * m_alpha + m_beta * m_beta); // modulation depth (1.0 ≈ full)
    _t_erpm  = omega_ctrl * _w_to_erpm;
    _t_theta = theta;
}

// Dead-time voltage error projected into αβ, for subtracting from the commanded
// voltage to get what the bridge actually delivered. Mirrors vedderb/bldc
// update_valpha_vbeta():
//     mod_alpha_sgn = 1/3·(2·sgn(ia) − sgn(ib) − sgn(ic))
//     mod_beta_sgn  = 1/√3·(sgn(ib) − sgn(ic))
// which is just the Clarke transform of the per-phase error, each phase being
// off by ±V_dt = ±t_dead·f_sw·vbus depending on its current direction.
//
// Scale note: VESC expresses this in its own modulation units (mod 1 ≡ ⅔·vbus,
// factor foc_dt_us·foc_f_zv), so its volt-domain magnitude works out to ⅔·V_dt.
// We keep our physically-derived V_dt (cfg.deadtime_comp_volts) rather than
// copying that ⅔ — in VESC foc_dt_us is a hand-tuned knob that absorbs the
// convention, whereas ours is computed from the real dead time.
//
// Sign comes from the FILTERED dq currents rotated back to phase currents, not
// the raw samples: at the zero-crossing the raw sign chatters, and chatter here
// would inject noise straight into the observer's voltage input.
void MotorControl::dt_comp_phase_signs(float sin_t, float cos_t,
                                       float &sa, float &sb, float &sc) const
{
    float ia_f, ib_f;
    FOC::inv_park(_id_filt, _iq_filt, sin_t, cos_t, ia_f, ib_f);   // dq → αβ
    float pa, pb, pc;
    FOC::inv_clarke(ia_f, ib_f, pa, pb, pc);                       // αβ → abc
    sa = (pa >= 0.0f) ? 1.0f : -1.0f;
    sb = (pb >= 0.0f) ? 1.0f : -1.0f;
    sc = (pc >= 0.0f) ? 1.0f : -1.0f;
}

void MotorControl::dt_comp_alpha_beta(float sin_t, float cos_t,
                                      float &dv_alpha, float &dv_beta) const
{
    if (_dt_comp_volts <= 0.0f || _dt_comp_on_duty) {
        // Nothing to correct: either disabled, or the duties were already
        // compensated so the commanded voltage IS the delivered voltage.
        dv_alpha = dv_beta = 0.0f;
        return;
    }
    float sa, sb, sc;
    dt_comp_phase_signs(sin_t, cos_t, sa, sb, sc);
    dv_alpha = _dt_comp_volts * (1.0f / 3.0f) * (2.0f * sa - sb - sc);
    dv_beta  = _dt_comp_volts * 0.57735026919f * (sb - sc);
}

// Feed-forward form: nudge each phase's duty in the SAME direction as its
// current (add where i>0, subtract where i<0) so the bridge actually delivers
// the commanded voltage. Clamped to [0,1] only — write_duties() owns the real
// ceiling and shifts all three together, which preserves the zero-sequence
// relationship SVPWM just established. Clamping to _duty_max here instead would
// saturate one phase early and distort the vector.
void MotorControl::dt_comp_apply_duties(float sin_t, float cos_t,
                                        float &da, float &db, float &dc) const
{
    if (_dt_comp_volts <= 0.0f || !_dt_comp_on_duty || _inv_vbus_half <= 0.0f) {
        return;
    }
    const float step = _dt_comp_volts * _inv_vbus_half * 0.5f;   // = V_dt / vbus
    float sa, sb, sc;
    dt_comp_phase_signs(sin_t, cos_t, sa, sb, sc);
    da = clampf(da + sa * step, 0.0f, 1.0f);
    db = clampf(db + sb * step, 0.0f, 1.0f);
    dc = clampf(dc + sc * step, 0.0f, 1.0f);
}

// Ortega flux-linkage observer (vedderb/bldc foc_observer_update).
//   x_dot = v − R·i + (γ/2)·(x − L·i)·(λ² − |x − L·i|²)
//   θ     = atan2(x2 − L·iβ, x1 − L·iα)
// L and R are the per-phase values pre-scaled by 3/2 in init.
void MotorControl::observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    // VESC-style speed/duty-scaled observer gain (m_gamma_now duty map): gain is
    // cut at low modulation (low speed) so the angle converges GENTLY after the
    // open-loop hard switch, ramping to full as back-EMF grows. This is what
    // keeps the OL→CLOSED handover from blipping, without any angle blend.
    const float mod    = sqrtf(v_alpha * v_alpha + v_beta * v_beta) * _inv_vbus_half;
    const float gscale = clampf(mod * _obs_gain_mod_inv, _obs_gain_slow_frac, 1.0f);
    const float gamma_half = _obs_gamma_half * gscale;

    const float L_ia = _obs_L * i_alpha;
    const float L_ib = _obs_L * i_beta;
    const float e1   = _obs_x1 - L_ia;
    const float e2   = _obs_x2 - L_ib;
    float err        = _obs_lambda2 - (e1 * e1 + e2 * e2);
    if (err > 0.0f) err = 0.0f;   // VESC: forcing err ≤ 0 aids observer convergence

    _obs_x1 += (v_alpha - _obs_R * i_alpha + gamma_half * e1 * err) * _dt;
    _obs_x2 += (v_beta  - _obs_R * i_beta  + gamma_half * e2 * err) * _dt;

    const float theta = atan2f(_obs_x2 - L_ib, _obs_x1 - L_ia);

    // Speed via a VESC-style PLL (foc_pll_run): a tracking loop locks _pll_theta
    // onto the observer angle, and its integrator IS the speed estimate. Unlike
    // differentiating the angle, this stays clean at low speed (small back-EMF),
    // which is what lets the lock survive far below the old ~900 erpm floor.
    const float delta = wrap_pi(theta - _pll_theta);
    _pll_theta  = wrap_pi(_pll_theta + (_obs_omega + _pll_kp * delta) * _dt);
    _obs_omega += _pll_ki * delta * _dt;

    _obs_theta   = theta;   // commutation still uses the raw observer angle, not the PLL angle
    _t_obs_theta = theta;
}

// ── Sensorless angle/torque state machine (I/f startup → observer) ───────────
// Extracted from the ISR for readability; behaviour is unchanged. Sets theta and
// iq_set (id_set stays 0). Returns false if it gated the bridge off this cycle
// (abandoned/failed start) — the caller must then return without writing PWM.
bool MotorControl::run_sensorless(Mode mode, float dir, float iq_cmd,
                                  float i_alpha, float i_beta, float i_max, bool coasting,
                                  float &theta, float &id_set, float &iq_set)
{
    id_set = 0.0f;
    iq_set = iq_cmd;
    bool started_now = false;

    // ── VESC-style sensorless open-loop override (mcpwm_foc control_current) ──
    // While too slow to sense, we force the angle through a timed lock→ramp→const
    // sequence and commutate on that FORCED angle. The Ortega observer free-runs
    // the whole time on the real applied v/i (never seeded or clobbered — same as
    // VESC's single observer) and its output is IGNORED here; it is used only at
    // the transfer, and only if it has converged by then (see the handover gate
    // in the else-branch below). If it hasn't, the start is abandoned and retried.
    //
    // Open-loop speed threshold scales with commanded current (more torque →
    // wider open-loop band), like VESC's openloop_rpm_max map.
    const float ol_cur = fabsf(iq_cmd) + _ol_boost_q;
    float ol_rpm_max = mapf(ol_cur, 0.0f, _current_max,
                            _ol_rpm_low * _open_handover_erpm, _open_handover_erpm);
    ol_rpm_max = clampf(ol_rpm_max, 0.0f, _open_handover_erpm);

    // Hysteresis TIMER: accumulate the time spent below the open-loop speed.
    if (fabsf(_obs_omega) < ol_rpm_max * _erpm_to_w && _hyst_timer < _ol_hyst) {
        _hyst_timer += _dt;
    } else if (_hyst_timer > 0.0f) {
        _hyst_timer -= _dt;
    }
    // Re-trigger a fresh sequence after a stall (skipped if just armed above,
    // and never while coasting/braking — must not auto-re-spin the motor).
    if (_hyst_timer >= _ol_hyst && _ol_timer <= 1e-4f && !coasting) {
        _ol_timer   = _ol_t_total;
        _lock_count = 0;          // fresh convergence watch for this attempt
        started_now = true;
    }

    if (_ol_timer > 0.0f) {
        // Forced rotation: 0 during lock, ramped 0→max during ramp, then full.
        const float time_fwd = _ol_t_total - _ol_timer;
        float rpm = ol_rpm_max;
        if (time_fwd < _ol_t_lock) {
            rpm = 0.0f;
        } else if (time_fwd < _ol_t_lock + _ol_t_ramp) {
            rpm = mapf(time_fwd, _ol_t_lock, _ol_t_lock + _ol_t_ramp, 0.0f, ol_rpm_max);
        }
        _override_ang = wrap_pi(_override_ang + dir * rpm * _erpm_to_w * _dt);
        if (started_now) {
            _override_ang = wrap_pi(_override_ang + dir * 1.04719755f); // +60° anti-stuck kick
        }

        theta  = _override_ang;
        // Cap the open-loop torque current (VESC foc_sl_openloop_max_q) to limit
        // heating during forced commutation, and clamp the speed-PI integrator to
        // the same so it can't wind up (→ over-current / overshoot) while forced.
        iq_set = clampf(iq_cmd + dir * _ol_boost_q, -_ol_max_q, _ol_max_q);
        // Capture soft-start: the rotor sits at an unknown angle, and a stepped
        // full current maximally excites its (lightly damped) swing into the
        // I/f well — the backward-run-then-jerk start. Ramp the current in over
        // the beginning of the (static) lock phase so the vector pulls the
        // rotor over instead of slingshotting it; accelerate only after capture.
        if (time_fwd < OL_IQ_RAMP_S) {
            iq_set *= time_fwd / OL_IQ_RAMP_S;
        }
        _integ_spd = clampf(_integ_spd, -_ol_max_q, _ol_max_q);

        // Convergence watch (decides the transfer only — NOT commutation, which
        // stays forced above). The free-running observer is "converged" once its
        // flux magnitude |x − L·i| holds in the [0.5λ, 1.5λ] band CONTINUOUSLY for
        // _lock_need samples. This naturally only accumulates once there is enough
        // back-EMF (the const phase at ol_rpm_max); at standstill the magnitude
        // wanders and the count keeps resetting. Same detector the TRACK phase uses.
        const float te1 = _obs_x1 - _obs_L * i_alpha;
        const float te2 = _obs_x2 - _obs_L * i_beta;
        const float fl2 = te1 * te1 + te2 * te2;
        if (fl2 > 0.25f * _obs_lambda2 && fl2 < 2.25f * _obs_lambda2) {
            if (_lock_count < 0xFFFFU) {
                _lock_count++;
            }
        } else {
            _lock_count = 0;
        }

        // Timer-based end of the forced sequence. When _ol_timer reaches 0 the
        // else-branch below runs the transfer gate: hand over to the observer angle
        // iff converged, otherwise abandon → coast → retry (bounded).
        _ol_timer -= _dt;
        _ol_release = _ol_t_release;   // armed for the post-handover boost fade
        _track_timer = 0.0f;           // OL running → TRACK is over
        _hyst_timer = 0.0f;
        _state = State::OPENLOOP;
    } else if (_track_timer > 0.0f) {
        // TRACK: commutate on the observer angle but hold iq=0 while the
        // observer re-converges (see the fresh-start comment above).
        _track_timer -= _dt;
        _override_ang = _obs_theta;
        theta  = _obs_theta;
        iq_set = 0.0f;
        _state = State::ALIGN;
        // Observer lock detector. Genuinely tracking ⇔ |flux| stays in a band
        // around λ. ω alone can't decide this (at standstill the angle is
        // atan2 of noise and the PLL random-walks ω high), and a single sample
        // of |flux| can't either: an unconverged observer carries a DC offset
        // that sweeps |flux| through 0..2λ every electrical cycle and can pass
        // at the sampled instant — CLOSED then slams full commanded iq onto a
        // wobbling angle (the intermittent CURRENT-mode start overcurrent).
        // Require the band to hold CONTINUOUSLY for _lock_need samples.
        const float te1 = _obs_x1 - _obs_L * i_alpha;
        const float te2 = _obs_x2 - _obs_L * i_beta;
        const float fl2 = te1 * te1 + te2 * te2;
        if (fl2 > 0.25f * _obs_lambda2 && fl2 < 2.25f * _obs_lambda2) {
            if (_lock_count < 0xFFFFU) {
                _lock_count++;
            }
        } else {
            _lock_count = 0;
        }
        if (_track_timer <= 0.0f) {
            const bool locked = _lock_count >= _lock_need;
            if (locked) {
                if (mode == Mode::SPEED) {
                    // Seed the speed loop to the speed the observer found
                    // (same reset as the OL handover).
                    _spd_set_erpm = _obs_omega * _w_to_erpm;
                    _integ_spd    = 0.0f;
                }
            } else {
                // No lock → standstill (or too slow to matter): forced start.
                _ol_timer     = _ol_t_total;
                _lock_count   = 0;      // fresh convergence watch for this attempt
                _override_ang = wrap_pi(_obs_theta + dir * 1.04719755f); // anti-stuck kick
                _hyst_timer   = 0.0f;
                _obs_omega    = 0.0f;   // discard the random-walk estimate
                _pll_theta    = _obs_theta;
            }
        }
    } else {
        const bool handover = (_state == State::OPENLOOP);
        // ── Transfer gate ────────────────────────────────────────────────────
        // First cycle after the forced sequence: only hand the commutation angle
        // to the observer if it actually converged (the watch above). If it did
        // not, the forced start failed — abandon it, coast for a cooldown, then
        // retry. Bounded: after _ol_max_attempts, latch FAULT_STALL so a motor
        // that never locks can't be force-driven (heated) indefinitely.
        if (handover && _lock_count < _lock_need) {
            _ol_attempts++;
            if (_ol_attempts >= _ol_max_attempts) {
                _ol_locked_out = true;         // stop retrying until throttle released
                _fault_code    = FAULT_STALL;
                _fault_latched = true;
                hold_off(State::FAULT);        // (counters/lockout survive reset_control)
                return false;
            }
            _ol_cooldown = _ol_cooldown_t; // high-Z coast, then a fresh attempt
            hold_off(State::IDLE);         // (survives reset_control: see header)
            return false;
        }
        if (handover) {
            _ol_attempts = 0;              // converged → restore the retry budget
        }
        _override_ang = _obs_theta;
        theta  = _obs_theta;   // pure sensorless
        _state = State::CLOSED;
        if (mode == Mode::SPEED) {
            // Handover into the speed loop: the integrator wound up against the
            // clamped setpoint during the forced ramp, and the open-loop current
            // was mostly non-torque-producing (absorbed by the load angle) — so
            // carrying either into closed loop just over-torques and overshoots.
            // Reset the loop and re-seed the setpoint to the measured speed; the
            // subsequent acceleration is governed by the setpoint slew
            // (speed_ramp_erpm_s), which the observer/PLL can track.
            if (handover) {
                _spd_set_erpm = _obs_omega * _w_to_erpm;
                _integ_spd    = 0.0f;
                iq_set        = 0.0f;
            }
            _ol_release = 0.0f;
        } else if (_ol_release > 0.0f && _ol_t_release > 1e-3f) {
            // CURRENT mode: no outer loop to hand over to, so just fade the
            // boost out instead of stepping ~12 A → iq_cmd in one cycle.
            iq_set = clampf(iq_cmd + dir * _ol_boost_q * (_ol_release / _ol_t_release),
                            -i_max, i_max);
            _ol_release -= _dt;
        }
    }
    return true;
}

// ── Hall sensors ────────────────────────────────────────────────────────────
// Raw 3-bit hall state from the J304 GPIOs: A=PB11, B=PB7, C=PB10 (all GPIOB).
uint8_t MotorControl::read_hall_state() const
{
    const uint32_t idr = palReadPort(GPIOB);
    uint8_t s = 0;
    if (idr & (1u << 11)) s |= 0x1;   // Hall A
    if (idr & (1u << 7))  s |= 0x2;   // Hall B
    if (idr & (1u << 10)) s |= 0x4;   // Hall C
    return s;
}

// A 3-hall motor has exactly six valid states (the two it never enters stay
// NaN), and the six sector centres must be spaced ~60° apart around the circle.
// Require BOTH before HALL mode may drive — matching VESC's "exactly 2 invalid"
// rule and additionally rejecting a malformed table (garbage params, detection
// under load, two states at the same angle) whose bad angles would drive at the
// wrong commutation position — the exact class of fault that killed a low-side
// FET. A partial (<6) or geometrically inconsistent table is refused.
void MotorControl::update_hall_table_valid()
{
    float a[6];
    uint8_t n = 0;
    for (uint8_t k = 0; k < 8; k++) {
        if (!isnan(_hall_table[k])) {
            if (n < 6) a[n] = _hall_table[k];
            n++;
        }
    }
    if (n != 6) { _hall_table_valid = false; return; }

    // Sort the six wrapped angles ascending (insertion sort — tiny, ISR-safe).
    for (uint8_t i = 1; i < 6; i++) {
        const float v = a[i];
        int8_t j = int8_t(i) - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
    // Each adjacent gap (and the wrap-around gap) must be one sector ±30°. Six
    // ~60° gaps sum to 360°, so a transposed/garbage entry breaks the spacing.
    bool ok = true;
    for (uint8_t i = 0; i < 6 && ok; i++) {
        const float gap = (i < 5) ? (a[i + 1] - a[i]) : (a[0] + TWO_PI - a[5]);
        if (gap < HALL_HALF_SECTOR || gap > HALL_SECTOR + HALL_HALF_SECTOR) {
            ok = false;
        }
    }
    _hall_table_valid = ok;
}

// Decode + debounce the halls, and produce an interpolated commutation angle
// (_hall_theta) plus a speed estimate from transition timing (_hall_omega).
// Validity is table-driven: any of the 8 states can be legal (some motors use
// 0/7 and skip 3/4). Returns false only on a state the table doesn't map (NaN)
// — i.e. one that never appeared during detection, or a genuine glitch.
bool MotorControl::update_hall()
{
    const uint8_t raw = read_hall_state();   // 0..7, all potentially valid

    // Debounce: commit a new state only after HALL_DEBOUNCE identical raw reads.
    if (raw == _hall_raw_prev) {
        if (_hall_deb < HALL_DEBOUNCE) _hall_deb++;
    } else {
        _hall_raw_prev = raw;
        _hall_deb = 0;
    }

    _hall_ticks++;

    if (_hall_deb >= HALL_DEBOUNCE && raw != _hall_state) {
        const float ang = _hall_table[raw];
        if (isnan(ang)) {
            return false;   // table has no angle for this state → not calibrated
        }
        if (_hall_state == HALL_NONE) {
            // First fix: no previous edge, so no speed yet — sit at the centre.
            _hall_base  = ang;
            _hall_dir   = 1.0f;
            _hall_omega = 0.0f;
        } else {
            // Direction + speed from the 60° step between sector centres.
            const float d   = wrap_pi(ang - _hall_table[_hall_state]);
            _hall_dir       = (d >= 0.0f) ? 1.0f : -1.0f;
            const float dts = float(_hall_ticks) * _dt;
            _hall_omega     = (dts > 1e-6f) ? (_hall_dir * HALL_SECTOR / dts) : 0.0f;
            // The rotor just crossed into this sector, so it is one half-sector
            // before the centre (in the direction of travel).
            _hall_base      = wrap_pi(ang - _hall_dir * HALL_HALF_SECTOR);
            // A genuine sector transition = confirmed rotor motion; count it so
            // the break-away current clamp releases once the rotor is turning.
            if (_hall_move_count < 255) _hall_move_count++;
        }
        _hall_state = raw;
        _hall_ticks = 0;
    }

    // Interpolate within the sector, capped at ±60° so a missed edge can't let
    // the angle run away past the next sector.
    float adv = _hall_omega * (float(_hall_ticks) * _dt);
    adv = clampf(adv, -HALL_SECTOR, HALL_SECTOR);
    _hall_theta = wrap_pi(_hall_base + adv);

    // No transition for a while → the rotor has stopped; drop the speed estimate.
    if (float(_hall_ticks) * _dt > HALL_STOP_S) {
        _hall_omega = 0.0f;
    }

    _t_hall_state = _hall_state;
    return true;
}

// Kick off a hall-table detection spin (thread context; call at standstill).
// Self-terminating after HD_REVS; the caller should not run the comms failsafe
// while it is in progress (it would coast the spin early).
void MotorControl::start_hall_detect()
{
    // Refuse while a trip is latched rather than clearing it: detection drives
    // real current into the motor, so the host must release the fault (command
    // zero/stop) before asking for a spin.
    if (!_initialized || _fault_latched) {
        return;
    }
    for (uint8_t k = 0; k < 8; k++) {
        _hd_sin[k] = _hd_cos[k] = 0.0f;
        _hd_n[k] = 0;
    }
    _hd_angle = 0.0f;
    _hd_ticks = 0;
    _hall_detect_done = false;
    _integ_d = _integ_q = 0.0f;   // detection runs the current PI — start clean
    _last_cmd_ms = AP_HAL::millis();
    _mode        = Mode::HALL_DETECT;
    arm_bridge();
}

bool MotorControl::hall_detect_result(float out_deg[8]) const
{
    if (!_hall_detect_done) {
        return false;
    }
    for (uint8_t k = 0; k < 8; k++) {
        out_deg[k] = _hall_detect_deg[k];
    }
    return true;
}

bool MotorControl::take_hall_detect_result(float out_deg[8])
{
    if (!_hall_detect_fresh) {
        return false;
    }
    for (uint8_t k = 0; k < 8; k++) {
        out_deg[k] = _hall_detect_deg[k];
    }
    _hall_detect_fresh = false;
    return true;
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
