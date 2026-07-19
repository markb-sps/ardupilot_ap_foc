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
constexpr float DT_COMP_I_BAND  = 1.0f;     // [A] current band over which dead-time sign() is softened
constexpr float OL_IQ_RAMP_S    = 0.2f;   // capture soft-start: OL current ramp-in time

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
    setup.deadtime_ticks             = cfg.deadtime_ticks;
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
    _vbus_fold_inv = (cfg.vbus_fold_band > 0.1f) ? (1.0f / cfg.vbus_fold_band) : 10.0f;
    _mode_switch_i = cfg.mode_switch_current;
    // vbus-dependent constants: seeded from cfg.vbus here, recomputed each ISR
    // cycle from the measured, filtered bus voltage.
    _vbus_flt      = cfg.vbus;
    _mod_to_vmax   = cfg.max_modulation * 0.57735026919f; // /√3
    _dt_comp_volts = cfg.deadtime_comp_volts;
    _v_max         = _mod_to_vmax * cfg.vbus;
    _inv_vbus_half = 2.0f / cfg.vbus;
    // Dead-time comp: convert the lost voltage [V] into a per-phase duty step
    // (phase-to-midpoint voltage = (duty-0.5)·vbus, so Δduty = ΔV / vbus).
    _dt_comp_duty  = (cfg.vbus > 0.0f) ? (cfg.deadtime_comp_volts / cfg.vbus) : 0.0f;

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
    _obs_x1 = _obs_x2 = _obs_theta = _obs_omega = 0.0f;
    _pll_theta = 0.0f;
    _stall_timer = 0.0f;
}

void MotorControl::trip_fault(uint8_t code)
{
    // ISR context: cut the output stage immediately via a bare MOE clear.
    stm32_foc_motor_control_disable_outputs_isr();
    _fault_code = code;
    _state      = State::FAULT;
    reset_control();
}

// Bridge held off (STOP / latched fault): truly high-Z the outputs (MOE=0) so
// the rotor freewheels rather than being short-braked by PWM(0,0,0) — which
// would tie all three phases to GND via the low-side FETs and damp/jitter the
// rotor against cogging. Outputs are re-enabled by the next spin-up command.
void MotorControl::hold_off(State s)
{
    reset_control();
    stm32_foc_motor_control_disable_outputs_isr();
    stm32_foc_motor_control_write_pwm(0, 0, 0);
    _state = s;
    _t_id = _t_iq = _t_vd = _t_vq = _t_duty = _t_erpm = 0.0f;
    _v_alpha_prev = _v_beta_prev = 0.0f;
}

void MotorControl::enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (_initialized) {
        stm32_foc_motor_control_enable_outputs();
    }
#endif
}

void MotorControl::disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (_initialized) {
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
    // Sensorless-start lockout: after too many failed forced starts the bridge is
    // latched off (thermal cap). A nonzero command must NOT clear it — otherwise
    // the arbiter re-commanding every loop would retry forever. Only a zero/stop
    // command (below) releases it.
    if (_ol_locked_out && fabsf(amps) >= 0.01f) {
        _last_cmd_ms = AP_HAL::millis();  // still a live host, just held off
        return;
    }
    _last_cmd_ms = AP_HAL::millis();
    _fault_code  = FAULT_NONE;        // any host current command clears a latched trip
    if (fabsf(amps) < 0.01f) {
        _ol_locked_out = false;       // throttle released → clear the start lockout
        _ol_attempts   = 0;
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
    _oc_over_count = 0; _oc_blank = OC_BLANK_SAMPLES;
    stm32_foc_motor_control_enable_outputs();   // re-arm bridge if previously released
}

// VESC COMM_SET_CURRENT_BRAKE: apply iq opposite to rotation for regenerative
// braking. Loop stays active so back-EMF energy returns to the bus in a controlled
// way (no high-Z body-diode rectification). Auto-releases the bridge once speed
// drops below the safe-release threshold (see adc_sample_isr).
void MotorControl::set_brake_current(float amps)
{
    _last_cmd_ms = AP_HAL::millis();
    _fault_code  = FAULT_NONE;
    const float mag = fabsf(amps);
    if (mag < 0.01f) {           // zero brake ≡ coast
        set_current(0.0f);
        return;
    }
    _cmd_current = (mag > _regen_max) ? _regen_max : mag;  // magnitude, capped to regen limit
    _mode = Mode::BRAKE;
}

void MotorControl::set_rpm(float erpm)
{
    _last_cmd_ms = AP_HAL::millis();
    if (fabsf(erpm) < 1.0f) {    // zero command = explicit stop (also clears a latched fault)
        stop();
        return;
    }
    // Refuse a hot swap from another active controller under load (see set_current).
    if (!mode_change_allowed(Mode::SPEED)) {
        return;
    }
    // Clear a latched trip like set_current does — without this a stall fault
    // is unrecoverable for an RPM-streaming host: each packet would arm the
    // bridge for one cycle before the fault path cuts it (a short-brake blip at
    // the command rate), and the fault never clears.
    _fault_code = FAULT_NONE;
    _cmd_erpm = erpm;
    _mode = Mode::SPEED;
    _oc_over_count = 0; _oc_blank = OC_BLANK_SAMPLES;
    stm32_foc_motor_control_enable_outputs();
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

void MotorControl::stop()
{
    _mode = Mode::STOP;
    _fault_code = FAULT_NONE;   // clear latched fault on explicit stop
    _ol_attempts = 0;           // fresh sensorless-start retry budget
    _ol_cooldown = 0.0f;
    _ol_locked_out = false;
}

void MotorControl::set_debug_voltage(float duty)
{
    _last_cmd_ms = AP_HAL::millis();
    const float a = fabsf(duty);
    if (a < 0.001f) {
        _mode = Mode::STOP;
        return;
    }
    _debug_mod = (a > _debug_max_mod) ? _debug_max_mod : a;
    _debug_dir = (duty >= 0.0f) ? 1.0f : -1.0f;
    _fault_code = FAULT_NONE;   // a fresh debug command clears a latched trip
    _mode = Mode::DEBUG_VOLTAGE;
    _oc_over_count = 0; _oc_blank = OC_BLANK_SAMPLES;
    stm32_foc_motor_control_enable_outputs();
}

// Play a tone through the motor (see header). A fixed-axis (α) voltage vector
// modulated at freq_hz vibrates the windings without net rotation. Amplitude is
// capped and the frequency floored so a mistaken low/DC tone can't dump current
// through the ~55 mΩ winding resistance (the hard OC trip still backstops it).
void MotorControl::play_tone(float freq_hz, float amplitude, uint16_t duration_ms)
{
    if (!_initialized || duration_ms == 0) {
        return;
    }
    if (freq_hz < 200.0f) freq_hz = 200.0f;   // keep inductive reactance meaningful
    _beep_phase      = 0.0f;
    _beep_phase_step = TWO_PI * freq_hz * _dt;
    _beep_amp        = clampf(amplitude, 0.0f, _debug_max_mod);
    _beep_ticks_left = uint32_t(duration_ms) * (_pwm_update_rate_hz / 1000U);
    _fault_code      = FAULT_NONE;
    _mode            = Mode::BEEP;
    _oc_over_count = 0; _oc_blank = OC_BLANK_SAMPLES;
    stm32_foc_motor_control_enable_outputs();
}

// ── ISR path ────────────────────────────────────────────────────────────────

void MotorControl::adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v, uint16_t sample_w)
{
    static_cast<MotorControl *>(ctx)->adc_sample_isr(sample_u, sample_v, sample_w);
}

// Full FOC cycle, once per PWM period (ADC injected-EOC ISR).
void MotorControl::adc_sample_isr(uint16_t sample_u, uint16_t sample_v, uint16_t sample_w)
{
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
    // Filter the measured vbus (τ ≈ 5 ms at 20 kHz) and recompute the
    // vbus-dependent constants each cycle, so the volts→duty conversion and the
    // observer's assumed applied voltage stay correct whatever the actual
    // supply is. Below the plausible-supply floor keep the last good value.
    {
        const float vraw = stm32_foc_vbus_read_volts();
        if (vraw > 6.0f) {
            _vbus_flt += (vraw - _vbus_flt) * 0.01f;
        }
        const float inv_vbus = 1.0f / _vbus_flt;
        _inv_vbus_half = 2.0f * inv_vbus;
        _v_max         = _mod_to_vmax * _vbus_flt;
        _dt_comp_duty  = _dt_comp_volts * inv_vbus;
        _vbus          = _vbus_flt;
    }
    // Bus-OV regen foldback: scales any decelerating (bus-charging) current
    // from full at (vbus_max - band) to zero at vbus_max.
    const float ov_scale = clampf((_vbus_max - _vbus_flt) * _vbus_fold_inv, 0.0f, 1.0f);
    // Thermal derate of the iq limit (thread-computed from the board NTC).
    const float i_max = _current_max * _i_derate;

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
        const float pf = float(_period_ticks);
        stm32_foc_motor_control_write_pwm(uint16_t(da * pf), uint16_t(db * pf), uint16_t(dc * pf));

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
        const float pf = float(_period_ticks);
        stm32_foc_motor_control_write_pwm(uint16_t(da * pf), uint16_t(db * pf), uint16_t(dc * pf));

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
        if (fabsf(_obs_omega) < RELEASE_OMEGA) {
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
        stm32_foc_motor_control_disable_outputs_isr();
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
    bool started_now = false;
    if (_state != State::OPENLOOP && _state != State::CLOSED &&
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
            _spd_set_erpm = _obs_omega * _w_to_erpm;
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
        const float erpm_err = set_erpm - _obs_omega * _w_to_erpm;
        _integ_spd = clampf(_integ_spd + erpm_err * _spd_ki_dt, -i_max, i_max);
        iq_cmd = clampf(erpm_err * _spd_kp + _integ_spd, -i_max, i_max);
        // Regen limit: when iq opposes rotation (decelerating) the braking energy
        // returns to the bus, which a bench PSU can't sink — cap the braking
        // current hard, folded toward zero as vbus approaches vbus_max.
        if (iq_cmd * _obs_omega < 0.0f) {
            const float rl = _regen_max * ov_scale;
            iq_cmd = clampf(iq_cmd, -rl, rl);
        }
    } else if (mode == Mode::BRAKE) {
        // _cmd_current holds the brake magnitude (already capped to _regen_max);
        // sign opposes rotation. Clamp again here as the single enforcement
        // point, folded by the bus-OV scale.
        const float rl = _regen_max * ov_scale;
        iq_cmd = clampf((_obs_omega >= 0.0f ? -1.0f : 1.0f) * _cmd_current, -rl, rl);
    } else {
        iq_cmd = clampf(_cmd_current, -i_max, i_max);
        // CURRENT mode has no steady regen cap by design, but a decelerating
        // command must still fold back rather than pump the bus past vbus_max.
        if (iq_cmd * _obs_omega < 0.0f) {
            const float rl = i_max * ov_scale;
            iq_cmd = clampf(iq_cmd, -rl, rl);
        }
    }

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

    float theta;
    float id_set = 0.0f;
    float iq_set = iq_cmd;

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
                hold_off(State::FAULT);        // (counters/lockout survive reset_control)
                return;
            }
            _ol_cooldown = _ol_cooldown_t; // high-Z coast, then a fresh attempt
            hold_off(State::IDLE);         // (survives reset_control: see header)
            return;
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

    // ── dq current PI (Kp = L·ωbw, Ki = R·ωbw) with anti-windup ─────────────
    const float sin_t = sinf(theta);
    const float cos_t = cosf(theta);
    float id, iq;
    FOC::park(i_alpha, i_beta, sin_t, cos_t, id, iq);

    // ── Stall protection ────────────────────────────────────────────────────
    // CLOSED only: during the open-loop override the observer is seeded to the
    // forced angle, so its speed doesn't reflect the rotor (an OL stall shows
    // up as the hysteresis retrigger loop instead). A locked rotor in closed
    // loop = no back-EMF, ω→0, torque current held → heat with no cooling.
    if (_state == State::CLOSED &&
        fabsf(_obs_omega) < _stall_w && fabsf(iq) > _stall_i) {
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

    // ── Dead-time compensation ──────────────────────────────────────────────
    // During the bridge's dead-time (both FETs briefly off at each switch-over)
    // the phase current — not the PWM — sets the output: a phase sourcing
    // current (i>0) gets pulled low, so it delivers LESS voltage than commanded;
    // a phase sinking current (i<0) gets pulled high and delivers MORE. The
    // error is a roughly fixed magnitude (_dt_comp_duty, = V_dt/vbus) whose sign
    // follows the phase current. We cancel it by nudging each phase's duty in
    // the SAME direction as its current: add duty where i>0, subtract where i<0.
    //
    // sign(i) is softened to a linear ramp across ±DT_COMP_I_BAND amps so the
    // correction doesn't chatter at the current zero-crossing, where both the
    // current sign and the dead-time effect itself are ill-defined.
    //
    // This makes the *delivered* voltage match the desired v_alpha/v_beta, which
    // is exactly what the observer assumes — so the observer's angle estimate
    // stays accurate even at low speed, where the lost ~0.1V was otherwise a
    // large fraction of the back-EMF and pushed the sensorless floor up.
    if (_dt_comp_duty > 0.0f) {
        constexpr float inv_band = 1.0f / DT_COMP_I_BAND;
        da = clampf(da + clampf(ia * inv_band, -1.0f, 1.0f) * _dt_comp_duty, 0.0f, 1.0f);
        db = clampf(db + clampf(ib * inv_band, -1.0f, 1.0f) * _dt_comp_duty, 0.0f, 1.0f);
        dc = clampf(dc + clampf(ic * inv_band, -1.0f, 1.0f) * _dt_comp_duty, 0.0f, 1.0f);
    }

    const float pf = float(_period_ticks);
    stm32_foc_motor_control_write_pwm(uint16_t(da * pf), uint16_t(db * pf), uint16_t(dc * pf));

    // Applied voltage for the next observer iteration. With dead-time comp on,
    // the delivered voltage ≈ this desired value, so no separate correction is
    // needed on the observer input.
    _v_alpha_prev = v_alpha;
    _v_beta_prev  = v_beta;

    // ── Telemetry snapshots ─────────────────────────────────────────────────
    _t_id    = id;
    _t_iq    = iq;
    _t_vd    = vd;
    _t_vq    = vq;
    _t_duty  = sqrtf(m_alpha * m_alpha + m_beta * m_beta); // modulation depth (1.0 ≈ full)
    _t_erpm  = _obs_omega * _w_to_erpm;
    _t_theta = theta;
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

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
