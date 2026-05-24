#include "MotorControl.h"
#include "stm32_foc_motor_control.h"
#include "foc_transforms.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <math.h>

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS   = 3.3f / 4095.0f;
constexpr uint16_t ZERO_SAMPLES = 64;       // zero-current calibration window
constexpr float TWO_PI          = 6.28318530718f;
constexpr float PI_F            = 3.14159265359f;
constexpr float SPEED_LPF       = 0.02f;    // observer-speed low-pass α
constexpr float HANDOVER_DROP   = 0.6f;     // closed→open re-sync hysteresis
constexpr float DT_COMP_I_BAND  = 1.0f;     // [A] current band over which dead-time sign() is softened

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
    _oc_trip       = cfg.overcurrent_trip;
    _v_max         = cfg.max_modulation * cfg.vbus * 0.57735026919f; // /√3
    _inv_vbus_half = 2.0f / cfg.vbus;
    // Dead-time comp: convert the lost voltage [V] into a per-phase duty step
    // (phase-to-midpoint voltage = (duty-0.5)·vbus, so Δduty = ΔV / vbus).
    _dt_comp_duty  = (cfg.vbus > 0.0f) ? (cfg.deadtime_comp_volts / cfg.vbus) : 0.0f;

    _erpm_to_w = TWO_PI / 60.0f;
    _w_to_erpm = 60.0f / TWO_PI;

    _open_current    = cfg.openloop_current;
    _open_handover_w = cfg.openloop_erpm * _erpm_to_w;
    _open_accel_w    = cfg.openloop_accel_erpm_s * _erpm_to_w * _dt;
    _align_cycles    = uint16_t(float(cfg.align_ms) * 1e-3f * float(_pwm_update_rate_hz));
    _blend_cycles    = uint16_t(float(cfg.blend_ms) * 1e-3f * float(_pwm_update_rate_hz));
    if (_blend_cycles == 0) _blend_cycles = 1;

    _obs_L          = 1.5f * cfg.motor_Ls;
    _obs_R          = 1.5f * cfg.motor_Rs;
    _obs_lambda2    = cfg.motor_flux * cfg.motor_flux;
    _obs_gamma_half = 0.5f * cfg.observer_gain;

    _spd_kp    = cfg.speed_kp;
    _spd_ki_dt = cfg.speed_ki * _dt;
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

void MotorControl::deinit()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_deinit();
    _current_sense_initialized = false;
    _current_zero_valid        = false;
    _initialized               = false;
#endif
}

void MotorControl::reset_control()
{
    _open_theta = _open_omega = 0.0f;
    _integ_d = _integ_q = _integ_spd = 0.0f;
    _stage_cnt = 0;
    _v_alpha_prev = _v_beta_prev = 0.0f;
    _obs_x1 = _obs_x2 = _obs_theta = _obs_omega = 0.0f;
}

void MotorControl::trip_fault(uint8_t code)
{
    // ISR context: cut the output stage immediately via a bare MOE clear.
    stm32_foc_motor_control_disable_outputs_isr();
    _fault_code = code;
    _state      = State::FAULT;
    reset_control();
}

// Bridge held off (STOP / latched fault): zero duty, clean control state.
void MotorControl::hold_off(State s)
{
    reset_control();
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

void MotorControl::set_current(float amps)
{
    _last_cmd_ms = AP_HAL::millis();
    if (fabsf(amps) < 0.01f) {   // zero command = explicit stop (also clears a latched fault)
        stop();
        return;
    }
    _cmd_current = clampf(amps, -_current_max, _current_max);
    _mode = Mode::CURRENT;
}

void MotorControl::set_rpm(float erpm)
{
    _last_cmd_ms = AP_HAL::millis();
    if (fabsf(erpm) < 1.0f) {    // zero command = explicit stop (also clears a latched fault)
        stop();
        return;
    }
    _cmd_erpm = erpm;
    _mode = Mode::SPEED;
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
}

// ── ISR path ────────────────────────────────────────────────────────────────

void MotorControl::adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v)
{
    static_cast<MotorControl *>(ctx)->adc_sample_isr(sample_u, sample_v);
}

// Full FOC cycle, once per PWM period (ADC injected-EOC ISR).
void MotorControl::adc_sample_isr(uint16_t sample_u, uint16_t sample_v)
{
    // ── Zero-current calibration (outputs gated, no current flowing) ────────
    if (!_current_zero_valid) {
        _zero_accum[0] += sample_u;
        _zero_accum[1] += sample_v;
        if (++_zero_count == ZERO_SAMPLES) {
            _current_zero_raw[0] = uint16_t(_zero_accum[0] / ZERO_SAMPLES);
            _current_zero_raw[1] = uint16_t(_zero_accum[1] / ZERO_SAMPLES);
            _current_zero_valid  = true;
        }
        return;
    }
    _adc_sample_cb_count++;

    // ── Phase currents ──────────────────────────────────────────────────────
    // Low-side shunt polarity: positive phase current pulls the amplified ADC
    // reading BELOW the zero reference, so current = (zero - sample).
    const float ia = float(int32_t(_current_zero_raw[0]) - int32_t(sample_u)) * ADC_LSB_VOLTS * _current_scale;
    const float ib = float(int32_t(_current_zero_raw[1]) - int32_t(sample_v)) * ADC_LSB_VOLTS * _current_scale;
    const float ic = -(ia + ib);
    _t_ia = ia;
    _t_ib = ib;

    // ── Hard overcurrent trip ───────────────────────────────────────────────
    if (fabsf(ia) > _oc_trip || fabsf(ib) > _oc_trip || fabsf(ic) > _oc_trip) {
        trip_fault(FAULT_ABS_OVERCURRENT);
    }

    float i_alpha, i_beta;
    FOC::clarke(ia, ib, i_alpha, i_beta);

    // Observer always runs (using the voltage applied last cycle).
    observer_update(_v_alpha_prev, _v_beta_prev, i_alpha, i_beta);

    const Mode mode = _mode;

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

    // Re-entering closed loop from a non-FOC state → restart from alignment.
    if (_state == State::FAULT || _state == State::DEBUG) {
        _state = State::IDLE;
    }

    const float dir = (mode == Mode::SPEED)
                          ? (_cmd_erpm >= 0.0f ? 1.0f : -1.0f)
                          : (_cmd_current >= 0.0f ? 1.0f : -1.0f);

    // ── State machine → electrical angle θ and dq set-points ────────────────
    float theta;
    float id_set = 0.0f;
    float iq_set = 0.0f;

    switch (_state) {
    case State::IDLE:
        _state = State::ALIGN;
        _stage_cnt = 0;
        _open_theta = 0.0f;
        _open_omega = 0.0f;
        // fallthrough
    case State::ALIGN:
        // Park stator field at θ=0 and pull the rotor there with d-axis current.
        theta  = 0.0f;
        id_set = _open_current;
        if (++_stage_cnt >= _align_cycles) {
            _state = State::OPENLOOP;
            _stage_cnt = 0;
        }
        break;

    case State::OPENLOOP: {
        const float target_w = (mode == Mode::SPEED)
                                   ? _cmd_erpm * _erpm_to_w
                                   : dir * (_open_handover_w * 1.2f);
        _open_omega  = step_towards(_open_omega, target_w, _open_accel_w);
        _open_theta  = wrap_pi(_open_theta + _open_omega * _dt);
        theta  = _open_theta;
        iq_set = dir * _open_current;
        // Hand over only past the floor AND once the observer agrees with the
        // forced rotation — same direction and speed within 50% — sustained for
        // the debounce window. Without this, a marginal low-speed observer can
        // hand over to a wrong angle and the motor runs backwards / desyncs.
        // If it never agrees we stay in open loop (correct direction, safe).
        const bool fast_enough = fabsf(_open_omega) >= _open_handover_w;
        const bool obs_agrees  = (_obs_omega * _open_omega > 0.0f) &&
                                 (fabsf(_obs_omega - _open_omega) < 0.5f * fabsf(_open_omega));
        if (fast_enough && obs_agrees) {
            if (++_stage_cnt >= _blend_cycles) {
                _state = State::BLEND;
                _stage_cnt = 0;
            }
        } else {
            _stage_cnt = 0;
        }
        break;
    }

    case State::BLEND: {
        _open_theta = wrap_pi(_open_theta + _open_omega * _dt);
        const float k = float(_stage_cnt) / float(_blend_cycles);
        theta  = wrap_pi(_open_theta + k * wrap_pi(_obs_theta - _open_theta));
        iq_set = dir * _open_current;
        if (++_stage_cnt >= _blend_cycles) {
            _state = State::CLOSED;
            _integ_spd = dir * _open_current; // seed speed PI to current iq
        }
        break;
    }

    case State::CLOSED:
    default:
        theta = _obs_theta;
        if (mode == Mode::SPEED) {
            const float erpm_err = _cmd_erpm - _obs_omega * _w_to_erpm;
            _integ_spd = clampf(_integ_spd + erpm_err * _spd_ki_dt, -_current_max, _current_max);
            iq_set = clampf(erpm_err * _spd_kp + _integ_spd, -_current_max, _current_max);
        } else {
            iq_set = clampf(_cmd_current, -_current_max, _current_max);
        }
        // Loss of sync / stall → fall back to forced commutation.
        if (fabsf(_obs_omega) < _open_handover_w * HANDOVER_DROP) {
            _state = State::OPENLOOP;
            _open_theta = _obs_theta;
            _open_omega = _obs_omega;
        }
        break;
    }

    // ── dq current PI (Kp = L·ωbw, Ki = R·ωbw) with anti-windup ─────────────
    const float sin_t = sinf(theta);
    const float cos_t = cosf(theta);
    float id, iq;
    FOC::park(i_alpha, i_beta, sin_t, cos_t, id, iq);

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
    const float L_ia = _obs_L * i_alpha;
    const float L_ib = _obs_L * i_beta;
    const float e1   = _obs_x1 - L_ia;
    const float e2   = _obs_x2 - L_ib;
    const float err  = _obs_lambda2 - (e1 * e1 + e2 * e2);

    _obs_x1 += (v_alpha - _obs_R * i_alpha + _obs_gamma_half * e1 * err) * _dt;
    _obs_x2 += (v_beta  - _obs_R * i_beta  + _obs_gamma_half * e2 * err) * _dt;

    const float theta = atan2f(_obs_x2 - L_ib, _obs_x1 - L_ia);

    // Electrical speed from the angle derivative (wrapped), low-pass filtered.
    const float dtheta = wrap_pi(theta - _obs_theta);
    const float omega  = dtheta / _dt;
    _obs_omega += SPEED_LPF * (omega - _obs_omega);
    _obs_theta  = theta;
    _t_obs_theta = theta;   // snapshot for diagnostics
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
