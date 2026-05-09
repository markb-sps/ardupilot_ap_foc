#include "MotorControl.h"
#include "stm32_foc_motor_control.h"
#include "foc_transforms.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <math.h>

namespace ChibiOS {

namespace {
constexpr float ADC_LSB_VOLTS      = 3.3f / 4095.0f;
constexpr float IIR_CURRENT_ALPHA  = 0.05f;
constexpr uint8_t ZERO_CAP_LOG2    = 6;  // 64 samples
constexpr float PHASE_TO_RAD       = 2.0f * 3.14159265358979f / 4294967296.0f;
constexpr float TWO_PI             = 2.0f * 3.14159265358979f;
}

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

    _current_zero_raw[0] = 0U;
    _current_zero_raw[1] = 0U;
    _zero_accum[0]       = 0U;
    _zero_accum[1]       = 0U;
    _zero_count          = 0;
    _filtered_volts[0]   = 0.0f;
    _filtered_volts[1]   = 0.0f;
    _filtered_volts[2]   = 0.0f;
    _current_zero_valid  = false;

    Stm32FocMotorControlSetup setup{};
    setup.pwm_clock_hz                = cfg.pwm_clock_hz;
    setup.pwm_frequency_hz            = cfg.pwm_frequency_hz;
    setup.current_sample_delay_ticks  = cfg.current_sample_delay_ticks;
    setup.deadtime_ticks              = cfg.deadtime_ticks;
    setup.center_aligned              = cfg.center_aligned;
    setup.break_input_enabled         = cfg.break_input_enabled;

    Stm32FocMotorControlCallbacks callbacks{};
    callbacks.ctx           = this;
    callbacks.phase_current = adc_sample_callback;

    const auto init_result = stm32_foc_motor_control_init(setup, callbacks);
    if (!init_result.ok) {
        return false;
    }
    _period_ticks              = init_result.period_ticks;
    _pwm_update_rate_hz        = init_result.update_rate_hz;
    _current_sense_initialized = init_result.current_sense_ok;

    // Precompute constants used in ISR
    const float dt         = 1.0f / float(_pwm_update_rate_hz);
    _current_scale         = cfg.current_scale;
    _smo_gain              = cfg.smo_gain;
    _dt_inv_Ls             = (cfg.motor_Ls > 0.0f) ? (dt / cfg.motor_Ls) : 0.0f;
    _rs_dt_inv_Ls          = cfg.motor_Rs * _dt_inv_Ls;
    _smo_omega_c_dt        = TWO_PI * cfg.smo_cutoff_hz * dt;
    _vbus_half             = cfg.vbus * 0.5f;

    // Reset observer state
    _smo_i_alpha_hat = 0.0f;
    _smo_i_beta_hat  = 0.0f;
    _smo_e_alpha     = 0.0f;
    _smo_e_beta      = 0.0f;
    _smo_theta       = 0.0f;
    _v_alpha_cmd     = 0.0f;
    _v_beta_cmd      = 0.0f;

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
    _period_ticks              = 0U;
    _pwm_update_rate_hz        = 0U;
    _initialized               = false;
#endif
}

void MotorControl::set_open_loop_target(float electrical_hz, float modulation, bool reset_phase)
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized || _pwm_update_rate_hz == 0U) {
        return;
    }

    _open_loop_amplitude = (modulation <= 0.0f) ? 0.0f : (modulation > 1.0f ? 1.0f : modulation);

    if (reset_phase) {
        _open_loop_phase = 0;
    }
    if (electrical_hz <= 0.0f) {
        _open_loop_phase_step = 0;
        return;
    }

    const uint64_t step = uint64_t(electrical_hz * (4294967296.0 / double(_pwm_update_rate_hz)));
    _open_loop_phase_step = uint32_t(step);
#else
    (void)electrical_hz;
    (void)modulation;
    (void)reset_phase;
#endif
}

void MotorControl::enable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_enable_outputs();
#endif
}

void MotorControl::disable_outputs()
{
#if HAL_USE_PWM == TRUE && STM32_PWM_USE_TIM1 == TRUE
    if (!_initialized) {
        return;
    }
    stm32_foc_motor_control_disable_outputs();
#endif
}

bool MotorControl::get_filtered_phase_volts(float &u, float &v, float &w) const
{
    if (!_current_zero_valid) {
        return false;
    }
    osalSysLock();
    u = _filtered_volts[0];
    v = _filtered_volts[1];
    w = _filtered_volts[2];
    osalSysUnlock();
    return true;
}

// ── ISR path ────────────────────────────────────────────────────────────────

void MotorControl::adc_sample_callback(void *ctx, uint16_t sample_u, uint16_t sample_v)
{
    static_cast<MotorControl *>(ctx)->adc_sample_isr(sample_u, sample_v);
}

// Called from ADC JEOC ISR at _pwm_update_rate_hz (typically 40 kHz for
// centre-aligned 20 kHz carrier).  Performs the full FOC cycle:
//   1. zero calibration  2. current sensing  3. SMO update
//   4. open-loop phase advance  5. SVPWM  6. CCR write
void MotorControl::adc_sample_isr(uint16_t sample_u, uint16_t sample_v)
{
    // ── Zero calibration ───────────────────────────────────────────────────
    if (!_current_zero_valid) {
        _zero_accum[0] += sample_u;
        _zero_accum[1] += sample_v;
        if (++_zero_count == (1U << ZERO_CAP_LOG2)) {
            _current_zero_raw[0] = uint16_t(_zero_accum[0] >> ZERO_CAP_LOG2);
            _current_zero_raw[1] = uint16_t(_zero_accum[1] >> ZERO_CAP_LOG2);
            _current_zero_valid  = true;
        }
        return;
    }

    _adc_sample_cb_count++;
    // ── Phase current sensing ──────────────────────────────────────────────
    const float u_v = float(int32_t(sample_u) - int32_t(_current_zero_raw[0])) * ADC_LSB_VOLTS;
    const float v_v = float(int32_t(sample_v) - int32_t(_current_zero_raw[1])) * ADC_LSB_VOLTS;
    const float w_v = -(u_v + v_v);

    _filtered_volts[0] += IIR_CURRENT_ALPHA * (u_v - _filtered_volts[0]);
    _filtered_volts[1] += IIR_CURRENT_ALPHA * (v_v - _filtered_volts[1]);
    _filtered_volts[2] += IIR_CURRENT_ALPHA * (w_v - _filtered_volts[2]);

    // ── Clarke transform (amplitude-invariant) ─────────────────────────────
    float i_alpha, i_beta;
    FOC::clarke(u_v * _current_scale, v_v * _current_scale, i_alpha, i_beta);

    // ── Sliding Mode Observer ──────────────────────────────────────────────
    // Uses _v_alpha_cmd/_v_beta_cmd from the previous cycle (applied voltage).
    update_smo_isr(i_alpha, i_beta);

    // ── Open-loop voltage reference ────────────────────────────────────────
    const uint32_t phase     = _open_loop_phase;
    const float    amplitude = _open_loop_amplitude;
    const float    theta     = float(phase) * PHASE_TO_RAD;
    const float    sin_t     = sinf(theta);
    const float    cos_t     = cosf(theta);

    const float v_alpha_ref = amplitude * cos_t;
    const float v_beta_ref  = amplitude * sin_t;

    float va, vb, vc;
    FOC::inv_clarke(v_alpha_ref, v_beta_ref, va, vb, vc);

    float da, db, dc;
    FOC::svpwm(va, vb, vc, da, db, dc);

    const float pf = float(_period_ticks);
    stm32_foc_motor_control_write_pwm(
        uint16_t(da * pf),
        uint16_t(db * pf),
        uint16_t(dc * pf));

    // Store commanded voltages [V] for the next SMO iteration
    _v_alpha_cmd = v_alpha_ref * _vbus_half;
    _v_beta_cmd  = v_beta_ref  * _vbus_half;

    _open_loop_phase = phase + _open_loop_phase_step;
}

// Sliding Mode Observer for sensorless rotor angle estimation.
//
// Machine model (αβ frame, stator currents, surface PMSM):
//   L · dî/dt = v − R·î − z,   z = k · sign(î − i)
//
// Back-EMF extracted via LPF on the switching signal z:
//   ê[n] += ωc·dt · (z − ê[n−1])
//
// Angle:  θ = atan2(−ê_α, ê_β)
//
// Precomputed coefficients: _dt_inv_Ls = dt/Ls,  _rs_dt_inv_Ls = Rs·dt/Ls,
//                           _smo_omega_c_dt = ωc·dt
void MotorControl::update_smo_isr(float i_alpha, float i_beta)
{
    const float s_alpha = _smo_i_alpha_hat - i_alpha;
    const float s_beta  = _smo_i_beta_hat  - i_beta;

    const float z_alpha = _smo_gain * (s_alpha >= 0.0f ? 1.0f : -1.0f);
    const float z_beta  = _smo_gain * (s_beta  >= 0.0f ? 1.0f : -1.0f);

    // Observer current: Euler integration with precomputed Rs/Ls·dt coefficient
    _smo_i_alpha_hat += _dt_inv_Ls * (_v_alpha_cmd - z_alpha) - _rs_dt_inv_Ls * _smo_i_alpha_hat;
    _smo_i_beta_hat  += _dt_inv_Ls * (_v_beta_cmd  - z_beta)  - _rs_dt_inv_Ls * _smo_i_beta_hat;

    // Back-EMF extraction (LPF on switching signal)
    _smo_e_alpha += _smo_omega_c_dt * (z_alpha - _smo_e_alpha);
    _smo_e_beta  += _smo_omega_c_dt * (z_beta  - _smo_e_beta);

    _smo_theta = atan2f(-_smo_e_alpha, _smo_e_beta);
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
