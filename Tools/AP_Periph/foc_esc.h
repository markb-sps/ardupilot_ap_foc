#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifdef HAL_PERIPH_ENABLE_FOC_ESC

#include <stdint.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL_ChibiOS/MotorControl.h>
#include <AP_HAL_ChibiOS/VescTelemetry.h>

namespace AP_HAL {
    class UARTDriver;
}

// Field-Oriented-Control ESC peripheral (TEST-G4-FOC-ESC / STM32G431).
//
// Owns the whole single-motor drive as one unit: the sensorless FOC current
// controller, the VESC-protocol telemetry/command link (USB serial), the J305
// PWM RC throttle input, and the torque-command arbiter that fuses them.
//
// Torque (current) control only. Three command sources, priority CAN > USB > PWM
// RC: whichever is freshest and highest-priority drives the motor, and it coasts
// only when all are stale. A VESC-Tool bench override (rpm / brake / duty-debug)
// takes exclusive control while active. See update() for the arbiter.
class FOC_ESC {
public:
    FOC_ESC() { AP_Param::setup_object_defaults(this, var_info); }

    // Build the board motor config, start the controller, bind the VESC link to
    // vesc_uart, and claim the PWM-input capture. Call once from AP_Periph::init.
    void init(AP_HAL::UARTDriver *vesc_uart);

    static const struct AP_Param::GroupInfo var_info[];

    // Service the arbiter + VESC link. Call every main loop with millis().
    void update(uint32_t now_ms);

    // DroneCAN ESC RawCommand source (this node = ESC index 0). frac is 0..1 of
    // the current limit. Called from the CAN dispatcher so DroneCAN transport
    // types stay out of this module.
    void set_can_throttle(float frac);

private:
    // Sink for VESC Tool "Write Motor Configuration": persist the mapped params
    // and schedule a reboot so they apply (reboot-to-apply model). The static
    // trampoline is what gets registered with vesc_telem.
    void on_mcconf_write(const ChibiOS::VescTelemetry::McconfIn &in);
    static void mcconf_write_trampoline(void *ctx,
                                        const ChibiOS::VescTelemetry::McconfIn &in) {
        static_cast<FOC_ESC *>(ctx)->on_mcconf_write(in);
    }
    // millis() at which a VESC-Tool config write asked for a reboot (0 = none).
    // Deferred so the COMM_SET_MCCONF ack + storage flush complete first.
    uint32_t _reboot_ms = 0;

    // ── Motor-parameter detection (VESC Tool "Measure R/L" / "Measure λ") ────
    // VESC runs these as blocking commands in a worker thread; we have no spare
    // thread and must not stall the periph main loop for the ~10 s a flux
    // measurement takes (DroneCAN and USB would both drop), so it is a state
    // machine ticked from update(). VESC Tool waits for the reply either way.
    void on_detect_request(const ChibiOS::VescTelemetry::DetectReq &req);
    static void detect_trampoline(void *ctx,
                                  const ChibiOS::VescTelemetry::DetectReq &req) {
        static_cast<FOC_ESC *>(ctx)->on_detect_request(req);
    }
    // Advance the detection. Returns true while it owns the motor, which holds
    // the throttle arbiter off exactly like the power-on chime does.
    bool update_detect(uint32_t now_ms);
    void detect_finish(float r, float l, float linkage, bool ok);

    enum class DetState : uint8_t {
        IDLE,
        RES_LOCK,      // ramp d-axis current in at zero speed, let it settle
        RES_MEAS,   // average vd/id → phase resistance
        IND_TONE,      // play the tone at each sweep frequency, record peak current
        IND_GAP,    // gap between tones so the peak window is clean
        FLUX_LOCK,      // flux: ramp current in with the rotor held
        FLUX_STILL,     // average the standstill duty (VESC duty_still)
        FLUX_RAMP,      // ramp commanded speed until duty reaches the target
        FLUX_SETTLE,    // let it stabilise before averaging
        FLUX_MEAS,   // average vd/vq/id/iq → linkage
        DONE_STOP,   // coast, then reply
    };
    DetState _det_state = DetState::IDLE;
    ChibiOS::VescTelemetry::DetectReq _det_req;
    uint32_t _det_ms      = 0;    // millis() the current phase started
    uint32_t _det_ticks   = 0;    // samples accumulated in an averaging phase
    float    _det_acc_a   = 0.0f; // accumulator (vd, or duty, or |v|)
    float    _det_acc_b   = 0.0f; // accumulator (id, or |i|)
    float    _det_erpm    = 0.0f; // commanded speed during F_RAMP
    float    _det_duty_max = 0.0f;// peak duty seen during F_RAMP (collapse check)
    float    _det_duty_still = 0.0f;
    float    _det_r       = 0.0f; // measured phase resistance [Ω]
    float    _det_l       = 0.0f; // measured phase inductance [H]
    float    _det_r_try   = 0.0f; // current level of the VESC resistance search [A]
    uint8_t  _det_freq_idx = 0;   // index into the L sweep frequency table
    // |Z|² and ω² per sweep point, for the least-squares |Z|² = R² + ω²L² fit.
    float    _det_zz[4]   = {0};
    float    _det_ww[4]   = {0};

    // Poll the J305 TIM3 capture and refresh the PWM throttle source (with the
    // boot-low arming gate + out-of-range coast).
    void read_pwm_throttle(uint32_t now_ms);

    // Play a short power-on chime through the motor once the controller is ready.
    // Returns true while the chime owns the motor (the arbiter stands off).
    // Aborts (and is done) if any real throttle command arrives or a fault trips.
    bool update_startup_chime(uint32_t now_ms, bool any_command);

    enum class ChimeState : uint8_t { WAIT, PLAY, DONE };
    ChimeState _chime_state = ChimeState::WAIT;
    uint8_t    _chime_idx     = 0;   // index into the note table
    uint32_t   _chime_step_ms = 0;   // millis() when the current note+gap ends

    ChibiOS::MotorControl  motor_control;
    ChibiOS::VescTelemetry vesc_telem{motor_control, 7};

    // ── Persisted config (AP_Periph storage) ────────────────────────────────
    AP_Int8  _p_sensor_mode;   // 0 = sensorless, 1 = hall
    // Hall table: electrical angle [deg 0..359] per hall state 0..7; -1 = invalid
    // (unused state). Populated by set_and_save() when a detection spin completes.
    AP_Int16 _p_hall[8];

    // ── Motor identity (VESC-writable via COMM_SET_MCCONF) ───────────────────
    AP_Float _p_motor_rs;      // phase resistance [Ω]      → foc_motor_r
    AP_Float _p_motor_ls;      // phase inductance [H]      → foc_motor_l
    AP_Float _p_motor_flux;    // PM flux linkage λ [Wb]    → foc_motor_flux_linkage
    AP_Float _p_obs_gain;      // Ortega observer γ        → foc_observer_gain
    AP_Float _p_cur_kp;        // current PI Kp [V/A]      → foc_current_kp (0 = derive)
    AP_Float _p_cur_ki;        // current PI Ki [V/(A·s)]  → foc_current_ki (0 = derive)
    AP_Float _p_hall_erpm0;    // blend start [eRPM]       → foc_sl_erpm_start
    AP_Float _p_hall_erpm1;    // blend end   [eRPM]       → foc_sl_erpm
    AP_Float _p_hall_intrp;    // rate-limiter floor [eRPM]→ foc_hall_interp_erpm
    AP_Int8  _p_motor_poles;   // pole PAIRS (si_motor_poles = 2×)

    // ── Current loop / limits (I_MAX + I_OCHARD VESC-writable) ───────────────
    AP_Float _p_i_max;         // iq command ceiling [A]    → l_current_max
    AP_Float _p_i_oc;          // debounced per-phase OC trip [A] (DroneCAN-only)
    AP_Float _p_i_oc_hard;     // instant per-phase OC trip [A] → l_abs_current_max
    AP_Float _p_i_regen;       // max braking/regen current [A] (DroneCAN-only)
    AP_Float _p_i_slew;        // torque-command slew [A/s]     (DroneCAN-only)
    AP_Float _p_i_scale;       // ADC volts → amps sense scale  (DroneCAN-only)

    // ── Bus / thermal / stall protections ────────────────────────────────────
    AP_Float _p_v_max;         // regen fully cut at this bus V → l_battery_regen_cut_end
    AP_Float _p_v_fold;        // OV regen foldback band [V]    → l_battery_regen_cut_start
    AP_Float _p_v_ov;          // hard bus over-voltage trip [V] → l_max_vin
    AP_Float _p_duty_max;      // per-phase duty ceiling        → l_max_duty
    // Sensorless open-loop start → VESC foc_openloop_rpm / foc_sl_openloop_*
    AP_Float _p_ol_boost;      // start boost current [A]
    AP_Float _p_ol_imax;       // open-loop iq cap [A]
    AP_Float _p_ol_erpm;       // handover speed [eRPM]
    AP_Float _p_ol_low;        // handover speed at zero current, fraction
    AP_Float _p_ol_hyst;       // dwell below threshold before override [s]
    AP_Float _p_ol_tlock;      // hold-angle lock time [s]
    AP_Float _p_ol_tramp;      // forced-speed ramp time [s]
    AP_Float _p_ol_tconst;     // forced-speed hold time [s]
    AP_Float _p_v_min;         // bus under-voltage floor [V]   (DroneCAN-only)
    AP_Float _p_v_uvfold;      // UV foldback band [V]          (DroneCAN-only)
    AP_Float _p_t_start;       // FET derate onset [°C]         → l_temp_fet_start
    AP_Float _p_t_max;         // FET hard trip [°C]            → l_temp_fet_end
    AP_Float _p_stall_rpm;     // stall speed threshold [eRPM]  (DroneCAN-only)
    AP_Float _p_stall_i;       // stall current threshold [A]   (DroneCAN-only)
    AP_Float _p_stall_t;       // stall dwell before trip [s]   (DroneCAN-only)

    // ── Throttle-source arbiter state (each stamps its latest torque + time) ──
    float    _can_amps  = 0.0f;   // last DroneCAN RawCommand torque [A]
    uint32_t _can_ms    = 0;      // millis() of that command (0 = none yet)
    float    _pwm_amps  = 0.0f;   // last valid PWM-derived torque [A], always ≥ 0
    uint32_t _pwm_ms    = 0;      // millis() of last valid PWM frame (0 = none)
    bool     _pwm_armed = false;  // boot-neutral safety gate for the PWM source
    // Trigger pulled back → _pwm_amps is a BRAKE magnitude for set_brake_current()
    // rather than a motoring command. Split from the sign of _pwm_amps because the
    // two dispatch to different MotorControl entry points, and a forward trigger
    // against a backwards roll is a reduced-magnitude MOTORING command, not a brake.
    bool     _pwm_brake = false;
};

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
