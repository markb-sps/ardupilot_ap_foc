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
    AP_Int8  _p_motor_poles;   // pole PAIRS (si_motor_poles = 2×)

    // ── Current loop / limits (I_MAX + I_OCHARD VESC-writable) ───────────────
    AP_Float _p_i_max;         // iq command ceiling [A]    → l_current_max
    AP_Float _p_i_oc;          // debounced per-phase OC trip [A] (DroneCAN-only)
    AP_Float _p_i_oc_hard;     // instant per-phase OC trip [A] → l_abs_current_max
    AP_Float _p_i_regen;       // max braking/regen current [A] (DroneCAN-only)
    AP_Float _p_i_slew;        // torque-command slew [A/s]     (DroneCAN-only)
    AP_Float _p_i_scale;       // ADC volts → amps sense scale  (DroneCAN-only)

    // ── Bus / thermal / stall protections ────────────────────────────────────
    AP_Float _p_v_max;         // regen fully cut at this bus V → l_max_vin
    AP_Float _p_v_fold;        // OV regen foldback band [V]    (DroneCAN-only)
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
    float    _pwm_amps  = 0.0f;   // last valid PWM-derived torque [A]
    uint32_t _pwm_ms    = 0;      // millis() of last valid PWM frame (0 = none)
    bool     _pwm_armed = false;  // boot-low safety gate for the PWM source
};

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
