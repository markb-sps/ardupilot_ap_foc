#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifdef HAL_PERIPH_ENABLE_FOC_ESC

#include <stdint.h>
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
    // Build the board motor config, start the controller, bind the VESC link to
    // vesc_uart, and claim the PWM-input capture. Call once from AP_Periph::init.
    void init(AP_HAL::UARTDriver *vesc_uart);

    // Service the arbiter + VESC link. Call every main loop with millis().
    void update(uint32_t now_ms);

    // DroneCAN ESC RawCommand source (this node = ESC index 0). frac is 0..1 of
    // the current limit. Called from the CAN dispatcher so DroneCAN transport
    // types stay out of this module.
    void set_can_throttle(float frac);

private:
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

    // ── Throttle-source arbiter state (each stamps its latest torque + time) ──
    float    _can_amps  = 0.0f;   // last DroneCAN RawCommand torque [A]
    uint32_t _can_ms    = 0;      // millis() of that command (0 = none yet)
    float    _pwm_amps  = 0.0f;   // last valid PWM-derived torque [A]
    uint32_t _pwm_ms    = 0;      // millis() of last valid PWM frame (0 = none)
    bool     _pwm_armed = false;  // boot-low safety gate for the PWM source
};

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
