#pragma once

#include <stdint.h>

namespace ChibiOS {

struct Stm32FocMotorControlSetup {
    uint32_t pwm_clock_hz = 0;
    uint32_t pwm_frequency_hz = 0;
    uint16_t current_sample_delay_ticks = 2;
    // Bridge dead time in NANOSECONDS — not PWM counter ticks. The BDTR DTG
    // field is clocked from t_DTS (the timer kernel clock, i.e. before the
    // prescaler), which is not the counter clock: on a 160 MHz G4 running a
    // 20 MHz PWM counter they differ by 8x. Passing real time units and doing
    // the encoding against the measured kernel clock keeps that from biting.
    uint16_t deadtime_ns = 0;
    bool     center_aligned = true;
    bool     break_input_enabled = false;
};

using Stm32FocMotorControlPhaseCurrentCallback = void (*)(void *ctx, uint16_t sample_u, uint16_t sample_v, uint16_t sample_w);

struct Stm32FocMotorControlCallbacks {
    void *ctx = nullptr;
    Stm32FocMotorControlPhaseCurrentCallback phase_current = nullptr;
};

struct Stm32FocMotorControlInitResult {
    bool ok = false;
    bool current_sense_ok = false;
    uint16_t period_ticks = 0;
    uint32_t update_rate_hz = 0;
    // Dead time the hardware was actually programmed with, after DTG encoding
    // and rounding. Report it — a silently-wrong dead time is invisible in
    // telemetry and destroys the power stage, so it must be observable.
    uint16_t deadtime_ns_actual = 0;
};

Stm32FocMotorControlInitResult stm32_foc_motor_control_init(const Stm32FocMotorControlSetup &setup,
                                                            const Stm32FocMotorControlCallbacks &callbacks);
void stm32_foc_motor_control_enable_outputs();
void stm32_foc_motor_control_disable_outputs();

// ISR-safe output cut: bare MOE clear, no OS lock. Call only from ISR context.
void stm32_foc_motor_control_disable_outputs_isr();

// Write CCR values directly — must be called from ISR context (no syslock).
void stm32_foc_motor_control_write_pwm(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);

// ── Diagnostic read-back (thread context, read-only) ────────────────────────
// What the BRIDGE HARDWARE actually has, as opposed to what the driver state
// machine believes. These exist because MOE can be cleared behind the driver's
// back — motor_control_fault_stop() does a bare BDTR write from the CPU
// exception handlers and deliberately touches no C++ state — after which
// MotorControl::arm_bridge() short-circuits on its own _outputs_on flag and
// silently never re-enables. Everything upstream then reports normal while the
// bridge coasts. Reads only; changes no register.
bool stm32_foc_motor_control_moe_set();
void stm32_foc_motor_control_read_ccr(uint16_t &ccr_u, uint16_t &ccr_v, uint16_t &ccr_w);

// Single-shot VBUS read via ADC1 IN1 (PA0). Returns DC-bus volts after the
// 357k/22k divider. Blocking (~1 µs); coexists with the injected current-sense
// channel, which has priority and will preempt the regular conversion.
// Call from thread context at low rate.
float stm32_foc_vbus_read_volts();

// Board-temp (NTC divider on PB12 = ADC1_IN11) as a fraction of Vref [0..1].
// Cached load like the VBUS read; the conversion alternates with VBUS on the
// ADC1 regular channel, so each updates at ~10 kHz.
float stm32_foc_temp_read_ratio();

} // namespace ChibiOS
