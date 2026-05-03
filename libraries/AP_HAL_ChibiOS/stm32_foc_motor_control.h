#pragma once

#include <stdint.h>

namespace ChibiOS {

struct Stm32FocMotorControlSetup {
    uint32_t pwm_clock_hz = 0;
    uint32_t pwm_frequency_hz = 0;
    uint16_t current_sample_delay_ticks = 2;
    uint8_t  deadtime_ticks = 0;
    bool     center_aligned = true;
    bool     break_input_enabled = false;
};

using Stm32FocMotorControlPhaseCurrentCallback = void (*)(void *ctx, uint16_t sample_u, uint16_t sample_v);

struct Stm32FocMotorControlCallbacks {
    void *ctx = nullptr;
    Stm32FocMotorControlPhaseCurrentCallback phase_current = nullptr;
};

struct Stm32FocMotorControlInitResult {
    bool ok = false;
    bool current_sense_ok = false;
    uint16_t period_ticks = 0;
    uint32_t update_rate_hz = 0;
};

Stm32FocMotorControlInitResult stm32_foc_motor_control_init(const Stm32FocMotorControlSetup &setup,
                                                            const Stm32FocMotorControlCallbacks &callbacks);
void stm32_foc_motor_control_deinit();
void stm32_foc_motor_control_enable_outputs();
void stm32_foc_motor_control_disable_outputs();

// Write CCR values directly — must be called from ISR context (no syslock).
void stm32_foc_motor_control_write_pwm(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);

} // namespace ChibiOS
