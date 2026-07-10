#pragma once

#include <stdint.h>

namespace ChibiOS {

// Dedicated TIM3_CH1 (PC6) hardware PWM-input capture for the J305 RC throttle
// connector. Owns TIM3 in reset-slave "PWM input" mode at 1 us resolution and is
// entirely polled (no ISR) — matching the register-level style of the FOC TIM1
// driver. Configured fully at runtime; PC6 is left unassigned in hwdef.
void stm32_pwm_input_init(void);

// Latest captured pulse. Returns true only when a fresh rising edge (i.e. a new
// RC frame) has been captured since the previous call, in which case
// *pulse_us  = measured high-time  [us] and
// *period_us = measured frame period [us].
// Returns false (leaving the outputs untouched) when no new frame has arrived —
// the caller ages that out into a signal-lost / coast condition.
bool stm32_pwm_input_read(uint16_t *pulse_us, uint16_t *period_us);

} // namespace ChibiOS
