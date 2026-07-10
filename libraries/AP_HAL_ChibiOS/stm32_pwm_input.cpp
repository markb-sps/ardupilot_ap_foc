#include "stm32_pwm_input.h"

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>

namespace ChibiOS {

// PC6 = TIM3_CH1 (AF2). Left out of hwdef and configured here at runtime, the
// same way MotorControl claims the ADC pins.
static const ioline_t PWM_IN_LINE = PAL_LINE(GPIOC, 6U);

void stm32_pwm_input_init(void)
{
    rccEnableTIM3(true);
    rccResetTIM3();

    // 1 us tick from the APB1 timer clock; ARR at full range so a lost signal
    // (long gap) simply free-runs and is caught by the no-fresh-frame path.
    TIM3->PSC = (uint16_t)((STM32_TIMCLK1 / 1000000U) - 1U);
    TIM3->ARR = 0xFFFFU;

    // "PWM input" mode, both captures fed from TI1 (the PC6 edge):
    //   IC1 = rising  -> CCR1 = frame period, and is the reset trigger
    //   IC2 = falling -> CCR2 = pulse high-time
    // Input filters (N=8) reject switching/edge noise on the servo lead.
    TIM3->CCMR1 = (1U << 0)     |     // CC1S = 01: IC1 mapped to TI1
                  (0x3U << 4)   |     // IC1F: N=8 filter
                  (2U << 8)     |     // CC2S = 10: IC2 mapped to TI1
                  (0x3U << 12);       // IC2F: N=8 filter
    TIM3->CCER  = TIM_CCER_CC1E |                    // CC1 rising  (CC1P=0)
                  TIM_CCER_CC2E | TIM_CCER_CC2P;     // CC2 falling (CC2P=1)
    // Slave mode: reset the counter on TI1FP1 (TS=101, SMS=0100 reset mode).
    TIM3->SMCR  = (5U << 4) | (4U << 0);
    TIM3->SR    = 0;
    TIM3->CR1   = TIM_CR1_CEN;

    palSetLineMode(PWM_IN_LINE,
                   PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_LOWEST | PAL_STM32_PUPDR_PULLDOWN);
}

bool stm32_pwm_input_read(uint16_t *pulse_us, uint16_t *period_us)
{
    // CC1IF sets on each rising edge (new frame) and is cleared by reading CCR1.
    // No fresh edge since the last poll → nothing new to report.
    if ((TIM3->SR & TIM_SR_CC1IF) == 0U) {
        return false;
    }
    const uint16_t period = (uint16_t)TIM3->CCR1;   // reading CCR1 clears CC1IF
    const uint16_t pulse  = (uint16_t)TIM3->CCR2;
    if (period_us != nullptr) { *period_us = period; }
    if (pulse_us  != nullptr) { *pulse_us  = pulse; }
    return true;
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
