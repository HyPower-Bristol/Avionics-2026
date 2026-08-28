/*
 * pwm.h
 *
 *  Created on: Sep 14, 2025
 *      Author: sara.shabbir-khan
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;  // Timer handle
    uint32_t channel;          // PWM channel

    uint32_t timer_clk_hz;

    uint32_t pulse_min;        // Min pulse width (timer counts)
    uint32_t pulse_max;        // Max pulse width (timer counts)

    uint16_t us_min;           // Min pulse in microseconds (1000)
    uint16_t us_max;           // Max pulse in microseconds (2000)

    uint8_t angle_min;         // Minimum angle (usually 0)
    uint8_t angle_max;         // Maximum angle (usually 180)

    uint8_t current_angle;
    uint8_t prev_angle;

    uint16_t can_id;
} Servo_t;

uint32_t CCR_Calculator(uint32_t pulse_us, uint32_t timer_clk_hz);
void ServoInit(Servo_t *servo);
void ServoActuator(Servo_t *servo, uint32_t pwm);


// Usage:
// uint32_t pulse_min = ccr_from_us(1000, TIM_CLK_HZ, htim2.Init.Prescaler, htim2.Init.Period);
// uint32_t pulse_max = ccr_from_us(2000, TIM_CLK_HZ, htim2.Init.Prescaler, htim2.Init.Period);

#endif /* INC_PWM_H_ */
