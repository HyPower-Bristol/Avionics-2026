/*
 * pwm.c
 *
 *  Created on: Aug 20, 2025
 *      Author: sara.shabbir-khan
 */

#include "pwm.h"
#include "math.h"

uint32_t CCR_Calculator(uint32_t pulse_us, uint32_t timer_clk_hz)
{
	float tick_time_us = 1e6f / timer_clk_hz;
	uint32_t ccr = (uint32_t)(pulse_us / tick_time_us);
    return ccr;
}

void ServoInit(Servo_t *servo){
	servo->pulse_min=CCR_Calculator(servo->us_min, servo->timer_clk_hz);
	servo->pulse_max=CCR_Calculator(servo->us_max, servo->timer_clk_hz);

}

void ServoActuator(Servo_t *servo, uint32_t angle)
{

    servo->current_angle=angle;
	uint32_t pwm = (uint32_t ) roundf((angle * 2000.0f)/180.0f + 500.0f);

	uint32_t ccr = (uint32_t) roundf(pwm * 1000.0f / 20000.0f); // CCR = PWM * ARR / Period

	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, ccr);

    servo->current_angle=angle;
}

/***
void servo_swing(void) {
    // Define min and max pulse widths (in timer counts)
    uint32_t pulse_min = 250;   // ~1ms
    uint32_t pulse_max = 500;  // ~2ms

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // start PWM

    while (1) {
        // Sweep from min to max
        for (uint32_t pulse = pulse_min; pulse <= pulse_max; pulse+=10) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
            HAL_Delay(10);  // adjust speed
        }
        // Sweep back from max to min using int
        for (uint32_t pulse = pulse_max; pulse >= pulse_min; pulse-=10) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
            HAL_Delay(10);
        }
    }
}
***/
