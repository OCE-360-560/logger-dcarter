/*
 * motor.h
 * The motor setup for the brushless motor. This takes in the duty cycle of the motor and converts that into the 
 * motor speed and rpm based upon the encoder_ppr or the roll of the board (stretch exercise)
 *
 * Author: David Carter
 * License: MIT (see LICENSE)
 */

#ifndef MOTOR_H 
#define MOTOR_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "imu.h"

typedef struct {
    uint signal_pin;          // GPIO pin number for the motor
    uint direction_pin1;     // GPIO pin number for the motor direction (forward)
    uint direction_pin2;     // GPIO pin number for the motor direction (reverse)
    uint16_t period_us;  // Minimum pulse width in microseconds (e.g., 1000)
    // Encoder configuration
    uint encoder_pin;
    float encoder_ppr;

    // Encoder measurement state
    volatile int32_t encoder_count; // Current encoder count
    uint32_t last_encoder_count; // Timestamp of the last encoder pulse
    uint32_t last_update_us; // Last encoder count for speed calculation
    float rpm; // Calculated RPM based on encoder counts
} Motor;

void motor_init(Motor *motor);
void motor_set_duty_cycle(Motor *motor, float duty); // Speed range: -1.
void motor_update(Motor *motor); // Update encoder count and calculate RPM

#endif // MOTOR_H 