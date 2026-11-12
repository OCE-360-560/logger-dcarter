#ifndef MOTOR_H 
#define MOTOR_H

#include <stdint.h>
#include "pico/stdlib.h"
#include <stdio.h>

typedef struct {
    uint gpio_pin;          // GPIO pin number for the motor
    uint direction_pin1;     // GPIO pin number for the motor direction (forward)
    uint direction_pin2;     // GPIO pin number for the motor direction (reverse)
    uint16_t period_us;  // Minimum pulse width in microseconds (e.g., 1000)
} Motor;

void motor_init(Motor *motor);
void motor_set_duty_cycle(Motor *motor, float duty); // Speed range: -1.

#endif // MOTOR_H 