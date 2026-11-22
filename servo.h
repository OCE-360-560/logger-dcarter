/*
 * servo.h
 * The servo setup for the servo motor that takes in the pulse width and angle and converts that into the motor movement
 * or uses the pitch of the board for the motor movement (stretch exercise).
 *
 * Author: David Carter
 * License: MIT (see LICENSE)
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "pico/stdlib.h"

typedef struct {
    uint signal_pin;            // GPIO pin number for the servo
    uint16_t period_us;         // Minimum pulse width in microseconds (e.g., 1000)
    uint16_t min_pulse_us;     // Minimum pulse width in microseconds (e.g., 1000)
    uint16_t max_pulse_us;     // Maximum pulse width in microseconds (e.g., 2000)
    uint16_t pulse_us;
} Servo;

void servo_init(Servo *servo);
void servo_set_angle(Servo *servo, float angle);

#endif //SERVO_H