#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void servo_init(Servo *servo) {
    // Set the GPIO pin as an output
    gpio_set_function(servo->signal_pin, GPIO_FUNC_PWM);
    
    // Get the PWM slice number for the given GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(servo->signal_pin);
    
    float clk_hz=(float)clock_get_hz(clk_sys);
    float divider=clk_hz/(1000000.0f); // 1 MHz clock for 1 us resolution

    pwm_set_clkdiv(slice_num, divider); // Set the clock divider for 1 MHz clock

    // 1MHz PwM counter: one tick per microsecond
    pwm_set_wrap(slice_num, servo->period_us - 1);
    
// Initialize at the midpoint of the pulse range
    uint16_t pulse_us=(servo->min_pulse_us + servo->max_pulse_us) / 2; // Start at the middle position
   
    // Set the initial pulse width to the minimum
    pwm_set_gpio_level(servo->signal_pin, pulse_us);    
    
    // Enable the PWM output
    pwm_set_enabled(slice_num, true);
}

void servo_set_angle(Servo *servo, float angle) {
    // Clamp the angle to the range [0, 180]
    if (angle < 0.0f) {
        angle = 0.0f;
    } 
    if (angle > 180.0f) {
        angle = 180.0f;
    }

    // Map the angle to the corresponding pulse width
    // uint16_t pulse_us = (uint16_t)(servo->min_pulse_us + (angle / 180.0f) * (servo->max_pulse_us - servo->min_pulse_us));
    uint16_t pulse_us = (servo->min_pulse_us + (angle / 180.0f) * (servo->max_pulse_us - servo->min_pulse_us));
    
    // Set the PWM level for the GPIO pin
    pwm_set_gpio_level(servo->signal_pin, pulse_us);
}