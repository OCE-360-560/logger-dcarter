#include "motor.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void motor_init(Motor *motor) {

    gpio_set_function(motor->gpio_pin, GPIO_FUNC_PWM);
    // Initialize GPIO pins for the motor
    gpio_init(motor->direction_pin1);    
    gpio_init(motor->direction_pin2);
    gpio_set_dir(motor->direction_pin1, GPIO_OUT);
    gpio_set_dir(motor->direction_pin2, GPIO_OUT);

    // Set up PWM for the signal pin
    uint slice = pwm_gpio_to_slice_num(motor->gpio_pin);
    
    float clk_hz=(float)clock_get_hz(clk_sys);
float divider=clk_hz/(1000000.0f); // 1 MHz clock for 1 us resolution

    // 1Mhz PWM counter: one tick per microsecond
    pwm_set_clkdiv(slice, divider); // Replace 'divider' with the actual divider value
   
    pwm_set_wrap(slice, motor->period_us - 1); // Set the PWM period
    pwm_set_gpio_level(motor->gpio_pin, 0); // Set initial PWM level to 0
    pwm_set_enabled(slice, true); // Enable the PWM slice
}

void motor_set_duty_cycle(Motor *motor, float duty) {
    uint16_t pulse_us = 0;
    // Clamp duty cycle to the range [-1.0, 1.0]
    if (duty > 1.0f) {
        duty = 1.0f;
    }
    if (duty < -1.0f) {
        duty = -1.0f;
    }

    // Determine direction based on the sign of the duty cycle
    if (duty >= 0) {
        gpio_put(motor->direction_pin1, true);
        gpio_put(motor->direction_pin2, false);
        pulse_us = (uint16_t) (duty * motor->period_us);
    } else {
        gpio_put(motor->direction_pin1, false);
        gpio_put(motor->direction_pin2, true);
        duty = -duty; // Make duty positive for PWM calculation
    }

    printf("pulse =%d\n", pulse_us);
    // Set the PWM level for the signal pin
    pwm_set_gpio_level(motor->gpio_pin, pulse_us);

}