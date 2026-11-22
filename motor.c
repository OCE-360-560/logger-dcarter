#include "motor.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"

#define RPM_UPDATE_INTERVAL_US 100000 // Update RPM every 100 ms

static Motor *encoder_motor = NULL; // Pointer to the motor with the encoder

static void encoder_irq_handler(uint gpio, uint32_t events)
{
    Motor *motor= encoder_motor;

    if (motor != NULL && 
        gpio == motor->encoder_pin && 
        (events & GPIO_IRQ_EDGE_RISE)){
        motor->encoder_count++;
    }
}
void motor_init(Motor *motor)
{
    gpio_set_function(motor->signal_pin, GPIO_FUNC_PWM);
    
    // Initialize GPIO pins for the motor
    gpio_init(motor->direction_pin1);
    gpio_init(motor->direction_pin2);
    gpio_set_dir(motor->direction_pin1, GPIO_OUT);
    gpio_set_dir(motor->direction_pin2, GPIO_OUT);

    // Encoder pin initialization
    gpio_init(motor->encoder_pin);
    gpio_set_dir(motor->encoder_pin, GPIO_IN);

    motor->encoder_count = 0;
    motor->last_encoder_count = 0;
    motor->rpm = 0.0f;
    // Registar the encoder interrupt
    encoder_motor=motor;
    gpio_set_irq_enabled_with_callback(
        motor->encoder_pin, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);

    // Set up PWM for the signal pin
    uint slice = pwm_gpio_to_slice_num(motor->signal_pin);

    float clk_hz = (float)clock_get_hz(clk_sys);
    float divider = clk_hz / (1000000.0f); // 1 MHz clock for 1 us resolution

    // 1Mhz PWM counter: one tick per microsecond
    pwm_set_clkdiv(slice, divider); // Replace 'divider' with the actual divider value

    pwm_set_wrap(slice, motor->period_us - 1); // Set the PWM period
    pwm_set_gpio_level(motor->signal_pin, 0);    // Set initial PWM level to 0
    pwm_set_enabled(slice, true);              // Enable the PWM slice

    motor->last_update_us = time_us_32();
}

void motor_set_duty_cycle(Motor *motor, float duty)
{
    uint16_t pulse_us = 0;
    // Clamp duty cycle to the range [-1.0, 1.0]
    if (duty > 1.0f)
    {
        duty = 1.0f;
    }
    if (duty < -1.0f)
    {
        duty = -1.0f;
    }

    // Determine direction based on the sign of the duty cycle
    if (duty >= 0)
    {
        gpio_put(motor->direction_pin1, true);
        gpio_put(motor->direction_pin2, false);
        pulse_us = (uint16_t)(duty * motor->period_us);
    }
    else
    {
        gpio_put(motor->direction_pin1, false);
        gpio_put(motor->direction_pin2, true);
        duty = -duty; // Make duty positive for PWM calculation
        pulse_us = (uint16_t)(duty * motor->period_us);

    }

    // printf("pulse =%d\n", pulse_us);
    // Set the PWM level for the signal pin
    pwm_set_gpio_level(motor->signal_pin, pulse_us);
}

void motor_update(Motor *motor)
{
    uint32_t now = time_us_32();
    uint32_t elapsed = now - motor->last_update_us;

    if (elapsed < RPM_UPDATE_INTERVAL_US){
        return;
    }
    // printf("count = %d\r\n", motor->encoder_count);
    
    // Snapshot and reset encoder coutner atomically
        uint32_t irq_state = save_and_disable_interrupts();
        uint32_t pulses=motor->encoder_count;
        motor->encoder_count=0;
        restore_interrupts(irq_state);

        // pulses/ppr/elapsed time get revolution per micor seconds, then times 60.0*1,000,000 to get RPM
        motor->rpm = (float)pulses *60.0f *1000000.0f/ 
        (motor->encoder_ppr * (float)elapsed);


        motor-> last_update_us = now;
    }