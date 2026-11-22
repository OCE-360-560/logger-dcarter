/*
 * A4_SD_Logging.c
 * Environmental logger: reads light
 * and temperature once per sample
 * period, prints each reading with the
 * corresponding time and voltage, and
 * appends it to multi_source_cali_ref.csv.
 *
 * Author: David Carter
 * Date: November 22nd, 2026
 * License: MIT (see LICENSE)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "sd_logger.h" // SD card logging
#include "servo.h"
#include "motor.h"
#include "ina219.h"

#define SAMPLE_PERIOD_MS 1000
#define SAMPLE_RATE_MS 100
#define LOG_HEADER "time_s, volts, amps, watts,  pitch_servo, motor_roll, pitch_deg, roll_deg, servo_pulse, motor_rpm\r\n"
#define PATH "Motor_data2.csv" // file name on the SD card

//  Assignmnt 6
#define I2C_BUS i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// 100 kHz, not 400: the BNO080 stretches the clock, and slower is safer
#define I2C_HZ (100 * 1000)
#define SCAN_TIMEOUT_US 10000

int main()
{
    stdio_init_all();

    sd_logger_init(); // initialize the SD card and SPI pins

    init_i2c_bus();    // initialize the I2C bus for the IMU and INA219
    imu_init(I2C_BUS); // initialize the IMU and start the rotation vector report

    sleep_ms(SAMPLE_PERIOD_MS);

    if (!sd_logger_open(PATH, LOG_HEADER))
    {
        printf("# SD open failed\r\n");
        halt();
    }

    if (!imu_init(I2C_BUS))
    {
        halt();
    }
    imu_pose_t pose = {0}; // main owns the pose

    // Servo testing code
    Servo my_servo = {.signal_pin = 16,
                      .period_us = 20000,
                      .min_pulse_us = 500,
                      .max_pulse_us = 2500,
                      .pulse_us = 0};
    servo_init(&my_servo);

    // Motor testing code
    Motor my_motor = {
        .signal_pin = 26,
        .direction_pin1 = 27,
        .direction_pin2 = 28,
        .period_us = 1000,

        // Lab 9
        .encoder_pin = 22,
        .encoder_ppr = 12 / 4 * 45, // motor depends on 211.2/4.0 or 116.16 or 135
    };

    motor_init(&my_motor);
    float duty_motor = pose.pitch_deg / 90;

    // setting motor duty cycle (normal and stretch exercise)
    // motor_set_duty_cycle(&my_motor, 0.4f); // Set initial motor speed to 0%
    motor_set_duty_cycle(&my_motor, duty_motor);    // Set initial motor speed to 0%

    // ina219 code
    INA219 ina;
    if (ina219_init(&ina, I2C_BUS, INA219_ADDR_40, 0.1f, 3.2f) != 0)
    {
        printf("INA219 init failed\r\n");
        halt();
    }
    printf("INA219 raedy\r\n");

    double next_sample_t = get_absolute_time(); // first sample delayed by one period
    sleep_ms(SAMPLE_RATE_MS);

    while (true)
    {
        float pitch_servo = pose.roll_deg + 90;
        float duty_motor = pose.pitch_deg / 180;

        // Servo testing code (normal for 0-180), and stretch goal of based upon pitch angle
        /* servo_set_angle(&my_servo, 0.0f);
                sleep_ms(100);

                servo_set_angle(&my_servo, 90.0f);
                sleep_ms(100);

                servo_set_angle(&my_servo, 180.0f);
                sleep_ms(100); */
        
        servo_set_angle(&my_servo, pitch_servo); // servo angle based upon roll angle (stretch)

        // Motor testing code (normal and stetch goal)
        
        // motor_set_duty_cycle(&my_motor, 0.5f); // Set motor speed to 50%
        // sleep_ms(1000); // Wait for 1 second
        //
        // motor_set_duty_cycle(&my_motor, 0.0f); // Set motor speed to 0%
        // sleep_ms(1000); // Wait for 1 second
        //
        // motor_set_duty_cycle(&my_motor, -0.5f); // Set motor speed to -50%
        // sleep_ms(1000); // Run motor for 2 seconds
        //
        // motor_set_duty_cycle(&my_motor, 0.0f); // Set motor speed to 0%
        // sleep_ms(1000); // Wait for 1 second

        motor_set_duty_cycle(&my_motor, duty_motor); // Set motor speed based upon pitch angle (stretch)
        motor_update(&my_motor);                     // Update encoder count and calculate RPM
        sleep_ms(100);                               

        // INA219
        float volts = ina219_read_bus_voltage(&ina);
        float amps = ina219_read_current_amps(&ina);
        float watts = ina219_read_power_watts(&ina);
        printf("V=%.2f A=%.3f W=%.3f\r\n", volts, amps, watts);

        sleep_ms(100);

        double time_s = to_ms_since_boot(get_absolute_time()) / 1000.0; // convert millisec to sec

        imu_read_pose(&pose); // fills in main's pose; keeps the last one if nothing new arrived

        // Write the data to the SD card
        if (!sd_logger_write(time_s, volts, amps, watts, pitch_servo, duty_motor, &pose, &my_servo, &my_motor))

            sleep_until(next_sample_t);
    }
}
