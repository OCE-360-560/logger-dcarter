/*
 * A4_SD_Logging.c
 * Environmental logger: reads light
 * and temperature once per sample
 * period, prints each reading with the
 * corresponding time and voltage, and
 * appends it to multi_source_cali_ref.csv.
 *
 * Author: David Carter
 * Date: October 18th, 2026
 * License: MIT (see LICENSE)
 */

#include <stdio.h>
#include "pico/stdlib.h"
// #include "hardware/spi.h"
#include <stdio.h>
// #include "hardware/gpio.h"
// #include "hardware/adc.h"
// #include "hardware/spi.h"
// #include "tf_card.h"   // pico_fatfs: SD card pins and clock
#include "sensors.h"   // photocell and LM19 temperature sensor
#include "sd_logger.h" // SD card logging
// #include "hardware/i2c.h"
// #include "bno08x.h"
// #include "imu.h"
// #include "gps.h"
// #include "air530.h"
#include "servo.h"
#include "motor.h"
#include "ina219.h"

#define SAMPLE_PERIOD_MS 1000
#define SAMPLE_RATE_MS 100
#define LOG_HEADER "time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy, " \
                   "fix_date, fix_time, gps_valid, lat_deg, lon_deg, alt_m, satellites, hdop, speed_mps, course_deg\r\n"
#define PATH "GPS_logger_data.csv" // file name on the SD card

#define PATH_GPS "nmea.txt" // file name on the SD card
// #define SAMPLE_PERIOD_MS 100
#//define LOG_HEADER "time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy\r\n"
// #define PATH "Logger_data_v3.csv" // file name on the SD card

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

    sensors_init();   // initialize the ADC and both sensor pins
    sd_logger_init(); // initialize the SD card and SPI pins

    init_i2c_bus();    // initialize the I2C bus for the IMU and INA219
    imu_init(I2C_BUS); // initialize the IMU and start the rotation vector report

    sleep_ms(SAMPLE_PERIOD_MS);
    // sleep_until(SAMPLE_PERIOD_MS);

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

    gps_fix_t fix = {0};

    if (!gps_init() || !gps_start_raw_log(PATH_GPS))
    {
        halt();
    }

    // Servo testing code
    Servo my_servo = {.signal_pin = 16,
                      .period_us = 20000,
                      .min_pulse_us = 500,
                      .max_pulse_us = 2500};

    servo_init(&my_servo);

    // Motor testing code
    Motor my_motor = {.signal_pin = 26,
                      .direction_pin1 = 27,
                      .direction_pin2 = 28,
                      .period_us = 1000,

                      // Lab 9
                    .encoder_pin = 22,
                    .encoder_ppr= 12/4*45, // motor depends on 211.2/4.0 or 116.16 or 135
                    };
    motor_init(&my_motor);
    float duty= pose.pitch_deg/90;
    // motor_set_duty_cycle(&my_motor, 0.4f); // Set initial motor speed to 0%
    motor_set_duty_cycle(&my_motor, duty); // Set initial motor speed to 0%

    
//ina219 code
INA219 ina;
if(ina219_init(&ina, I2C_BUS, INA219_ADDR_40, 0.1f, 3.2f) !=0){
    printf("INA219 init failed\r\n");
    halt();
}
printf("INA219 raedy\r\n");

    double next_sample_t = get_absolute_time(); // first sample delayed by one period
    sleep_ms(SAMPLE_RATE_MS);
   
    while (true)
    {
// Servo testing code
/* servo_set_angle(&my_servo, 0.0f);
        sleep_ms(100);

        servo_set_angle(&my_servo, 90.0f);
        sleep_ms(100);

        servo_set_angle(&my_servo, 180.0f);
        sleep_ms(100); */

        float pitch_servo=pose.roll_deg+90;        
        float duty_motor= pose.pitch_deg/180;

        // Motor testing code
        servo_set_angle(&my_servo, pitch_servo);
        motor_set_duty_cycle(&my_motor, duty_motor); // Set motor speed to 50%
        sleep_ms(100); // Run motor for 2 seconds

        // motor_set_duty_cycle(&my_motor, duty); // Set motor speed to 0%
        // sleep_ms(1000); // Wait for 1 second
        //
        // motor_set_duty_cycle(&my_motor, duty); // Set motor speed to -50%
        // sleep_ms(1000); // Run motor for 2 seconds
        //
        // motor_set_duty_cycle(&my_motor, duty); // Set motor speed to 0%
        // sleep_ms(1000); // Wait for 1 second
        //
        // Stretch exercise
        // float duty= pose.pitch_deg/90;
        // motor_set_duty_cycle(&my_motor, duty); // Set initial motor speed to 0%
        // motor_update(&my_motor); // Update encoder count and calculate RPM
        printf("Pitch  %.6f\r\n", pose.pitch_deg);
    
     // INA219
        float volts = ina219_read_bus_voltage(&ina);
        float amps = ina219_read_current_amps(&ina);
        float watts = ina219_read_power_watts(&ina);
        printf("V=%.2f A=%.3f W=%.3f\r\n", volts, amps, watts);

        sleep_ms(100);


        // Stretch goal: change the orientation reference by taring the IMU
        int key = getchar_timeout_us(0); // PICO_ERROR_TIMEOUT if nothing was typed
        if (key == 'y')                  // type y in the Serial Monitor's send box
        {
            imu_tare();
        }

        double time_s = to_ms_since_boot(get_absolute_time()) / 1000.0; // convert millisec to sec
        float lux = sensors_read_lux();                                 // read the photocell once, returns illuminance in lux
        float temp_c = sensors_read_celsius();                          // read the LM19 once, returns temperature in

        imu_read_pose(&pose); // fills in main's pose; keeps the last one if nothing new arrived

        // GPS test
        next_sample_t = delayed_by_ms(
            next_sample_t, SAMPLE_RATE_MS);
        while (!time_reached(next_sample_t))
        {
            gps_update(&fix);
        }
        if ((gps_update(&fix)))
        {
            printf("fix: %s %.6f, %.6f\r\n", fix.fix_time, fix.latitude_deg, fix.longitude_deg);
        }

        // Write the data to the SD card
        if (!sd_logger_write(time_s, lux, temp_c, &pose, &fix))
            // printf("# SD write failed\r\n");

        //     printf("Roll, Pitch, Yaw, Accuracy\r\n");
        // printf("%.1f, %.1f, %.1f, %u\r\n",
        //        pose.roll_deg, pose.pitch_deg, pose.yaw_deg, pose.accuracy);

        sleep_until(next_sample_t);
    }
}
