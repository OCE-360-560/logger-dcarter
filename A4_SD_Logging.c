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
#include "hardware/spi.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "tf_card.h"   // pico_fatfs: SD card pins and clock
#include "sensors.h"   // photocell and LM19 temperature sensor
#include "sd_logger.h" // SD card logging
#include "hardware/i2c.h"
#include "bno08x.h"
#include "imu.h"
#include "gps.h"
#include "air530.h"

#define SAMPLE_PERIOD_MS 100
#define LOG_HEADER "time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy\r\n"
#define PATH "Logger_data_v1.csv" // file name on the SD card

// #define LOG_HEADER_GPS "fix_date, fix_time, gps_valid, lat_deg, lon_deg, alt_m, satellites, hdop, speed_mps,course_deg\r\n"
#define PATH_GPS "nmea.txt" // file name on the SD card

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

    // // Stretch goal: change the orientation reference by taring the IMU
    // printf("Press the Y key to activate the function tare to change orientation reference\n");
    // int key;
    // scanf("%d", &key);  // Reads a key press (integer)
    // if (key == 'y' || key == 'Y')
    // {
    //     imu_tare();
    // }

    init_i2c_bus();    // initialize the I2C bus for the IMU and INA219
    imu_init(I2C_BUS); // initialize the IMU and start the rotation vector report

    sleep_ms(SAMPLE_PERIOD_MS);

    if (!sd_logger_open(PATH, LOG_HEADER))
    {
        printf("# SD open failed\r\n");
        halt();
    }

    gps_start_raw_log(PATH_GPS);

    if (!imu_init(I2C_BUS))
    {
        halt();
    }
    imu_pose_t pose = {0}; // main owns the pose

    // GPS Update
    //gps_fix_t fix = {0}; // one variable; every member starts at 0
    // fix.latitude_deg;
    // fix.valid;
    // fix.longitude_deg;
    // fix.altitude_m;
    // fix.satellites_used;
    // fix.hdop;
    // fix.course_deg;
    // fix.speed_mps;
    // fix.fix_date[11];
    // fix.fix_time[13];

    //gps_update(&fix); // & passes fix's address, so the function can fill in main's fix


        // GPS test
        // test first, before gps_init(): no real sentence can arrive yet
        air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2E");
        // gps_init(); // sets up the UART and the driver, on the kit pins
        gps_fix_t test = {0};
        if (true)
        {
            printf("test: %s %.6f, %.6f\r\n", test.fix_time, test.latitude_deg, test.longitude_deg);
        }
        // expect: test: 14:30:12.000 41.485390, -71.52613


    sleep_ms(SAMPLE_PERIOD_MS);
    while (true)
    {
        double time_s = to_ms_since_boot(get_absolute_time()) / 1000.0; // convert millisec to sec
        float lux = sensors_read_lux();                                 // read the photocell once, returns illuminance in lux
        float temp_c = sensors_read_celsius();                          // read the LM19 once, returns temperature in

        imu_read_pose(&pose); // fills in main's pose; keeps the last one if nothing new arrived

        //  if (!sd_logger_write(time_s, lux, temp_c))
        if (!sd_logger_write(time_s, lux, temp_c, &pose))
            // printf("# SD write failed\r\n");

            printf("Roll, Pitch, Yaw, Accuracy\r\n");
        printf("%.1f, %.1f, %.1f, %u\r\n",
               pose.roll_deg, pose.pitch_deg, pose.yaw_deg, pose.accuracy);
        /*  printf("Time, lux, temp_c\r\n");
         printf("%.3f, %.3f, %.3f\r\n",
                time_s, lux, temp_c); */

        sleep_ms(SAMPLE_PERIOD_MS);
    }
}
