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
#include <string.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "ff.h"        // FatFs: f_mount, f_open, f_write, f_sync
#include "tf_card.h"   // pico_fatfs: SD card pins and clock
#include "sensors.h"   // photocell and LM19 temperature sensor
#include "sd_logger.h" // SD card logging
#include "hardware/i2c.h"
#include "bno08x.h"
#include "imu.h"

#define SAMPLE_PERIOD_MS 100
#define LOG_HEADER "time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy\r\n"
#define PATH "Logger_data_v3.csv" // file name on the SD card

//  Assignmnt 6
#define I2C_BUS i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
// 100 kHz, not 400: the BNO080 stretches the clock, and slower is safer
#define I2C_HZ (100 * 1000)
#define SCAN_TIMEOUT_US 10000

// README from bno08x
// You set up the bus; the driver never does (so other I2C parts can share it).

/** Print the address of every device that answers on the bus. */

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
    sleep_ms(SAMPLE_PERIOD_MS);
    // sleep_until(SAMPLE_PERIOD_MS);
    while (true)
    {
        // Stretch goal: change the orientation reference by taring the IMU
        int key = getchar_timeout_us(0); // PICO_ERROR_TIMEOUT if nothing was typed
        if (key == 'y')                  // type y in the Serial Monitor's send box
        {
            imu_tare();
        }
        // else
        // {
        //     imu_tare_clear();
        // }

        double time_s = to_ms_since_boot(get_absolute_time()) / 1000.0; // convert millisec to sec

        float lux = sensors_read_lux();        // read the photocell once, returns illuminance in lux
        float temp_c = sensors_read_celsius(); // read the LM19 once, returns temperature in

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
        // sleep_until(time_s + SAMPLE_PERIOD_MS);
    }
}
