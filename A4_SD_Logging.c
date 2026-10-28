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

#define SAMPLE_PERIOD_MS 1000
//  Assignmnt 6
#define I2C_BUS i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
// 100 kHz, not 400: the BNO080 stretches the clock, and slower is safer
#define I2C_HZ (100 * 1000)
#define SCAN_TIMEOUT_US 10000

// README from bno08x
// You set up the bus; the driver never does (so other I2C parts can share it).

/** Set up the I2C bus that the IMU (and later the INA219) share. */
static void init_i2c_bus(void)
{
    i2c_init(I2C_BUS, I2C_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // added as step 6 of lab 6, to initialize the IMU from bno08x.h
    bno08x_config_t config = {
        .i2c = i2c1,
        .address = BNO08X_ADDRESS_DEFAULT,
        .int_pin = 12,
        .reset_pin = 13, // or BNO08X_PIN_NOT_WIRED
    };
    if (!bno08x_init(&config))
    { /* not found */
    }
    bno08x_enable_report(BNO08X_REPORT_ROTATION_VECTOR, 20000); // every 20 ms

    // in imu_init(), next to the rotation vector: one line per report you need
    bno08x_enable_report(BNO08X_REPORT_ACCELEROMETER, 20000);  // m/s², with gravity
    bno08x_enable_report(BNO08X_REPORT_GYROSCOPE, 20000);      // rad/s
    bno08x_enable_report(BNO08X_REPORT_MAGNETIC_FIELD, 20000); // µT
    // also: BNO08X_REPORT_LINEAR_ACCELERATION (gravity removed), _GRAVITY
    // then, after bno08x_service():
    bno08x_vector_t gyro;
    if (bno08x_get_vector(BNO08X_REPORT_GYROSCOPE, &gyro))
    { // true if new
        printf("%.3f %.3f %.3f  status %u\r\n", gyro.x, gyro.y, gyro.z, gyro.status);
    }
}
/** Print the address of every device that answers on the bus. */
static void scan_bus(void)
{
    printf("I2C scan:");
    for (int address = 0x08; address < 0x78; address++)
    { // 0x00-0x07, 0x78-0x7F are reserved
        uint8_t ignored;
        if (i2c_read_timeout_us(I2C_BUS, address, &ignored, 1, false, SCAN_TIMEOUT_US) >= 0)
        {
            printf("Lab 6 exercise 1:  0x%02X", address);
        }
    }
    printf("\r\n");
}

int main()
{
    stdio_init_all();

    sensors_init(); // initialize the ADC and both sensor pins

    init_i2c_bus(); // initialize the I2C bus for the IMU and INA219
    bno08x_config_t imu_config = {
        .i2c = I2C_BUS,
        .address = BNO08X_ADDRESS_DEFAULT,
        .int_pin = 12,
        .reset_pin = 13,
        // INT and RST are both required
    };
    if (!bno08x_init(&imu_config))
    {
        printf("# IMU init failed\r\n");
        halt();
    }
    bno08x_enable_report(BNO08X_REPORT_ROTATION_VECTOR, 20000);
    // every 20 ms = 50 Hz

    sd_logger_init(); // initialize the SD card and SPI pins

    sleep_ms(SAMPLE_PERIOD_MS);

    if (!sd_logger_open("multi_source_cali_ref.csv", "time_s, lux, temp_C\r\n"))
    {
        printf("# SD open failed\r\n");
        halt();
    }
    // Lab 6 step 7

    sleep_ms(SAMPLE_PERIOD_MS);
    while (true)
    {
        scan_bus();
        bno08x_service();
        bno08x_quaternion_t q;
        if (bno08x_get_quaternion(BNO08X_REPORT_ROTATION_VECTOR, &q))
        {
            printf("Real: %.3f i: %.3f j: %.3f k: %.3f  status %u\r\n", q.real, q.i, q.j, q.k, q.status);
        }

        double time_s = to_ms_since_boot(get_absolute_time()) / 1000.0; // convert millisec to sec

        float lux = sensors_read_lux();        // read the photocell once, returns illuminance in lux
        float temp_c = sensors_read_celsius(); // read the LM19 once, returns temperature in

        if (!sd_logger_write(time_s, lux, temp_c))
            // printf("# SD write failed\r\n");
            sleep_ms(SAMPLE_PERIOD_MS);
    }
}
