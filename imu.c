
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

#define IMU_INT_PIN 12
#define IMU_RESET_PIN 13
#define IMU_REPORT_PERIOD_US (20 * 1000)   // 50 Hz
#define SCAN_TIMEOUT_US 10000
#define RAD_TO_DEG (180.0f / (float)M_PI)

//  Assignmnt 6
#define I2C_BUS i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
// 100 kHz, not 400: the BNO080 stretches the clock, and slower is safer
#define I2C_HZ (100 * 1000)
#define SCAN_TIMEOUT_US 10000


/** Print the address of every device that answers on the bus. */
//static void scan_bus(void)
static void scan_bus(i2c_inst_t *bus)
{
    printf("I2C scan:");
    for (int address = 0x08; address < 0x78; address++)
    { // 0x00-0x07, 0x78-0x7F are reserved
        uint8_t ignored;
        if (i2c_read_timeout_us(bus, address, &ignored, 1, false, SCAN_TIMEOUT_US) >= 0)
        {
            printf("Lab 6 exercise 1:  0x%02X", address);
        }
    }
    printf("\r\n");
}

bool imu_init(i2c_inst_t *bus) {
     bno08x_config_t config = {
        .i2c = i2c1,
        .address = BNO08X_ADDRESS_DEFAULT,
        .int_pin = 12,
        .reset_pin = 13, // or BNO08X_PIN_NOT_WIRED
    };
    if (!bno08x_init(&config)) {
        printf("# IMU not found at 0x%02X\r\n", BNO08X_ADDRESS_DEFAULT);
        scan_bus(bus);
        return false;
    }
    if (!bno08x_enable_report(BNO08X_REPORT_ROTATION_VECTOR, IMU_REPORT_PERIOD_US)) {
        printf("# IMU did not start the rotation vector\r\n");
        return false;
    }
    return true;
    };

    /** Roll, pitch, yaw (Z-Y-X order) from the pose's quaternion. */
static void fill_euler_angles(imu_pose_t *pose) {
    float w = pose->qw, x = pose->qx, y = pose->qy, z = pose->qz;
    pose->roll_deg = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
    float sin_pitch = 2.0f * (w * y - z * x);
    // rounding can push this just past 1 near +-90 degrees, and asinf(1.0001) is NaN
    sin_pitch = fminf(1.0f, fmaxf(-1.0f, sin_pitch));
    pose->pitch_deg = asinf(sin_pitch) * RAD_TO_DEG;
    pose->yaw_deg = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
}

bool imu_read_pose(imu_pose_t *pose) {
    int reports = bno08x_service();
    if (reports < 0) {
        printf("# IMU read error %d\r\n", reports);
    }
    bno08x_quaternion_t quaternion;
    if (!bno08x_get_quaternion(BNO08X_REPORT_ROTATION_VECTOR, &quaternion)) {
        return false;
    }
    pose->qw = quaternion.real;
    pose->qx = quaternion.i;
    pose->qy = quaternion.j;
    pose->qz = quaternion.k;
    pose->accuracy = quaternion.status;
    fill_euler_angles(pose);
    return true;
}