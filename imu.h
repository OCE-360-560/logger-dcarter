/*
 * imu.h
 * The IMU setup for the BNO080 sensor.
 *
 * Author: David Carter
 * License: MIT (see LICENSE)
 */
#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"   // i2c_inst_t, for imu_init()

typedef struct {          // a new type, like int or float
    float roll_deg;       // its members
    float pitch_deg;
    uint8_t accuracy;     // uint8_t: a whole number 0 to 255
} pose_t;

// pose_t now;                     // one variable holds them all
// now.roll_deg = 12.5f;           // a dot picks one member
// pose_t level = {.pitch_deg = 0.0f};   // name what you set; the rest are 0

/** One orientation reading. Angles in degrees; the quaternion is unitless. */
typedef struct {
    float qw, qx, qy, qz;   // quaternion (real part first)
    float roll_deg;         // about the board's X axis
    float pitch_deg;        // about the board's Y axis
    float yaw_deg;          // about the board's Z axis
    uint8_t accuracy;       // 0 unreliable, 1 low, 2 medium, 3 high
} imu_pose_t;

/**
 * Start the BNO080's rotation vector report.
 *   bus: the I2C bus the IMU is on, already set up by main
 * Returns false if the IMU doesn't answer; the I2C bus scan is printed to serial.
 */
bool imu_init(i2c_inst_t *bus);
/**
 * Read any new reports from the IMU.
 *   pose: filled with the newest reading
 * Returns true if a new reading arrived since the last call; false leaves pose unchanged.
 */
bool imu_read_pose(imu_pose_t *pose);
#endif
