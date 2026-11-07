/*
 * gps.h
 * The GPS setup for the GPS sensor.
 * Uses the Air530 driver from GITHUB, provided to us.
 * 
 * Author: David Carter
 * License: MIT (see LICENSE)
 */
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"


typedef struct
{
    bool valid;                 // the receiver has a fix: position fields mean something
    double latitude_deg;        // decimal degrees, north positive
    double longitude_deg;       // decimal degrees, east positive (Rhode Island is negative) 
    float altitude_m;           // above mean sea level
    uint8_t satellites_used;    // in this fix
    float hdop;                 // horizontal dilution of precision: lower is better
    float speed_mps;            // speed over ground, m/s
    float course_deg;           // direction of travel over ground, degrees true
    char fix_date[11];          // UTC date of this fix, "2026-10-29"; "" until known
    char fix_time[13];          // UTC time of this fix, "14:30:12.000"; "" until known
} gps_fix_t;

bool gps_init(void);                        // the UART and the driver, on the kit pins
bool gps_update(gps_fix_t *fix);            // never waits; true once per new fix
bool gps_start_raw_log(const char *path);  // every sentence to a file on the SD card

#endif