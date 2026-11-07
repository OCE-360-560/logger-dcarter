/* Function call for gps creating pin setup and initializing the gps driver.
 Also takes the saved GPS data and allows it to be stored on a SD card .csv file 

Wiring: 
Grove GPS label     Pico 2      Pin
VIN                 3V3(OUT)    36
GND                 GND         38 (any GND)
TXD                 GP5         7
RXD                 GP4         6

Author: David Carter
Liscenc MIT
*/

#include "gps.h"
#include "air530.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "ff.h"      // FatFs: f_mount, f_open, f_write, f_sync
#include "tf_card.h" // pico_fatfs: SD card pins and clock
#include "imu.h"
#include <string.h>

#define GPS_UART uart1
#define GPS_TX_PIN 4 // Pico TX -> GPS RXD
#define GPS_RX_PIN 5 // Pico RX <- GPS TXD
#define KNOT2MPS 0.514444; // convert knots to m/s
static FIL log_file;
static FATFS filesystem;

/** Append one line and sync it to the card.
 * Returns false if any part didn't reach the card. */
static void write_raw_sentence(const char *sentence)
{
    UINT bw; // bytes written
    FRESULT fr = f_write(&log_file, sentence, strlen(sentence), &bw); // into the buffer
    fr = f_write(&log_file, "\r\n", sizeof("\r\n"), &bw);             // into the buffer
    f_sync(&log_file);
};

bool gps_init(void)
{
    air530_config_t config = {
        .uart = GPS_UART,
        .tx_pin = GPS_TX_PIN,
        .rx_pin = GPS_RX_PIN,
        .baud = AIR530_BAUD_DEFAULT,
        // Air530Z on the label? AIR530_CHIP_AIR530Z
        .chip = AIR530_CHIP_AIR530,
    };
    return air530_init(&config);
}

bool gps_update(gps_fix_t *fix)
{
    air530_service();                           // Parse the latest sentences, if any, and update the fix
    air530_fix_t latest = {0};
    if (air530_get_fix(&latest) == false)
    {
        return false;                           // no new fix since last call
    }

    fix->latitude_deg = latest.latitude_deg;
    fix->longitude_deg = latest.longitude_deg;
    fix->altitude_m = latest.altitude_m;
    fix->satellites_used = latest.satellites_used;
    fix->hdop = latest.hdop;
    fix->speed_mps = latest.speed_knots * KNOT2MPS;     // convert knots to m/s
    fix->course_deg = latest.course_deg;
    if (latest.date_valid)
    {
        snprintf(fix->fix_date, 11, "%03u-%02u-%02u", latest.year, latest.month, latest.day);
    }
    if (latest.time_valid)
    {
        snprintf(fix->fix_time, 13, "%02u:%02u:%02u.%03u", latest.hour, latest.minute, latest.second, latest.millisecond);
    }

    return true;
}


bool gps_start_raw_log(const char *path) // every sentence to a file on the SD card
{
    // Open the file for writing, creating it if it doesn't exist
    // static FIL file;
    FRESULT fr = f_open(&log_file, path, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        printf("f_open in gps_start_raw_log.  Failed to open file for writing: %d\n", fr);
        return false;
    }
    air530_set_sentence_callback(write_raw_sentence);
    return true;
}
