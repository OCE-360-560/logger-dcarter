/* Function call for gps creating pin setup and initializing the gps driver. 
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


#define GPS_UART uart1
#define GPS_TX_PIN 4 // Pico TX -> GPS RXD
#define GPS_RX_PIN 5 // Pico RX <- GPS TXD

static FIL log_file;
static FATFS gps_filesystem;
UINT bw; // bytes written

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
        air530_service();   // Parse the latest sentences, if any, and update the fix
        air530_fix_t latest={0};
        if (air530_get_fix(&latest)==false)
        {
            return false; // no new fix since last call
        }

    fix->latitude_deg = latest.latitude_deg;  
    fix->longitude_deg = latest.longitude_deg;
    fix->altitude_m = latest.altitude_m;
    fix->satellites_used = latest.satellites_used;
    fix->hdop = latest.hdop;
    fix->speed_mps = latest.speed_knots * 0.514444; //convert knots to m/s
    fix->course_deg = latest.course_deg;
    // fix->fix_date[11] = latest.date_valid;  
    if (latest.date_valid)
    {
        snprintf(fix->fix_date, 11, "%03u-%02u-%02u", latest.year, latest.month, latest.day);
    }
    // fix->fix_time[13] = latest.time_valid;
    if (latest.time_valid)
    {
        snprintf(fix->fix_time, 13, "%02u:%02u:%02u.%03u", latest.hour, latest.minute, latest.second, latest.millisecond);
    }
    
    return true;
}

 bool gps_start_raw_log(const char *path, gps_fix_t *fix)  // every sentence to a file on the SD card
{
    // Open the file for writing, creating it if it doesn't exist
   // static FIL file;
    FRESULT fr = f_open(&log_file, path, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        printf("Failed to open file for writing: %d\n", fr);
        return false;
    }
    
    // step 12
    char sentence[128];
    write_raw_sentence(&sentence, &fix);
    air530_set_sentence_callback(write_raw_sentence);

}

/** Append one line and sync it to the card.
 * Returns false if any part didn't reach the card. */
 void write_raw_sentence(const char *sentence, gps_fix_t *fix)
{
    int sentence_len =  snprintf(sentence, sizeof sentence, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.10f, %.10f, %u\r\n",
             fix->latitude_deg, fix->longitude_deg, fix->altitude_m, fix->satellites_used, fix->hdop, fix->speed_mps, fix->course_deg,
             fix->fix_date, fix->fix_time);

    f_write(&log_file, sentence, sentence_len, &bw); // into the buffer
    f_sync(&log_file);  
};



