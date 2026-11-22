// sd_logger.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "ff.h"      // FatFs: f_mount, f_open, f_write, f_sync
#include "tf_card.h" // pico_fatfs: SD card pins and clock
#include "imu.h"
#include "servo.h"
#include "motor.h"


#define PIN_SCK 18
#define PIN_TX 19 // PICO: to the card's DI
#define PIN_RX 20 // POCI: from the card's DO
#define PIN_CS 21

static FIL log_file;
static FATFS filesystem;
UINT bw; // bytes written

// Describe the wiring:
static pico_fatfs_spi_config_t sd_config = {
    spi0,             // SPI instance
    CLK_SLOW_DEFAULT, // 100 kHz while the card starts
    12500 * KHZ,      // 12.5 MHz after that
    20,               // DO  (SPI0 RX)
    21,               // CS
    18,               // CLK (SPI0 SCK)
    19,               // DI  (SPI0 TX)
    true              // pull-ups on DO and DI
};

 void sd_logger_init(void)
{
    spi_init(spi0, 1000 * 1000);               // 1 MHz (slow and safe)
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI); // spio gets 3 pins
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_init(PIN_CS); // CS: plain output
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);                        // idle high
    uint8_t tx = 0xA5;                          // out on TX, in on RX
}

void halt(void){
    printf("Halting program.\r\n");
    return; // return to main() and let it loop forever
}

/** Mount the card and open a file for appending.
 *   path:   file name on the card, e.g. "log.csv"
 *   header: written first, only if the file is new
 * Returns false if the card or file can't be opened. */
bool sd_logger_open(const char *path, const char *header)
{
    // Step 10   File save name
    pico_fatfs_set_config(&sd_config); // before f_mount

    FRESULT fr = f_mount(&filesystem, "", 1);
    if (fr != FR_OK)
        printf("f_mount: %d\r\n", fr); // 0 means FR_OK

    fr = f_open(&log_file, path, FA_OPEN_APPEND | FA_WRITE);

    if (fr != FR_OK)
        printf("f_open: %d\r\n", fr);
    if (f_size(&log_file) == 0) // header, new file only
    {
        f_write(&log_file, header, strlen(header), &bw);
    }
};

/** Append one line and sync it to the card.
 * Returns false if any part didn't reach the card. */
bool sd_logger_write(double time_s, float volts, float amps, float watts, float pitch_servo, float duty_motor, imu_pose_t *pose, Servo *servo,Motor *motor)
{
    char line[256];
    int len = snprintf(line, sizeof line, "%.3f, %.6f, %.6f, %.6f, %.6f, %.6f,%.6f, %.6f, %d, %.6f\r\n",
                       time_s, volts, amps,
                       watts, pitch_servo, duty_motor, pose->pitch_deg, pose->roll_deg, servo->pulse_us, motor->rpm);

    f_write(&log_file, line, len, &bw); // into the buffer
    f_sync(&log_file);                  // onto the card
};

/** Close the file and unmount. Safe to remove after. */
void sd_logger_close(void)
{
    f_close(&log_file);
    f_unmount("");
};

