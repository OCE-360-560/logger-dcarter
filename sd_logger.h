
#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "imu.h"


/** Mount the card and open a file for appending.
 *   path:   file name on the card, e.g. "log.csv"
 *   header: written first, only if the file is new
 * Returns false if the card or file can't be opened. */
void sd_logger_init(void);
void halt(void);

bool sd_logger_open(const char *path, const char *header);

/** Append one line and sync it to the card.
 * Returns false if any part didn't reach the card. */
//bool sd_logger_write(double time_s, float lux, float temp_c);
 bool sd_logger_write(imu_pose_t *pose);

/** Close the file and unmount. Safe to remove after. */
void sd_logger_close(void);
#endif
