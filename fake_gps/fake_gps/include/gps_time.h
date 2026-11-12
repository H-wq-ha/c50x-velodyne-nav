#ifndef GPS_TIME_H
#define GPS_TIME_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
} gps_time_t;

enum { GPS_NMEA_CAPACITY = 83 }; /* NMEA maximum 82 bytes plus terminator. */
bool gps_time_valid(const gps_time_t *time);
void gps_time_advance(gps_time_t *time);
size_t gps_rmc_format(char *buffer, size_t capacity, const gps_time_t *time,
                      char status);
/* All checks use the hardware timer phase; stale sentences are discarded. */
bool gps_tx_window(uint32_t timer_tick);
#endif
