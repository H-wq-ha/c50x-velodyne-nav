#include "gps_time.h"
#include "gps_config.h"
#include <stdio.h>

static bool leap(uint16_t year)
{
    return year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U);
}

static uint8_t month_days(uint16_t year, uint8_t month)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return (uint8_t)(days[month - 1U] + (month == 2U && leap(year) ? 1U : 0U));
}

bool gps_time_valid(const gps_time_t *time)
{
    return time != NULL && time->year >= 2000U && time->year <= 2099U &&
           time->month >= 1U && time->month <= 12U && time->day >= 1U &&
           time->day <= month_days(time->year, time->month) &&
           time->hour < 24U && time->minute < 60U && time->second < 60U;
}

void gps_time_advance(gps_time_t *time)
{
    if (++time->second < 60U) return;
    time->second = 0U;
    if (++time->minute < 60U) return;
    time->minute = 0U;
    if (++time->hour < 24U) return;
    time->hour = 0U;
    if (++time->day <= month_days(time->year, time->month)) return;
    time->day = 1U;
    if (++time->month <= 12U) return;
    time->month = 1U;
    ++time->year;
}

size_t gps_rmc_format(char *buffer, size_t capacity, const gps_time_t *time,
                      char status)
{
    if (buffer == NULL || capacity == 0U) return 0U;
    buffer[0] = '\0';
    if (!gps_time_valid(time) || (status != 'A' && status != 'V')) return 0U;
    /* No float printf, no dynamic allocation. Includes all legacy RMC fields.
     * Time denotes the preceding PPS, not the delayed UART transmission time.
     */
    int count = snprintf(buffer, capacity,
                         "$GPRMC,%02u%02u%02u.000,%c,0000.0000,N,00000.0000,E,0.00,0.00,%02u%02u%02u,,,",
                         (unsigned)time->hour, (unsigned)time->minute,
                         (unsigned)time->second, status, (unsigned)time->day,
                         (unsigned)time->month, (unsigned)(time->year % 100U));
    if (count < 0 || (size_t)count + 6U > capacity || (size_t)count + 5U > 82U) {
        buffer[0] = '\0';
        return 0U;
    }
    uint8_t checksum = 0U;
    for (int i = 1; i < count; ++i) checksum ^= (uint8_t)buffer[i];
    static const char hex[] = "0123456789ABCDEF";
    buffer[count++] = '*';
    buffer[count++] = hex[checksum >> 4U];
    buffer[count++] = hex[checksum & 15U];
    buffer[count++] = '\r';
    buffer[count++] = '\n';
    buffer[count] = '\0'; /* DMA length excludes this terminator. */
    return (size_t)count;
}

bool gps_tx_window(uint32_t timer_tick)
{
    return timer_tick >= GPS_NMEA_COMPARE && timer_tick < GPS_TX_LATEST_TICK;
}
