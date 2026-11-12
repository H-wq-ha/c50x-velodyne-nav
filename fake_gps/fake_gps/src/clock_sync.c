#include "clock_sync.h"

uint64_t Clock_EpochUs(const gps_time_t *time)
{
    if (!gps_time_valid(time)) return 0U;
    gps_time_t day = {2000U, 1U, 1U, 23U, 59U, 59U};
    uint64_t days = 10957U; /* Days from 1970-01-01 to 2000-01-01. */
    while (day.year != time->year || day.month != time->month || day.day != time->day) {
        gps_time_advance(&day);
        day.hour = 23U;
        day.minute = 59U;
        day.second = 59U;
        ++days;
    }
    return (days * 86400ULL + time->hour * 3600U + time->minute * 60U + time->second)
           * 1000000ULL;
}

uint64_t Clock_ExtendCapture(uint64_t overflow_base, uint16_t captured, bool overflow)
{
    /* Capture and pending overflow may arrive in the same ISR. A low CCR is
     * after the wrap, a high CCR is before it. IRQ latency must stay <32ms.
     */
    return overflow_base + captured + (overflow && captured < 0x8000U ? 65536ULL : 0ULL);
}

void Clock_OnPps(sync_clock_t *clock, uint64_t captured_us)
{
    if (!clock->valid) {
        clock->first_pps_us = captured_us;
        clock->valid = true;
    } else {
        uint64_t interval = captured_us - clock->last_pps_us;
        uint64_t error = interval > 1000000ULL ? interval - 1000000ULL : 1000000ULL - interval;
        if (error > clock->interval_error_max_us)
            clock->interval_error_max_us = error > UINT32_MAX ? UINT32_MAX : (uint32_t)error;
    }
    clock->last_pps_us = captured_us;
    ++clock->pps_count;
}

bool Clock_Stamp(const sync_clock_t *clock, uint64_t captured_us, sync_stamp_t *stamp)
{
    if (!clock->valid || captured_us < clock->first_pps_us) return false;
    uint64_t elapsed = captured_us - clock->first_pps_us;
    stamp->monotonic_us = captured_us;
    stamp->epoch_us = clock->epoch_start_us + elapsed;
    stamp->hour_us = (uint32_t)(stamp->epoch_us % 3600000000ULL);
    stamp->pps_count = (uint32_t)(elapsed / 1000000ULL) + 1U;
    return true;
}
