#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H
#include "gps_time.h"

typedef struct {
    uint64_t monotonic_us, epoch_us;
    uint32_t hour_us, pps_count;
} sync_stamp_t;
typedef struct {
    uint64_t epoch_start_us, first_pps_us, last_pps_us;
    uint32_t pps_count, interval_error_max_us;
    bool valid;
} sync_clock_t;
uint64_t Clock_EpochUs(const gps_time_t *time);
uint64_t Clock_ExtendCapture(uint64_t overflow_base, uint16_t captured, bool overflow);
void Clock_OnPps(sync_clock_t *clock, uint64_t captured_us);
bool Clock_Stamp(const sync_clock_t *clock, uint64_t captured_us, sync_stamp_t *stamp);
#endif
