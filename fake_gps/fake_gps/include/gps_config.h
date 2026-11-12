#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

/* Arbitrary simulation epoch, UTC, assigned to the FIRST PPS rising edge.
 * This is deliberately not __DATE__/__TIME__ (host timezone and upload delay).
 * Change these values if an application needs a different simulated epoch.
 */
#define GPS_START_YEAR   2026U
#define GPS_START_MONTH  1U
#define GPS_START_DAY    1U
#define GPS_START_HOUR   0U
#define GPS_START_MINUTE 0U
#define GPS_START_SECOND 0U

#ifndef GPS_FORCE_HSI
#define GPS_FORCE_HSI 0
#endif
#define GPS_UART_BAUD 9600U
#define GPS_TIMER_HZ 10000U
#define GPS_PERIOD_TICKS 10000U
#define GPS_PULSE_TICKS 1000U
/* PWM2: rising edge at 900ms, falling at timer wrap, sentence at 100ms.
 * Relative to PPS rising: pulse falls at +100ms; NMEA starts at +200ms.
 */
#define GPS_PPS_COMPARE (GPS_PERIOD_TICKS - GPS_PULSE_TICKS)
#define GPS_NMEA_COMPARE 1000U
#define GPS_TX_LATEST_TICK 2000U /* Drop work >100ms late, never catch up. */
/* Normal receivers report the time of the preceding PPS. Keep 0 unless a
 * measured, firmware-specific integration requires a different association.
 */
#ifndef GPS_RMC_SECOND_OFFSET
#define GPS_RMC_SECOND_OFFSET 0
#endif
/* A = deliberately simulated valid fix, NOT a physical GNSS fix.
 * Coordinates below are placeholders (0N, 0E), never navigation data.
 */
#define GPS_RMC_STATUS 'A'

#if GPS_RMC_SECOND_OFFSET != 0
#error "Combined GPS/IMU firmware requires preceding-PPS RMC association (offset=0)."
#endif

#endif
