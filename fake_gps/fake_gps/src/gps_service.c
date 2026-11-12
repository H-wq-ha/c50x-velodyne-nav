#include "board.h"
#include "gps_config.h"

void GPS_Service(void)
{
    static char nmea[GPS_NMEA_CAPACITY];
    gps_time_t time;
    uint32_t sequence;
    if (!GPS_TakeSentence(&time, &sequence)) return;
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
        ++g_gps.uart_errors;
        Board_Fatal(10U);
    }
    size_t length = gps_rmc_format(nmea, sizeof(nmea), &time, GPS_RMC_STATUS);
    if (length == 0U) Board_Fatal(11U);
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint32_t phase = TIM1->CNT;
    if (sequence == g_gps.pps_count && gps_tx_window(phase) &&
        (TIM1->SR & TIM_SR_CC1IF) == 0U) {
        g_gps.last_tx_tick = phase;
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)nmea, (uint16_t)length) != HAL_OK)
            Board_Fatal(12U);
    } else ++g_gps.nmea_dropped;
    __set_PRIMASK(saved);
}
