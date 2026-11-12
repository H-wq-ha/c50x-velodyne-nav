#ifndef SERIAL_HAL_H
#define SERIAL_HAL_H
#include "board.h"
#include "imu_protocol.h"
typedef enum {
    SERIAL_CMD_NONE, SERIAL_CMD_PING, SERIAL_CMD_START,
    SERIAL_CMD_STOP, SERIAL_CMD_STATS, SERIAL_CMD_INVALID
} serial_command_t;
typedef struct {
    volatile uint32_t tx_dropped, rx_dropped, uart_errors;
} serial_stats_t;
extern serial_stats_t g_serial_stats;
bool Serial_Init(void);
bool Serial_Write(const uint8_t *data, uint16_t size);
bool Serial_SendSample(const imu_sample_t *sample);
bool Serial_SendStatus(const uint32_t words[16]);
serial_command_t Serial_PollCommand(void);
void Serial_Service(void);
void Serial_OnTxComplete(void);
void Serial_OnRxComplete(void);
void Serial_OnError(void);
#endif
