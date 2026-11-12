#include "serial_hal.h"
#include <string.h>

#define TX_CAPACITY 1024U
#define RX_CAPACITY 128U
static uint8_t tx_buffer[TX_CAPACITY];
static uint8_t rx_buffer[RX_CAPACITY];
static volatile uint16_t tx_head, tx_tail, rx_head, rx_tail;
static volatile uint16_t tx_inflight;
static volatile bool tx_busy;
static uint8_t rx_byte;
static volatile uint32_t tx_started_ms;
serial_stats_t g_serial_stats;

/* Caller masks interrupts. DMA reads only bytes already committed to the ring. */
static void start_tx(void)
{
    if (tx_busy || tx_head == tx_tail) {
        return;
    }
    tx_inflight = (uint16_t)(tx_head > tx_tail ? (uint32_t)tx_head-tx_tail : TX_CAPACITY-tx_tail);
    tx_busy = true;
    tx_started_ms = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(&huart2, tx_buffer + tx_tail, tx_inflight) != HAL_OK) {
        tx_busy = false;
        ++g_serial_stats.uart_errors;
    }
}
bool Serial_Init(void)
{
    return HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) == HAL_OK;
}
bool Serial_Write(const uint8_t *data, uint16_t size)
{
    if (size == 0U) {
        return true;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t available = (uint16_t)((tx_tail + TX_CAPACITY - tx_head - 1U) % TX_CAPACITY);
    if (size > available) {
        ++g_serial_stats.tx_dropped;
        __set_PRIMASK(primask);
        return false;
    }
    for (uint16_t i = 0; i < size; ++i) {
        tx_buffer[tx_head] = data[i];
        tx_head = (uint16_t)((tx_head + 1U) % TX_CAPACITY);
    }
    start_tx();
    __set_PRIMASK(primask);
    return true;
}
bool Serial_SendSample(const imu_sample_t *sample)
{
    uint8_t packet[IMU_PACKET_SIZE];
    IMU_PackBinary(sample, packet);
    return Serial_Write(packet, sizeof(packet));
}
bool Serial_SendStatus(const uint32_t words[16])
{
    uint8_t packet[IMU_PACKET_SIZE];
    IMU_PackStatus(words, packet);
    return Serial_Write(packet, sizeof(packet));
}
void Serial_OnTxComplete(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_tail = (uint16_t)((tx_tail + tx_inflight) % TX_CAPACITY);
    tx_busy = false;
    start_tx();
    __set_PRIMASK(primask);
}
void Serial_OnRxComplete(void)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail) {
        ++g_serial_stats.rx_dropped;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}
void Serial_OnError(void)
{
    ++g_serial_stats.uart_errors;
    if ((huart2.ErrorCode & HAL_UART_ERROR_DMA) != 0U) {
        /* HAL has ended the DMA transfer before entering this callback. */
        tx_tail = tx_head;
        tx_busy = false;
        ++g_serial_stats.tx_dropped;
    }
    /* ORE ends RX in HAL; FE/NE may keep it running. No blocking work in IRQ. */
    if (huart2.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}
void Serial_Service(void)
{
    /* Recover an unexpected stalled TX outside the interrupt context. */
    if (tx_busy && (uint32_t)(HAL_GetTick() - tx_started_ms) > 250U) {
        HAL_UART_AbortTransmit(&huart2);
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        tx_tail = tx_head;
        tx_busy = false;
        ++g_serial_stats.tx_dropped;
        ++g_serial_stats.uart_errors;
        __set_PRIMASK(primask);
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    start_tx();
    __set_PRIMASK(primask);
}
serial_command_t Serial_PollCommand(void)
{
    static char command[32];
    static unsigned used;
    static bool collecting;
    while (rx_tail != rx_head) {
        uint8_t ch = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
        if (ch == '@') {
            collecting = true;
            used = 0U;
        } else if (collecting && ch == '\n') {
            command[used] = '\0';
            collecting = false;
            if (strcmp(command, "PING") == 0) return SERIAL_CMD_PING;
            if (strcmp(command, "START") == 0) return SERIAL_CMD_START;
            if (strcmp(command, "STOP") == 0) return SERIAL_CMD_STOP;
            if (strcmp(command, "STATS") == 0) return SERIAL_CMD_STATS;
            return SERIAL_CMD_INVALID;
        } else if (collecting && ch != '\r') {
            if (used + 1U < sizeof(command)) {
                command[used++] = (char)ch;
            } else {
                collecting = false;
                used = 0U;
            }
        }
    }
    return SERIAL_CMD_NONE;
}
