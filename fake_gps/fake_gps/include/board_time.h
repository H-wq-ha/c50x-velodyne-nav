#ifndef BOARD_TIME_H
#define BOARD_TIME_H
#include <stdint.h>
uint32_t Board_Micros(void);
uint64_t Board_MonotonicUs(void);
void Board_DelayUs(uint32_t us);
#endif
