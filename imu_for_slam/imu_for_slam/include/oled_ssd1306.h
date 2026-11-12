#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H
#include "board.h"
#define OLED_WIDTH 128U
#define OLED_PAGES 8U
typedef struct {
    soft_i2c_t *bus;
    uint8_t address;
    uint8_t pending_pages;
    uint8_t page_offset;
    uint8_t pixels[OLED_WIDTH*OLED_PAGES];
} oled_t;
bool OLED_Init(oled_t *oled, soft_i2c_t *bus);
bool OLED_BeginFrame(oled_t *oled);
void OLED_DrawLine(oled_t *oled, uint8_t row, const char *text);
void OLED_QueueRefresh(oled_t *oled);
/* Sends at most APP_OLED_CHUNK_BYTES pixels per call to bound GPIO bus latency. */
HAL_StatusTypeDef OLED_Service(oled_t *oled);
#endif
