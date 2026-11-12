#include "oled_ssd1306.h"
#include <string.h>

#if APP_ENABLE_OLED /* OLED驱动保留源码，默认不参与编译。 */
/* Original 5x7 row bitmap glyphs; no external font/library dependency. */
typedef struct { char code; uint8_t rows[7]; } glyph_t;
static const glyph_t font[] = {
    {' ', {0,0,0,0,0,0,0}},
    {'0', {14,17,19,21,25,17,14}}, {'1', {4,12,4,4,4,4,14}},
    {'2', {14,17,1,2,4,8,31}}, {'3', {30,1,1,14,1,1,30}},
    {'4', {2,6,10,18,31,2,2}}, {'5', {31,16,16,30,1,1,30}},
    {'6', {14,16,16,30,17,17,14}}, {'7', {31,1,2,4,8,8,8}},
    {'8', {14,17,17,14,17,17,14}}, {'9', {14,17,17,15,1,1,14}},
    {'A', {14,17,17,31,17,17,17}}, {'B', {30,17,17,30,17,17,30}},
    {'C', {14,17,16,16,16,17,14}}, {'D', {30,17,17,17,17,17,30}},
    {'E', {31,16,16,30,16,16,31}}, {'F', {31,16,16,30,16,16,16}},
    {'G', {14,17,16,23,17,17,14}}, {'H', {17,17,17,31,17,17,17}},
    {'I', {14,4,4,4,4,4,14}}, {'J', {7,2,2,2,2,18,12}},
    {'K', {17,18,20,24,20,18,17}}, {'L', {16,16,16,16,16,16,31}},
    {'M', {17,27,21,21,17,17,17}}, {'N', {17,25,21,19,17,17,17}},
    {'O', {14,17,17,17,17,17,14}}, {'P', {30,17,17,30,16,16,16}},
    {'Q', {14,17,17,17,21,18,13}}, {'R', {30,17,17,30,20,18,17}},
    {'S', {15,16,16,14,1,1,30}}, {'T', {31,4,4,4,4,4,4}},
    {'U', {17,17,17,17,17,17,14}}, {'V', {17,17,17,17,17,10,4}},
    {'W', {17,17,17,21,21,21,10}}, {'X', {17,17,10,4,10,17,17}},
    {'Y', {17,17,10,4,4,4,4}}, {'Z', {31,1,2,4,8,16,31}},
    {'a', {0,0,14,1,15,17,15}}, {'d', {1,1,15,17,17,17,15}},
    {'m', {0,0,26,21,21,21,21}}, {'r', {0,0,22,25,16,16,16}},
    {'s', {0,0,15,16,14,1,30}}, {'x', {0,0,17,10,4,10,17}},
    {'z', {0,0,31,2,4,8,31}},
    {'.', {0,0,0,0,0,12,12}}, {'-', {0,0,0,31,0,0,0}},
    {'+', {0,4,4,31,4,4,0}}, {'/', {1,1,2,4,8,16,16}},
    {':', {0,12,12,0,12,12,0}}, {'?', {14,17,1,2,4,0,4}},
    {'=', {0,0,31,0,31,0,0}},
};
static const uint8_t *glyph(char c)
{
    for (unsigned i = 0; i < sizeof(font)/sizeof(font[0]); ++i) {
        if (font[i].code == c) return font[i].rows;
    }
    return font[sizeof(font)/sizeof(font[0])-2U].rows; /* '?' */
}
bool OLED_Init(oled_t *oled, soft_i2c_t *bus)
{
    oled->bus = bus;
    oled->pending_pages = 0U;
    oled->page_offset = 0U;
    memset(oled->pixels, 0, sizeof(oled->pixels));
    bool found = false;
    for (uint8_t address = 0x3CU; address <= 0x3DU; ++address) {
        oled->address = address;
        if (SoftI2C_Probe(bus, address) == HAL_OK) {
            found = true;
            break;
        }
    }
    if (!found) return false;
    uint8_t commands[] = {
        0x00, /* I2C command control byte. */
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, /* Charge pump, page addressing. */
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F,
        0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF
    };
    return SoftI2C_Write(bus, oled->address, commands, sizeof(commands)) == HAL_OK;
}
bool OLED_BeginFrame(oled_t *oled)
{
    if (oled->pending_pages != 0U) return false;
    memset(oled->pixels, 0, sizeof(oled->pixels));
    return true;
}
void OLED_DrawLine(oled_t *oled, uint8_t row, const char *text)
{
    if (row >= OLED_PAGES) return;
    uint8_t *page = oled->pixels + OLED_WIDTH*row;
    memset(page, 0, OLED_WIDTH);
    for (unsigned c = 0; text[c] != '\0' && c < OLED_WIDTH/6U; ++c) {
        const uint8_t *rows = glyph(text[c]);
        for (unsigned x = 0; x < 5U; ++x) {
            uint8_t bits = 0U;
            for (unsigned y = 0; y < 7U; ++y) {
                if ((rows[y] & (1U << (4U-x))) != 0U) bits |= (uint8_t)(1U << y);
            }
            page[6U*c+x] = bits;
        }
    }
}
void OLED_QueueRefresh(oled_t *oled)
{
    oled->page_offset = 0U;
    oled->pending_pages = 0xFFU;
}
HAL_StatusTypeDef OLED_Service(oled_t *oled)
{
    if (oled->pending_pages == 0U) return HAL_OK;
    uint8_t page = 0U;
    while ((oled->pending_pages & (1U << page)) == 0U) ++page;
    uint8_t command[] = {0x00U, (uint8_t)(0xB0U+page),
                         (uint8_t)(oled->page_offset & 0x0FU),
                         (uint8_t)(0x10U | (oled->page_offset >> 4U))};
    HAL_StatusTypeDef result = SoftI2C_Write(oled->bus, oled->address, command, sizeof(command));
    if (result != HAL_OK) return result;
    uint16_t count = OLED_WIDTH-oled->page_offset;
    if (count > APP_OLED_CHUNK_BYTES) count = APP_OLED_CHUNK_BYTES;
    uint8_t data[APP_OLED_CHUNK_BYTES+1U];
    data[0] = 0x40U; /* I2C display-data control byte. */
    memcpy(data+1U, oled->pixels+OLED_WIDTH*page+oled->page_offset, count);
    result = SoftI2C_Write(oled->bus, oled->address, data, count+1U);
    if (result == HAL_OK) {
        uint16_t next = (uint16_t)oled->page_offset+count;
        if (next == OLED_WIDTH) {
            oled->page_offset = 0U;
            oled->pending_pages &= (uint8_t)~(1U << page);
        } else {
            oled->page_offset = (uint8_t)next;
        }
    }
    return result;
}
#endif /* APP_ENABLE_OLED */
