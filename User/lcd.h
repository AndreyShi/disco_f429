#ifndef LCD_H
#define LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_FRAME_ADDRESS_STATIC_RAM 0x2000A800

#define LCD_COLOR1    1
#define LCD_COLOR565 16
#define LCD_COLOR888 24
#define LCD_BUFFER_SIZE (320 * 240 * LCD_COLOR565 / 8)

typedef struct {
    void*   buf;
    uint8_t w;   //ширина
    uint8_t h;   //высота
    uint8_t bit; //цвет дисплея, 1 bit, 16 bit
    uint8_t currentX;
    uint8_t currentY;
} lcd_buffer_t;

/*
размер lcd buffera в байтах:
128 *  32 *  1bit(monochrome) / 8bit = 512    bytes
128 *  64 *  1bit(monochrome) / 8bit = 1024   bytes(1Kbytes)
320 * 240 * 16bit(RGB565)     / 8bit = 153600 bytes(150Kbytes)
320 * 240 * 24bit(RGB888)     / 8bit = 230400 bytes(225Kbytes)
*/

void init_lcd(void);
void print_lcd(int x, int y, const char *format, ...);
void upd_lcd(void);

#ifdef __cplusplus
}
#endif
#endif // !1