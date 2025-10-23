#ifndef LCD_H
#define LCD_H

#include "sdram.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_FRAME_ADDRESS_SDRAM      SDRAM_BASE_ADDR
#define LCD_FRAME_ADDRESS_STATIC_RAM 0x2000A800

#define LCD_COLOR565 16
#define LCD_COLOR888 24
#define LCD_BUFFER_SIZE (320 * 240 * LCD_COLOR565 / 8)
/*
размер lcd buffera в байтах:
320 * 240 * 16bit(RGB565) / 8bit = 153600 bytes или 150Kbytes
320 * 240 * 24bit(RGB888) / 8bit = 230400 bytes или 225Kbytes
*/

#ifdef __cplusplus
}
#endif
#endif // !1