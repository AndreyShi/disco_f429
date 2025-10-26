#include "ssd1306.h"
#include <stdio.h>
#include "user.h"

OLED_HandleTypeDef oled;

// Функция отправки команды
void OLED_WriteCommand(OLED_HandleTypeDef *oled, uint8_t command) {
    uint8_t data[2] = {OLED_COMMAND, command};
   HAL_StatusTypeDef status =  HAL_I2C_Master_Transmit(oled->hi2c, OLED_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
   //printf("HAL_StatusTypeDef: %d\n",status);
}

// Функция отправки данных
void OLED_WriteData(OLED_HandleTypeDef *oled, uint8_t data) {
    uint8_t tx_data[2] = {OLED_DATA, data};
    HAL_I2C_Master_Transmit(oled->hi2c, OLED_ADDRESS << 1, tx_data, 2, HAL_MAX_DELAY);
}

// Инициализация дисплея
void OLED_Init(OLED_HandleTypeDef *oled, I2C_HandleTypeDef *hi2c) {
    oled->hi2c = hi2c;
    oled->currentX = 0;
    oled->currentY = 0;
    
    HAL_Delay(100);
    
    // Последовательность команд инициализации
    OLED_WriteCommand(oled, 0xAE); // Display OFF
    OLED_WriteCommand(oled, 0x20); // Memory addressing mode
    OLED_WriteCommand(oled, 0x00); // Horizontal addressing mode
    OLED_WriteCommand(oled, 0xB0); // Set page start address
    OLED_WriteCommand(oled, 0xC8); // Set COM output scan direction
    OLED_WriteCommand(oled, 0x00); // Set low column address
    OLED_WriteCommand(oled, 0x10); // Set high column address
    OLED_WriteCommand(oled, 0x40); // Set start line address
    OLED_WriteCommand(oled, 0x81); // Set contrast control
    OLED_WriteCommand(oled, 0x7F); // Contrast value
    OLED_WriteCommand(oled, 0xA1); // Set segment re-map
    OLED_WriteCommand(oled, 0xA6); // Set normal display
    OLED_WriteCommand(oled, 0xA8); // Set multiplex ratio
    OLED_WriteCommand(oled, 0x1F); // 1/32 duty
    OLED_WriteCommand(oled, 0xD3); // Set display offset
    OLED_WriteCommand(oled, 0x00); // No offset
    OLED_WriteCommand(oled, 0xD5); // Set display clock divide ratio
    OLED_WriteCommand(oled, 0xF0); // Set divide ratio
    OLED_WriteCommand(oled, 0xD9); // Set pre-charge period
    OLED_WriteCommand(oled, 0x22); // 
    OLED_WriteCommand(oled, 0xDA); // Set COM pins hardware configuration
    OLED_WriteCommand(oled, 0x02); // 
    OLED_WriteCommand(oled, 0xDB); // Set VCOMH deselect level
    OLED_WriteCommand(oled, 0x20); // 
    OLED_WriteCommand(oled, 0x8D); // Set DC-DC enable
    OLED_WriteCommand(oled, 0x14); // 
    OLED_WriteCommand(oled, 0xAF); // Display ON
    
    // Очистка дисплея
    OLED_Clear(oled);
}

// Очистка дисплея
void OLED_Clear(OLED_HandleTypeDef *oled) {
    memset(oled->buffer, 0, sizeof(oled->buffer));
    OLED_UpdateScreen(oled);
}

// Обновление экрана
int u = 0;
void OLED_UpdateScreen(OLED_HandleTypeDef *oled) {
    for (uint8_t i = 0; i < 4; i++) {
        OLED_WriteCommand(oled, 0xB0 + i); // Set page address
        OLED_WriteCommand(oled, 0x00);     // Set lower column address
        OLED_WriteCommand(oled, 0x10);     // Set higher column address
        
        for (uint8_t j = 0; j < OLED_WIDTH; j++) {
            if (u == 0)
                {OLED_WriteData(oled, ~oled->buffer[j + (i * OLED_WIDTH)]);}
            else
                {OLED_WriteData(oled, oled->buffer[j + (i * OLED_WIDTH)]);}

            //DWT_Delay(0.005);
        }
    }

    if( u == 0)
        {u = 1;}
    else
        {u = 1;}
}

// Установка позиции курсора
void OLED_SetCursor(OLED_HandleTypeDef *oled, uint8_t x, uint8_t y) {
    oled->currentX = x;
    oled->currentY = y;
}

// Очистка области под символ
void OLED_ClearCharArea(OLED_HandleTypeDef *oled, uint8_t x, uint8_t y) {
    // Очищаем область 7x8 пикселей (5x7 символ + отступы)
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t col = x + i;
        if (col >= OLED_WIDTH) continue;
        
        for (uint8_t row = 0; row < 8; row++) {
            uint8_t page = (y + row) / 8;
            uint8_t bit_pos = (y + row) % 8;
            uint16_t buffer_index = col + (page * OLED_WIDTH);
            
            if (buffer_index < sizeof(oled->buffer)) {
                oled->buffer[buffer_index] &= ~(1 << bit_pos); // Очищаем бит
            }
        }
    }
}

// Простой вывод символа (базовый шрифт 5x7)
void OLED_WriteChar(OLED_HandleTypeDef *oled, char ch) {
    if (ch < 32 || ch > 127) return;

        // Очищаем область перед выводом нового символа
    OLED_ClearCharArea(oled, oled->currentX, oled->currentY);
    
    // Простой шрифт 5x7 (каждый символ занимает 5 байт)
    static const uint8_t font5x7[] = {
	0x00, 0x00, 0x00, 0x00, 0x00,// (space)
	0x00, 0x00, 0x5F, 0x00, 0x00,// !
	0x00, 0x07, 0x00, 0x07, 0x00,// "
	0x14, 0x7F, 0x14, 0x7F, 0x14,// #
	0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
	0x23, 0x13, 0x08, 0x64, 0x62,// %
	0x36, 0x49, 0x55, 0x22, 0x50,// &
	0x00, 0x05, 0x03, 0x00, 0x00,// '
	0x00, 0x1C, 0x22, 0x41, 0x00,// (
	0x00, 0x41, 0x22, 0x1C, 0x00,// )
	0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
	0x08, 0x08, 0x3E, 0x08, 0x08,// +
	0x00, 0x50, 0x30, 0x00, 0x00,// ,
	0x08, 0x08, 0x08, 0x08, 0x08,// -
	0x00, 0x60, 0x60, 0x00, 0x00,// .
	0x20, 0x10, 0x08, 0x04, 0x02,// /
	0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
	0x00, 0x42, 0x7F, 0x40, 0x00,// 1
	0x42, 0x61, 0x51, 0x49, 0x46,// 2
	0x21, 0x41, 0x45, 0x4B, 0x31,// 3
	0x18, 0x14, 0x12, 0x7F, 0x10,// 4
	0x27, 0x45, 0x45, 0x45, 0x39,// 5
	0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
	0x01, 0x71, 0x09, 0x05, 0x03,// 7
	0x36, 0x49, 0x49, 0x49, 0x36,// 8
	0x06, 0x49, 0x49, 0x29, 0x1E,// 9
	0x00, 0x36, 0x36, 0x00, 0x00,// :
	0x00, 0x56, 0x36, 0x00, 0x00,// ;
	0x00, 0x08, 0x14, 0x22, 0x41,// <
	0x14, 0x14, 0x14, 0x14, 0x14,// =
	0x41, 0x22, 0x14, 0x08, 0x00,// >
	0x02, 0x01, 0x51, 0x09, 0x06,// ?
	0x32, 0x49, 0x79, 0x41, 0x3E,// @
	0x7E, 0x11, 0x11, 0x11, 0x7E,// A
	0x7F, 0x49, 0x49, 0x49, 0x36,// B
	0x3E, 0x41, 0x41, 0x41, 0x22,// C
	0x7F, 0x41, 0x41, 0x22, 0x1C,// D
	0x7F, 0x49, 0x49, 0x49, 0x41,// E
	0x7F, 0x09, 0x09, 0x01, 0x01,// F
	0x3E, 0x41, 0x41, 0x51, 0x32,// G
	0x7F, 0x08, 0x08, 0x08, 0x7F,// H
	0x00, 0x41, 0x7F, 0x41, 0x00,// I
	0x20, 0x40, 0x41, 0x3F, 0x01,// J
	0x7F, 0x08, 0x14, 0x22, 0x41,// K
	0x7F, 0x40, 0x40, 0x40, 0x40,// L
	0x7F, 0x02, 0x04, 0x02, 0x7F,// M
	0x7F, 0x04, 0x08, 0x10, 0x7F,// N
	0x3E, 0x41, 0x41, 0x41, 0x3E,// O
	0x7F, 0x09, 0x09, 0x09, 0x06,// P
	0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
	0x7F, 0x09, 0x19, 0x29, 0x46,// R
	0x46, 0x49, 0x49, 0x49, 0x31,// S
	0x01, 0x01, 0x7F, 0x01, 0x01,// T
	0x3F, 0x40, 0x40, 0x40, 0x3F,// U
	0x1F, 0x20, 0x40, 0x20, 0x1F,// V
	0x7F, 0x20, 0x18, 0x20, 0x7F,// W
	0x63, 0x14, 0x08, 0x14, 0x63,// X
	0x03, 0x04, 0x78, 0x04, 0x03,// Y
	0x61, 0x51, 0x49, 0x45, 0x43,// Z
	0x00, 0x00, 0x7F, 0x41, 0x41,// [
	0x02, 0x04, 0x08, 0x10, 0x20,// "\"
	0x41, 0x41, 0x7F, 0x00, 0x00,// ]
	0x04, 0x02, 0x01, 0x02, 0x04,// ^
	0x40, 0x40, 0x40, 0x40, 0x40,// _
	0x00, 0x01, 0x02, 0x04, 0x00,// `
	0x20, 0x54, 0x54, 0x54, 0x78,// a
	0x7F, 0x48, 0x44, 0x44, 0x38,// b
	0x38, 0x44, 0x44, 0x44, 0x20,// c
	0x38, 0x44, 0x44, 0x48, 0x7F,// d
	0x38, 0x54, 0x54, 0x54, 0x18,// e
	0x08, 0x7E, 0x09, 0x01, 0x02,// f
	0x08, 0x14, 0x54, 0x54, 0x3C,// g
	0x7F, 0x08, 0x04, 0x04, 0x78,// h
	0x00, 0x44, 0x7D, 0x40, 0x00,// i
	0x20, 0x40, 0x44, 0x3D, 0x00,// j
	0x00, 0x7F, 0x10, 0x28, 0x44,// k
	0x00, 0x41, 0x7F, 0x40, 0x00,// l
	0x7C, 0x04, 0x18, 0x04, 0x78,// m
	0x7C, 0x08, 0x04, 0x04, 0x78,// n
	0x38, 0x44, 0x44, 0x44, 0x38,// o
	0x7C, 0x14, 0x14, 0x14, 0x08,// p
	0x08, 0x14, 0x14, 0x18, 0x7C,// q
	0x7C, 0x08, 0x04, 0x04, 0x08,// r
	0x48, 0x54, 0x54, 0x54, 0x20,// s
	0x04, 0x3F, 0x44, 0x40, 0x20,// t
	0x3C, 0x40, 0x40, 0x20, 0x7C,// u
	0x1C, 0x20, 0x40, 0x20, 0x1C,// v
	0x3C, 0x40, 0x30, 0x40, 0x3C,// w
	0x44, 0x28, 0x10, 0x28, 0x44,// x
	0x0C, 0x50, 0x50, 0x50, 0x3C,// y
	0x44, 0x64, 0x54, 0x4C, 0x44,// z
	0x00, 0x08, 0x36, 0x41, 0x00,// {
	0x00, 0x00, 0x7F, 0x00, 0x00,// |
	0x00, 0x41, 0x36, 0x08, 0x00,// }
	0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
	0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
    };
    
    if (ch < 32 || ch > 127) ch = ' '; // Заменяем непечатные символы на пробелы
    
    uint8_t charIndex = ch - 32;
    const uint8_t *charData = &font5x7[charIndex * 5];
    
    // Рисуем символ в буфере
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t x = oled->currentX + i;
        if (x >= OLED_WIDTH) continue;
        
        uint8_t y = oled->currentY;
        if (y >= OLED_HEIGHT) continue;
        
        // Рисуем каждый бит символа
        uint8_t fontByte = charData[i];
        for (uint8_t bit = 0; bit < 7; bit++) {
            if (fontByte & (1 << bit)) {
                uint16_t pos = x + ((y + bit) / 8) * OLED_WIDTH;
                oled->buffer[pos] |= (1 << ((y + bit) % 8));
            }
        }
    }
    
    // Перемещаем курсор
    oled->currentX += 7 + 1;
    
    // Перенос на следующую строку
    if (oled->currentX > OLED_WIDTH - 7) {
        oled->currentX = 0;
        oled->currentY += 8;
        if (oled->currentY >= OLED_HEIGHT) {
            oled->currentY = 0;
        }
    }
}

// Вывод строки
void OLED_WriteString(uint8_t update_src, OLED_HandleTypeDef *oled, uint8_t row, uint8_t col, char *str, ...) {

    OLED_SetTextCursor(oled, row, col);

    char buff[50] = {0};
    va_list args;
    va_start(args, str);
    vsnprintf(buff, sizeof(buff), str, args);
    va_end(args);
    int i = 0;
    while (buff[i]) {
        OLED_WriteChar(oled, buff[i++]);
    }
    if(update_src == 1){
        OLED_UpdateScreen(oled);
    }
}

// Вывод строки
void OLED_WriteString_light(uint8_t update_src, OLED_HandleTypeDef *oled, uint8_t row, uint8_t col, char *str) {

    OLED_SetTextCursor(oled, row, col);
    int i = 0;
    while (str[i]) {
        OLED_WriteChar(oled, str[i++]);
    }
}

// Перевернуть экран по горизонтали
void OLED_FlipHorizontal(OLED_HandleTypeDef *oled, uint8_t flip) {
    if (flip) {
        OLED_WriteCommand(oled, 0xA0); // Seg re-map: column address 0 mapped to SEG0
    } else {
        OLED_WriteCommand(oled, 0xA1); // Seg re-map: column address 127 mapped to SEG0
    }
}

// Перевернуть экран по вертикали
void OLED_FlipVertical(OLED_HandleTypeDef *oled, uint8_t flip) {
    if (flip) {
        OLED_WriteCommand(oled, 0xC0); // COM output scan direction: normal mode
    } else {
        OLED_WriteCommand(oled, 0xC8); // COM output scan direction: remapped mode
    }
}

// Установка позиции текста (строка 0-3, столбец 0-25)
void OLED_SetTextCursor(OLED_HandleTypeDef *oled, uint8_t row, uint8_t col) {
    oled->currentX = col * (FONT_WIDTH + 1);
    oled->currentY = row * 8; // 8 пикселей на строку
}

// Инвертировать цвета дисплея
void OLED_InvertColors(OLED_HandleTypeDef *oled, uint8_t invert) {
    if (invert) {
        OLED_WriteCommand(oled, 0xA7); // Inverted display
    } else {
        OLED_WriteCommand(oled, 0xA6); // Normal display
    }
}