#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "sdram.h"
#include "lcd.h"
#include <string.h>
#include "ili9341.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "../Component/ssd1306.h"
#include "../Component/ssd1306.c"
#include <math.h>

extern osMessageQueueId_t adc_queueHandle;

void init_lcd(void){
    return;
}
void print_lcd(int x, int y, const char *format, ...){
    char buff[50] = {0};

    va_list args;
    va_start(args, format);
    vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);

    OLED_WriteString_light(0, &oled, y, x, buff);
}

void upd_lcd(void){
    Convert_to_565Colors(&oled,(void*)LCD_FRAME_ADDRESS_SDRAM);
}

void lcd_task_func(void *argument){
    #ifdef LCD_SPI
    ili9341_Init_direct();
    int rotation = 4;
    while(1){
        for (rotation = 0; rotation < 4; rotation++){
        ILI9341_SetRotation(rotation);
        uint16_t colors[] = {0x0000,0xF800, 0x07E0, 0x001F, 0xFFFF}; // Black, R,G,B,W
        ILI9341_FillScreen(colors[rotation]);
        HAL_Delay(2000);
        }
    } 
    #endif
    #ifndef LCD_TRAINING
    ili9341_Init();
    #endif
    #ifdef LCD_BSP_EXAMPLE
        /* Initialize the LCD */
    BSP_LCD_Init();
    /* Initialize the LCD Layers */
    BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);
        /* Set LCD Foreground Layer  */
    BSP_LCD_SelectLayer(1);
    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
    /* Clear the LCD */ 
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    /* Set the LCD Text Color */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);  
    /* Display LCD messages */
    BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"STM32F429I BSP", CENTER_MODE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"Drivers examples", CENTER_MODE);
    #endif
    
    memset((void*)LCD_FRAME_ADDRESS_SDRAM, 255, LCD_BUFFER_SIZE);
    signed char color = 0;
    float result = 0.0F;
    while(1){
        color++;
        osDelay(10);
        print_lcd(0, 0, "Hello: %4d", color);
        xQueueReceive(adc_queueHandle, &result, 0);
        print_lcd(0, 1, "stm vdda: %.2f", result);
        print_lcd(0, 2, "0123456789012345678901234567890123456789012a", result);
        //print_lcd(0, 2, "0123456789012345678901234567890ab", result);
        print_lcd(1, 3, " ", result);
        print_lcd(0, 4, "%.25f", acos(-1.0));
        upd_lcd();
        //memset((void*)LCD_FRAME_ADDRESS_SDRAM, color, LCD_BUFFER_SIZE);
    }
}
