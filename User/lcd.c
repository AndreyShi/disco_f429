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

#include "../lvgl/lvgl.h"

#define TFT_HOR_RES 240
#define TFT_VER_RES 320
#define BYTES_PER_PIXEL 2
#define SCREEN_WIDTH TFT_HOR_RES

extern osMessageQueueId_t adc_queueHandle;

void init_lcd(void){
    #ifdef LCD_SPI
    //инициализация SPI в CubeMX
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
    //инициализация SPI в CubeMX
    //инициализация LTDC в CubeMX
    //инициализация external RAM в CubeMX
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
    Copy_oled_mem(&oled,(void*)LCD_FRAME_ADDRESS_SDRAM,0);
}

static uint32_t my_tick_cb(void)
{
    return HAL_GetTick();
}

static void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    int32_t x, y;
    uint8_t *fb_ptr = (uint8_t *)LCD_FRAME_ADDRESS_SDRAM; // Ваш фреймбуфер
    
    // Копируем пиксели построчно
    for(y = area->y1; y <= area->y2; y++) {
        uint8_t *line_src = px_map + ((y - area->y1) * (area->x2 - area->x1 + 1)) * BYTES_PER_PIXEL;
        uint8_t *line_dst = fb_ptr + (y * SCREEN_WIDTH + area->x1) * BYTES_PER_PIXEL;
        memcpy(line_dst, line_src, (area->x2 - area->x1 + 1) * BYTES_PER_PIXEL);
    }
    
    lv_display_flush_ready(disp);
}

static int my_touch_is_pressed(){
      return 0;
}

static void my_touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
   if(my_touch_is_pressed()) {
       data->point.x = 0;
       data->point.y = 0;
       data->state = LV_INDEV_STATE_PRESSED;
   } else {
       data->state = LV_INDEV_STATE_RELEASED;
   }
}



void lcd_task_func(void *argument){
    init_lcd();
    
    memset((void*)LCD_FRAME_ADDRESS_SDRAM, 255, LCD_BUFFER_SIZE);
    signed char color = 0;
    int delay_ad = 700;
    float result = 0.0F;
    /*Initialize LVGL*/
    lv_init();

    /*Set millisecond-based tick source for LVGL so that it can track time.*/
    lv_tick_set_cb(my_tick_cb);

    /*Create a display where screens and widgets can be added*/
    lv_display_t * display = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

    /*Add rendering buffers to the screen.
     *Here adding a smaller partial buffer assuming 16-bit (RGB565 color format)*/
    static uint8_t buf[TFT_HOR_RES * TFT_VER_RES / 10 * 2]; /* x2 because of 16-bit color depth */
    lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /*Add a callback that can flush the content from `buf` when it has been rendered*/
    lv_display_set_flush_cb(display, my_flush_cb);

    /*Create an input device for touch handling*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read_cb);

    // Создание кнопки
    lv_obj_t * btn = lv_btn_create(lv_screen_active());

    // Установка размера
    lv_obj_set_size(btn, 100, 50); // ширина, высота

    // Позиционирование
    lv_obj_set_pos(btn, 10, 10); // x, y
    // или центрирование
    //lv_obj_center(btn);

    // Добавление текста на кнопку
    lv_obj_t * label_bt = lv_label_create(btn);
    lv_label_set_text(label_bt, "Нажми!");
    lv_obj_center(label_bt);
        /*The drivers are in place; now we can create the UI*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    //lv_obj_center(label);
    lv_obj_set_pos(label, 10, 150); // x, y
    lv_obj_set_style_text_font(label, &lv_font_unscii_16, 0);
    
    while(1){
        lv_label_set_text_fmt(label, "Привет %5d",color++);
        printf("%d\n",result * 100);
        lv_timer_handler(); // Должен вызываться каждые ~5-30 мс
        osDelay(500);
        //print_lcd(0, 0, "Hello: %4d", color);
        xQueueReceive(adc_queueHandle, &result, 0);
        //print_lcd(0, 1, "stm vdda: %.2f", result);
        //print_lcd(0, 2, "0123456789012345678901234567890123456789012a", result);
        //print_lcd(1, 3, " ", result);
        //print_lcd(0, 4, "%.25f", acos(-1.0));
        //upd_lcd();
    }
}
