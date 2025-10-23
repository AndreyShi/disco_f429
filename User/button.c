#include "button.h"
#include "lcd.h"

int counter = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == INT_button_Pin){
        *(uint16_t*)(LCD_FRAME_ADDRESS_SDRAM + counter) = 0;
        counter+=2;
    }
}