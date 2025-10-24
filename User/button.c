#include "button.h"
#include "main.h"
#include "sdram.h"
#include "cmsis_os2.h"

int counter = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == INT_button_Pin){
        *(uint16_t*)(LCD_FRAME_ADDRESS_SDRAM + counter) = 0;
        counter+=2;
    }
}

void button_task_func(void *argument){

    while(1){
        osDelay(100);
    }
}
