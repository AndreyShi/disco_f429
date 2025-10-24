#include "cmsis_os2.h"
#include "main.h"
#include "user.h"

extern ADC_HandleTypeDef hadc1;

float get_stm_VDDA(ADC_HandleTypeDef *hadc){

    int adc_data = 0;
    float res = -1.0;

    HAL_ADC_Start(hadc);
    if(HAL_ADC_PollForConversion(hadc, 1000) == HAL_OK){
        adc_data = (int)HAL_ADC_GetValue(hadc);

        //VREFINT = (adc_data × VREF_PLUS_CHARAC) / 4095.0f 
        //VDDA
        res = (*((uint16_t*)VREFINT_CAL_ADDR) * (float)VREFINT_CAL_VREF/1000) / adc_data;
        //printf("ADC value: %d adc vol: %.2f\n",adc_data, res );
    }
    return res;
}

void adc_task_func(void *argument){
    float f = 1.45;
    while(1){
        //print_terminal("Stm intref: %.2f\n",get_stm_VDDA(&hadc1));
        print_terminal("Stm intref: %.2f\n",f);
        osDelay(1500);
    }
}