#include "user.h"
#ifdef HAL_PCD_MODULE_ENABLED
#include "usbd_cdc_if.h"
#endif

/*
heap увеличивает адрес ,а стек уменьшает адрес 
в STM heap и stack растут друк к другу навстречу
*/
void stack_check(void){
    char stack_data[10] = { 45,45,45,45,45,45,45,45,45,45};
    HAL_Delay(1);
    stack_check();
}
#ifdef HAL_ADC_MODULE_ENABLED
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
#endif
/*
Счетчик DWT 32-битный, переполняется каждые ~21 секунду при 200 МГц
Работает на частоте ядра процессора
Не требует прерываний
Очень высокая точность (1 такт процессора)
*/
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Задержка на основе DWT (работает без прерываний)
void DWT_Delay(float seconds)
{
    uint32_t start = DWT->CYCCNT;
    // Количество циклов на микросекунду (для 400MHz: 400 циклов/мкс)
    //#define MSCIN1SEC 1000000 //кол-во микросекунд в 1 секунде
    //#define MSIN1SEC  1000    //кол-во милисекунд в 1 секунде
    //uint32_t cycles = microseconds * (SystemCoreClock / MSCIN1SEC);
    float period = 1.0F / SystemCoreClock;
    uint32_t cycles = seconds / period;

    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

void print_terminal(const char *format, ...) {
  char buffer[128]; // Подберите размер под ваши нужды
  va_list args;
  
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  if (len > 0) {
    //CDC_Transmit_FS((uint8_t *)buffer, len);
    HAL_UART_Transmit(DEBUG_UART_OUT, (uint8_t *)buffer, len, 100);
  }
}