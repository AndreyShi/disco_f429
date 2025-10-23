#include "user.h"

#define printf print_user

/*
heap увеличивает адрес ,а стек уменьшает адрес 
в STM heap и stack растут друк к другу навстречу
*/
void stack_check(void){
    char stack_data[10] = { 45,45,45,45,45,45,45,45,45,45};
    HAL_Delay(1);
    stack_check();
}

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

void print_user(const char *format, ...) {
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

#define SDRAM_BASE_ADDR   0xD0000000
#define TEST_SIZE         0x10  // 4KB для теста
/**
 * @brief чтение запись bank1 sdram адрес 0xD0000000
 * 
 * @return true - чтение запись прошла успешно
 * @return false - чтение запись прошла с ошибками
 */
bool test_sdram_basic(void) {
    volatile uint32_t* sdram = (uint32_t*)SDRAM_BASE_ADDR;
    
    printf("Testing SDRAM at 0x%08lX...\n", (uint32_t)sdram);
    
    // Тест записи
    printf("Writing test pattern...\n");
    for(uint32_t i = 0; i < TEST_SIZE / 4; i++) {
        sdram[i] = 0x12345678 + i;  // Простой паттерн
    }
    
    // Тест чтения
    printf("Verifying data...\n");
    for(uint32_t i = 0; i < TEST_SIZE / 4; i++) {
        if(sdram[i] != (0x12345678 + i)) {
            printf("SDRAM ERROR at 0x%08lX: wrote 0x%08lX, read 0x%08lX\n",
                   (uint32_t)&sdram[i], (0x12345678 + i), sdram[i]);
            return false;
        }
    }
    
    printf("SDRAM basic test PASSED!\n");
    return true;
}