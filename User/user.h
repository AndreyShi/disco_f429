#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

//information from datasheet DS9405 Rev 12 for STM32F427xx STM32F429xx
//#define VREF_PLUS_CHARAC 3.3f
//#define VREFINT_CAL_ADDR 0x1FFF7A2A

extern UART_HandleTypeDef huart1;
#define DEBUG_UART_OUT   &huart1

#ifdef __cplusplus
extern "C" {
#endif

void stack_check(void);
float get_stm_VDDA(ADC_HandleTypeDef *hadc);
void DWT_Init(void);
void DWT_Delay(float seconds);
void print_user(const char *format, ...);
bool test_sdram_basic(void);

#ifdef __cplusplus
}
#endif





