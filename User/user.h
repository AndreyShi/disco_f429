#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef USER_H
#define USER_H

#ifdef HAL_UART_MODULE_ENABLED
extern UART_HandleTypeDef huart1;
#define DEBUG_UART_OUT   &huart1
#endif

#define printf print_terminal

#ifdef __cplusplus
extern "C" {
#endif

void stack_check(void);
#ifdef HAL_ADC_MODULE_ENABLED
float get_stm_VDDA(ADC_HandleTypeDef *hadc);
#endif
void DWT_Init(void);
void DWT_Delay(float seconds);
void print_terminal(const char *format, ...);


#ifdef __cplusplus
}
#endif

#endif


