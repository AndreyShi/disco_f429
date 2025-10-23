#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef USER_H
#define USER_H

#include "sdram.h"

extern UART_HandleTypeDef huart1;
#define DEBUG_UART_OUT   &huart1

#define printf print_user

#ifdef __cplusplus
extern "C" {
#endif

void stack_check(void);
float get_stm_VDDA(ADC_HandleTypeDef *hadc);
void DWT_Init(void);
void DWT_Delay(float seconds);
void print_user(const char *format, ...);


#ifdef __cplusplus
}
#endif

#endif


