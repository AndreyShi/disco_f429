/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern SPI_HandleTypeDef hspi5;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define lcd_spi_sck_Pin GPIO_PIN_7
#define lcd_spi_sck_GPIO_Port GPIOF
#define lcd_spi_mosi_Pin GPIO_PIN_9
#define lcd_spi_mosi_GPIO_Port GPIOF
#define lcd_csx_Pin GPIO_PIN_2
#define lcd_csx_GPIO_Port GPIOC
#define button_Pin GPIO_PIN_0
#define button_GPIO_Port GPIOA
#define B5_Pin GPIO_PIN_3
#define B5_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_6
#define G2_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_0
#define R3_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_1
#define R6_GPIO_Port GPIOB
#define G4_Pin GPIO_PIN_10
#define G4_GPIO_Port GPIOB
#define G5_Pin GPIO_PIN_11
#define G5_GPIO_Port GPIOB
#define lcd_dcx_Pin GPIO_PIN_13
#define lcd_dcx_GPIO_Port GPIOD
#define rs485_DERE_Pin GPIO_PIN_2
#define rs485_DERE_GPIO_Port GPIOG
#define R7_Pin GPIO_PIN_6
#define R7_GPIO_Port GPIOG
#define G6_Pin GPIO_PIN_7
#define G6_GPIO_Port GPIOC
#define R4_Pin GPIO_PIN_11
#define R4_GPIO_Port GPIOA
#define R5_Pin GPIO_PIN_12
#define R5_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_10
#define R2_GPIO_Port GPIOC
#define rs485_DI_Pin GPIO_PIN_12
#define rs485_DI_GPIO_Port GPIOC
#define rs485_RO_Pin GPIO_PIN_2
#define rs485_RO_GPIO_Port GPIOD
#define G7_Pin GPIO_PIN_3
#define G7_GPIO_Port GPIOD
#define B2_Pin GPIO_PIN_6
#define B2_GPIO_Port GPIOD
#define G3_Pin GPIO_PIN_10
#define G3_GPIO_Port GPIOG
#define B3_Pin GPIO_PIN_11
#define B3_GPIO_Port GPIOG
#define B4_Pin GPIO_PIN_12
#define B4_GPIO_Port GPIOG
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOG
#define LD4_Pin GPIO_PIN_14
#define LD4_GPIO_Port GPIOG
#define B6_Pin GPIO_PIN_8
#define B6_GPIO_Port GPIOB
#define B7_Pin GPIO_PIN_9
#define B7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
