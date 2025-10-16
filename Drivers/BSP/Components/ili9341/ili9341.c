/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ili9341.h"
int ILI9341_LCD_PIXEL_WIDTH_param = ILI9341_LCD_PIXEL_WIDTH;
int ILI9341_LCD_PIXEL_HEIGHT_param = ILI9341_LCD_PIXEL_HEIGHT;
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ILI9341
  * @brief This file provides a set of functions needed to drive the 
  *        ILI9341 LCD.
  * @{
  */

/** @defgroup ILI9341_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup ILI9341_Private_Defines
  * @{
  */
/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Macros
  * @{
  */
/**
  * @}
  */  

/** @defgroup ILI9341_Private_Variables
  * @{
  */ 

LCD_DrvTypeDef   ili9341_drv = 
{
  ili9341_Init,
  ili9341_ReadID,
  ili9341_DisplayOn,
  ili9341_DisplayOff,
  0,
  0,
  0,
  0,
  0,
  0,
  ili9341_GetLcdPixelWidth,
  ili9341_GetLcdPixelHeight,
  0,
  0,
  ILI9341_SetRotation,    
};

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Functions
  * @{
  */   

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9341_Init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  LCD_IO_Init();
  
  /* Configure LCD */
  ili9341_WriteReg(0xCA);
  ili9341_WriteData(0xC3);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x50);
  ili9341_WriteReg(LCD_POWERB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x30);
  ili9341_WriteReg(LCD_POWER_SEQ);
  ili9341_WriteData(0x64);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x12);
  ili9341_WriteData(0x81);
  ili9341_WriteReg(LCD_DTCA);
  ili9341_WriteData(0x85);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x78);
  ili9341_WriteReg(LCD_POWERA);
  ili9341_WriteData(0x39);
  ili9341_WriteData(0x2C);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x34);
  ili9341_WriteData(0x02);
  ili9341_WriteReg(LCD_PRC);
  ili9341_WriteData(0x20);
  ili9341_WriteReg(LCD_DTCB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_FRMCTR1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x1B);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA2);
  ili9341_WriteReg(LCD_POWER1);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_POWER2);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_VCOM1);
  ili9341_WriteData(0x45);
  ili9341_WriteData(0x15);
  ili9341_WriteReg(LCD_VCOM2);
  ili9341_WriteData(0x90);
  ili9341_WriteReg(LCD_MAC);
  ili9341_WriteData(0xC8);
  ili9341_WriteReg(LCD_3GAMMA_EN);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_RGB_INTERFACE);
  ili9341_WriteData(0xC2);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA7);
  ili9341_WriteData(0x27);
  ili9341_WriteData(0x04);
  
  /* Colomn address set */
  ili9341_WriteReg(LCD_COLUMN_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xEF);
  /* Page address set */
  ili9341_WriteReg(LCD_PAGE_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x3F);
  ili9341_WriteReg(LCD_INTERFACE);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x06);
  
  ili9341_WriteReg(LCD_GRAM);
  LCD_Delay(200);
  
  ili9341_WriteReg(LCD_GAMMA);
  ili9341_WriteData(0x01);
  
  ili9341_WriteReg(LCD_PGAMMA);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x29);
  ili9341_WriteData(0x24);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x4E);
  ili9341_WriteData(0x78);
  ili9341_WriteData(0x3C);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x13);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x17);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_NGAMMA);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x16);
  ili9341_WriteData(0x1B);
  ili9341_WriteData(0x04);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x33);
  ili9341_WriteData(0x42);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0x28);
  ili9341_WriteData(0x2F);
  ili9341_WriteData(0x0F);
  
  ili9341_WriteReg(LCD_SLEEP_OUT);
  LCD_Delay(200);
  ili9341_WriteReg(LCD_DISPLAY_ON);
  /* GRAM start writing */
  ili9341_WriteReg(LCD_GRAM);
}

void ili9341_Init_direct(void)
{
  /* Initialize ILI9341 low level bus layer */
  LCD_IO_Init();
  
  LCD_Delay(120);
  
  /* Software Reset */
  ili9341_WriteReg(0x01); // SWRESET
  LCD_Delay(120);
  
  /* Power Control A */
  ili9341_WriteReg(0xCB);
  ili9341_WriteData(0x39);
  ili9341_WriteData(0x2C);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x34);
  ili9341_WriteData(0x02);
  
  /* Power Control B */
  ili9341_WriteReg(0xCF);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x30);
  
  /* Driver timing control A */
  ili9341_WriteReg(0xE8);
  ili9341_WriteData(0x85);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x78);
  
  /* Driver timing control B */
  ili9341_WriteReg(0xEA);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  
  /* Power on sequence control */
  ili9341_WriteReg(0xED);
  ili9341_WriteData(0x64);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x12);
  ili9341_WriteData(0x81);
  
  /* Pump ratio control */
  ili9341_WriteReg(0xF7);
  ili9341_WriteData(0x20);
  
  /* Power Control 1 */
  ili9341_WriteReg(0xC0);
  ili9341_WriteData(0x23); // VRH[5:0]
  
  /* Power Control 2 */
  ili9341_WriteReg(0xC1);
  ili9341_WriteData(0x10); // SAP[2:0];BT[3:0]
  
  /* VCOM Control 1 */
  ili9341_WriteReg(0xC5);
  ili9341_WriteData(0x3E); // Contrast
  ili9341_WriteData(0x28);
  
  /* VCOM Control 2 */
  ili9341_WriteReg(0xC7);
  ili9341_WriteData(0x86); // --
  
  /* Memory Access Control - КРИТИЧЕСКИ ВАЖНО! */
  ili9341_WriteReg(0x36); // MADCTL
  ili9341_WriteData(0x48); // Portrait mode
  
  /* Pixel Format Set */
  ili9341_WriteReg(0x3A); // COLMOD
  ili9341_WriteData(0x55); // 16 bits/pixel
  
  /* Frame Rate Control */
  ili9341_WriteReg(0xB1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x18);
  
  /* Display Function Control */
  ili9341_WriteReg(0xB6);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x82);
  ili9341_WriteData(0x27);
  
  /* 3Gamma Function Disable */
  ili9341_WriteReg(0xF2);
  ili9341_WriteData(0x00);
  
  /* Gamma curve selected */
  ili9341_WriteReg(0x26);
  ili9341_WriteData(0x01);
  
  /* Positive Gamma Correction */
  ili9341_WriteReg(0xE0);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x2B);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x4E);
  ili9341_WriteData(0xF1);
  ili9341_WriteData(0x37);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x10);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x00);
  
  /* Negative Gamma Correction */
  ili9341_WriteReg(0xE1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x14);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x48);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x36);
  ili9341_WriteData(0x0F);
  
  /* Sleep Out */
  ili9341_WriteReg(0x11);
  LCD_Delay(120);
  
  /* Display ON */
  ili9341_WriteReg(0x29);
  LCD_Delay(50);
  
  /* Set orientation to default */
  ILI9341_SetRotation(0);
}
/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9341_ReadID(void)
{
  LCD_IO_Init();
  return ((uint16_t)ili9341_ReadData(LCD_READ_ID4, LCD_READ_ID4_SIZE));
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  /* Display On */
  ili9341_WriteReg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  ili9341_WriteReg(LCD_DISPLAY_OFF);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_WriteReg(uint8_t LCD_Reg)
{
  LCD_IO_WriteReg(LCD_Reg);
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_WriteData(uint16_t RegValue)
{
  LCD_IO_WriteData(RegValue);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
uint32_t ili9341_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  /* Read a max of 4 bytes */
  return (LCD_IO_ReadData(RegValue, ReadSize));
}

/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_LCD_PIXEL_WIDTH_param;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_LCD_PIXEL_HEIGHT_param;
}

void ILI9341_SetRotation(int rotation)
{
  ili9341_WriteReg(0x36); // MADCTL
    
    switch(rotation) {
        case 0: // Portrait 0° (240x320)
            ili9341_WriteData(0x48 | 0x08); // MY=1, BGR=1
            break;
        case 1: // Landscape 90° (320x240)
            ili9341_WriteData(0x28 | 0x08); // MV=1, MX=1, BGR=1
            break;
        case 2: // Portrait 180° (240x320)
            ili9341_WriteData(0x88 | 0x08); // MX=1, MY=1, BGR=1
            break;
        case 3: // Landscape 270° (320x240)
            ili9341_WriteData(0xE8 | 0x08); // MV=1, MY=1, BGR=1
            break;
    }
    
    // КРИТИЧЕСКИ ВАЖНО: устанавливаем правильное окно после поворота
    if(rotation == 1 || rotation == 3) {
        // Для 90° и 270°: 320x240
        ili9341_SetWindow(0, 0, 319, 239);
    } else {
        // Для 0° и 180°: 240x320  
        ili9341_SetWindow(0, 0, 239, 319);
    }
}


void ili9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  /* Column address set */
  ili9341_WriteReg(LCD_COLUMN_ADDR);
  ili9341_WriteData(x0 >> 8);
  ili9341_WriteData(x0 & 0xFF);
  ili9341_WriteData(x1 >> 8);
  ili9341_WriteData(x1 & 0xFF);
  
  /* Page address set */
  ili9341_WriteReg(LCD_PAGE_ADDR);
  ili9341_WriteData(y0 >> 8);
  ili9341_WriteData(y0 & 0xFF);
  ili9341_WriteData(y1 >> 8);
  ili9341_WriteData(y1 & 0xFF);

  // Memory write
  ili9341_WriteReg(0x2C);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    ili9341_SetWindow(x, y, x, y);
    ili9341_WriteData(color >> 8);
    ili9341_WriteData(color & 0xFF);
}

void ILI9341_FillScreen(uint16_t color) {
    //ili9341_SetWindow(0, 0, 319,239);
    
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    
    for(uint32_t i = 0; i < 240 * 320; i++) {
        ili9341_WriteData(hi);
        ili9341_WriteData(lo);
    }
}
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */
