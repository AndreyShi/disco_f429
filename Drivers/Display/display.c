#include "display.h"
#include "main.h"

#define cs_low HAL_GPIO_WritePin(lcd_csx_GPIO_Port, lcd_csx_Pin, GPIO_PIN_RESET);
#define cs_hi  HAL_GPIO_WritePin(lcd_csx_GPIO_Port, lcd_csx_Pin, GPIO_PIN_SET);

#define dcx_cmd HAL_GPIO_WritePin(lcd_dcx_GPIO_Port, lcd_dcx_Pin, GPIO_PIN_RESET);
#define dcx_data HAL_GPIO_WritePin(lcd_dcx_GPIO_Port, lcd_dcx_Pin, GPIO_PIN_SET);

#define display_spi &hspi5

uint32_t ReadDisplayStatus(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x09", 1, 2);
  uint32_t pData = 0; 
  uint32_t error = HAL_SPI_Receive(display_spi, (uint8_t*)&pData, 4, 2);
  cs_hi;
  return pData;
}

uint32_t ReadDisplayId(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x04", 1, 2);
  uint8_t pData[4] = {0}; 
  HAL_SPI_Receive(display_spi, pData, 4, 2);
  cs_hi;
  return 0;
}

uint8_t ReadDisplayPowerMode(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x0A", 1, 2);
  uint8_t pData = 0; 
  HAL_SPI_Receive(display_spi, &pData, 1, 2);
  cs_hi;
  return pData;
}

uint16_t ReadDisplayPixelFormat(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x0C", 1, 2);
  uint8_t pData[1] = {0}; 
  HAL_SPI_Receive(display_spi, pData, 1, 2);
  cs_hi;
  return 0; 
}

void ResetDisplay(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x01", 1, 2);
  cs_hi;
  HAL_Delay(5);
}

void DisplayInversionON(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x21", 1, 2);
  cs_hi;
}

void DisplayON(void)
{
  cs_low;
  dcx_cmd;
  HAL_SPI_Transmit(display_spi,(uint8_t*)"\x29", 1, 2);
  cs_hi;    
}