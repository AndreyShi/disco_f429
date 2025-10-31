#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "mb.h"
#include "mbport.h"
#include "port_internal.h"
#include "mbutils.h"

#include "main.h"
extern UART_HandleTypeDef huart1;

/* ----------------Modbus register callback functions -----------------------*/
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs);
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode);
eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode);
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);

void modbus_task_func(void *argument){

    //HAL_UART_DeInit(&huart1);
    MB_Uart_Init();
    eMBInit(MB_RTU, 1, 0, 115200, MB_PAR_NONE, 1);
    eMBEnable();

    while(1){
        //TO DO сделать TAskNotification чтобы ждал сигнала когда пакет принят 
        osDelay(5);
        eMBPoll();
    }

}

/* ----------------------- Modbus register callback functions ---------------------------------*/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    eStatus = MB_ENOREG;
    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    UNUSED( pucRegBuffer );
    UNUSED( usAddress );
    UNUSED( usNRegs );
    UNUSED( eMode );

    return MB_ENOREG;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    eMBErrorCode eStatus = MB_ENOERR;

    // FreeModbus использует 1-based адресацию!
    // Запрос адреса 0 в Modbus → usAddress = 1 в callback
    // Запрос адреса 1 в Modbus → usAddress = 2 в callback
    // Конвертируем 1-based адресацию FreeModbus в 0-based
    // Защита от underflow (на всякий случай)
    if (usAddress > 0) {
        usAddress--;  // Декремент только если адрес > 0
    }

    if (eMode == MB_REG_READ)
    {
        // Проверяем диапазон адресов
        if (usAddress + usNCoils > 8)
        {
            return MB_ENOREG;
        }
        
        // Устанавливаем все запрошенные coils в 0
        for (USHORT i = 0; i < usNCoils; i++)
        {
            // Используем утилиту FreeModBUS для установки битов
            xMBUtilSetBits(pucRegBuffer, usAddress + i, 1, 0);
        }
        
        eStatus = MB_ENOERR;
    }
    else
    {
        // Запись не поддерживаем
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    UNUSED( pucRegBuffer );
    UNUSED( usAddress );
    UNUSED( usNDiscrete );

    return MB_ENOREG;
}