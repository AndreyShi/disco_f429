#include "FreeRTOS.h"
#include "cmsis_os2.h"



#include "mb.h"
#include "mbport.h"
#include "port_internal.h"

void modbus_task_func(void *argument){

    eMBInit(MB_RTU, 1, 0, 9600, MB_PAR_NONE, 1);
    eMBEnable();

    while(1){
        osDelay(100);
        // Запрос к разным slave-устройствам
       // eMBFuncReadCoils( UCHAR * pucFrame, USHORT * usLen );
       //eMBMasterReqReadHoldingRegister(1, 0, 10, 1); // К slave с адресом 1
       //eMBMasterReqReadHoldingRegister(2, 0, 10, 1); // К slave с адресом 2
       //eMBMasterReqReadHoldingRegister(3, 0, 10, 1); // К slave с адресом 3
    }

}