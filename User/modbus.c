#include "FreeRTOS.h"
#include "cmsis_os2.h"

void modbus_task_func(void *argument){

    //eMBInit(MB_RTU, 0xFF, ucPort, ulBaudRate, eParity, ucStopBits);
    //eMBEnable();

    while(1){
        osDelay(1);
    }

}