#include "Setpoint.h"

void xSetPointTask(void *arg){
    int i = 0;
    while(1) {
        UARTprintf("Hello SetPoint! (%i)\n",i);
        i++;
        osDelay(500);   
    }
//  uint8_t index_i = 0, count = 0;
//  
//  while(1){
//    osSemaphoreAcquire(vazio_id, osWaitForever); // h� espa�o dispon�vel?
//    //SE��O CRITICA
//    osSemaphoreRelease(cheio_id); // sinaliza um espa�o a menos
//    
//    osDelay(500);
//  } // while  
}