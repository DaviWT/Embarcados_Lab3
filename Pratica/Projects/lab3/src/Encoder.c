#include "Encoder.h"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xEncoderTask(void *arg){
    int i = 0;
    while(1) {
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf("Hello Encoder! (%i)\n",i);
        osSemaphoreRelease(mutexUartDriver_id);
        i++;
        osDelay(500);
    }
//  uint8_t index_i = 0, count = 0;
//  
//  while(1){
//    osSemaphoreAcquire(vazio_id, osWaitForever); // há espaço disponível?
//    //SEÇÃO CRITICA
//    osSemaphoreRelease(cheio_id); // sinaliza um espaço a menos
//    
//    osDelay(500);
//  } // while  
}