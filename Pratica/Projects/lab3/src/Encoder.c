#include "Encoder.h"

void xEncoderTask(void *arg){
    int i = 0;
    osDelay(100);
    while(1) {
        UARTprintf("Hello Encoder! (%i)\n",i);
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