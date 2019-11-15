#include "Encoder.h"
#define TAG "ENCODER"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xEncoderTask(void *arg){
    //LOCAL VARIABLES
    int inSpeed;
    bool wise;
    
    //TODO: QEIVelocityEnable()
    
    while(1) {
        
        //TODO: QEIVelocityGet()
        //TODO: QEIDirectionGet()
      
        osSemaphoreAcquire(mutexMeasurement_id, osWaitForever);
        //escrita na struct de medição
        measurement.velocidade = inSpeed;
        measurement.sentido = wise;
        osSemaphoreRelease(mutexMeasurement_id);
      
        //TODO: INSERIR LOGICA DE CONTADORES PARA NAO SOBRECARREGAR UART
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf(TAG"| Velocidade do motor: %d | Sentido: %d\n",measurement.velocidade, measurement.sentido);
        osSemaphoreRelease(mutexUartDriver_id);
        
        osDelay(500);
    }
}