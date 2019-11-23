#include "Encoder.h"
#define TAG "ENCODER"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;
osTimerId_t timer1_id;

void callback(void* arg)
{
    //
    //Printing current motor speed and its direction in UART
    //
    osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
    UARTprintf(TAG" -> Velocidade do motor: %d | Sentido: %d\n",measurement.velocidade, measurement.sentido);
    osSemaphoreRelease(mutexUartDriver_id);
    
} // RTOS Timer callback

void xEncoderTask(void *arg){
    
    // Creating RTOS periodic timer
    timer1_id = osTimerNew(callback, osTimerPeriodic, NULL, NULL);
    
    // Starts timer with periodicity of PRINT_TIME_MS
    osTimerStart(timer1_id, PRINT_TIME_MS);
    
    //LOCAL VARIABLES
    int inSpeed;
    bool wise;
    
    while(1) {
        
        // Acquiring info from encoder module
        inSpeed = QEIVelocityGet(QEI0_BASE);    //Num. of pulses in PERIOD_MS
        wise = (QEIDirectionGet(QEI0_BASE) == 1) ? true:false;
        
        // Converting to RPM
        inSpeed = (int)((float)((float)30000/((float)HOLES_PER_REV*(float)PERIOD_MS)) * (float)inSpeed);   //Pulses to RPM
      
        // Storing data in global structure
        osSemaphoreAcquire(mutexMeasurement_id, osWaitForever);
        measurement.velocidade = inSpeed;
        measurement.sentido = wise;
        osSemaphoreRelease(mutexMeasurement_id);
        
        osDelay(10);
    }
}