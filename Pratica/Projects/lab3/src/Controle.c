#include "Controle.h"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xControleTask(void *arg){
    
    // PI controller variable structure
    typedef struct{
        float integratState; // Integrator state
        float integratMax;   // Maximum and minimum
        float integratMin;   // allowable integrator state
        float kp;                 // integral gain
        float ki;                 // proportional gain
        float ref;                // reference value
        float v;                  // Sensor Value
        float y;                  // out PWM value
        float e;                  // Current error
    } StructPI;
    
    // Variables initialization
    StructPI PI = {0, PWMTICKS, 0, KP, KI, 0, 0, 0, 0};
    float P = 0;    // Parte proporcional do controle
    float I = 0;    // Parte integrativa do controle
    bool dir = true;
    
    while(1){
        
        // Obter a referencia atual
        osMutexAcquire(mutexSetPointParams_id, osWaitForever);
        PI.ref = configs.velocidade;
        dir = configs.sentido;
        osMutexRelease(mutexSetPointParams_id);
        
        // Obter a velocidade atual
        osMutexAcquire(mutexMeasurement_id, osWaitForever);
        PI.v = measurement.velocidade;
        osMutexRelease(mutexMeasurement_id);

        // Regra de controle PI
        PI.e = PI.ref - PI.v;           // Ganho atual
        P = PI.e*PI.kp;                 // Parte proporcional da regra
        
        PI.integratState = PI.integratState + PI.e;
        if (PI.integratState > PI.integratMax)
        {
            PI.integratState = PI.integratMax;
        }
        else if (PI.integratState < PI.integratMin)
        {
            PI.integratState = PI.integratMin;
        }
        I = PI.integratState*PI.ki;     // Parte integrativa da regra

        PI.y = P + I;                   // Regra de controle

        PI.y = PI.y + PWMOFFSET;        // Ajuste da saída
        if (PI.y < PWMOFFSET)
        {
            PI.y = PWMOFFSET;
        }
        if (PI.y > PWMTICKS)
        {
            PI.y = PWMTICKS-1;
        }
        
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int)PI.y);
        
        if(dir) //CLOCKWISE
        {
          GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
          GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);
        }
        else    //COUNTERCLOCKWISE
        {
          GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
          GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
        
#if DEBUG_MODE
        //TODO: CRIAR OUTRO TIMER DE RTOS PARA PRINT DO PWM DE SAIDA
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf("Velocidade = %i  |  PWM = %i percent \n",(int)PI.v,(int)(PI.y*100/((float)PWMTICKS))); 
        osSemaphoreRelease(mutexUartDriver_id);
#endif
        osDelay(1000);
    }
}