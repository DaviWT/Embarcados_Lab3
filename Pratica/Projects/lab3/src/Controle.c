#include "Controle.h"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xControleTask(void *arg){
    
    // PI controller variable structure
    typedef struct{
        uint32_t integratState; // Integrator state
        uint32_t integratMax;   // Maximum and minimum
        uint32_t integratMin;   // allowable integrator state
        int kp;                 // integral gain
        int ki;                 // proportional gain
        int ref;                // reference value
        int v;                  // Sensor Value
        int y;                  // out PWM value
        int e;                  // Current error
    } StructPI;
    
    // Variables initialization
    StructPI PI = {0, PWMTICKS, 0, KP, KI, REF, 0, 0, 0};
    float P = 0;    // Parte proporcional do controle
    float I = 0;    // Parte integrativa do controle
    
    while(1){
        
        // Obter a referencia atual
        osMutexAcquire(mutexSetPointParams_id, osWaitForever);
        PI.ref = configs.velocidade;
        osMutexRelease(mutexSetPointParams_id);
        
        // Obter a velocidade atual
        osMutexAcquire(mutexMeasurement_id, osWaitForever);
        PI.v = measurement.velocidade;
        osMutexRelease(mutexMeasurement_id);

        // Regra de controle PI
        PI.e = PI.ref - PI.v;   // Ganho atual
        P = PI.e*PI.kp;         // Parte proporcional da regra
        
        PI.integratState = PI.integratState + PI.e;
        if (PI.integratState > PI.integratMax)
        {
            PI.integratState = PI.integratMax;
        }
        else if (PI.integratState < PI.integratMin)
        {
            PI.integratState = PI.integratMin;
        }
        I = PI.integratState*PI.ki; // Parte integrativa da regra

        PI.y = (int)(P + I); // Regra de controle

        PI.y = PI.y + PWMOFFSET;   // Ajuste da saída
        if (PI.y < PWMOFFSET)
        {
            PI.y = PWMOFFSET;
        }
        if (PI.y > PWMTICKS)
        {
            PI.y = PWMTICKS-1;
        }
        
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PI.y);
        
        //TODO: SETAR PINO CORRESPONDENTE AO SENTIDO
        
#if DEBUG_MODE
        //TODO: CRIAR OUTRO TIMER DE RTOS PARA PRINT DO PWM DE SAIDA
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf("Velocidade = %i  |  PWM = %i percent \n",PI.v,(int)((float)PI.y*100/(float)5000)); 
        osSemaphoreRelease(mutexUartDriver_id);
#endif
        osDelay(1000);
    }
}