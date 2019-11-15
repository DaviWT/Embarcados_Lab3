#include "Controle.h"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xControleTask(void *arg){
  
  while(1)
  {
    /*
    // PI controller variable structure
    typedef struct{
        uint32_t integratState; // Integrator state
        uint32_t integratMax;   // Maximum and minimum
        uint32_t integratMin;   // allowable integrator state
        int kp;               // integral gain
        int ki;               // proportional gain
        int ref;                // reference value
        int h;                  // Sensor Value
        int y;                  // out PWM value
        int e;                  // Current error
    } StructPI;
    
    // Variables
    StructPI PI = {0, PWMTICKS, 0, KP, KI, REF, 0, 0, 0};
    float P = 0;    // Parte proporcional do controle
    float I = 0;    // Parte integrativa do controle
    
    // @TODO REMOVER CODIGO DE TESTE
    // CODIGO DE TESTE /////////////////////////////////////////////////////////
//    int i = 0;
//    osDelay(200);  
//    while(1) {
//        UARTprintf("Hello Controle! (%i)\n",i);
//        i++;
//        osDelay(500);   
//    }
    ////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    // @TODO Fazer logica de entrar velocidade em RPM e saida em porcentagem do
    //       Duty Cycle
    
    
    
    
    
    
    // Obter a velocidade atual
    osMutexAcquire(mutexMeasurement_id, osWaitForever);
    PI.h = measurement.velocidade;
    osMutexRelease(mutexMeasurement_id);

    // Regra de controle PI
    PI.e = PI.ref - PI.h;   // Ganho atual
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
    
    if (DEBUG_MODE)
        UARTprintf("Velocidade = %i  |  PWM = %i\n",PI.h,PI.y);
*/
  }
  
}