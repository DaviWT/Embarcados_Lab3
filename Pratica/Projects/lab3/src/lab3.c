//******************************************************************************
//  LAB 3 SISTEMAS EMBARCADOS - UTFPR 2019/2
//
//  Alunos:
//  -Adriano Ricardo de Abreu Gamba
//  -Davi Wei Tokikawa
//
//  Controle de velocidade e sentido de motor DC
//
//  Outputs: PWM, velocidade e sentido do motor (Serial).
//  Inputs: Encoder.
//
//  Obs:
//  1) Based on TivaWare_C_Series-2.1.4.178;
//******************************************************************************

/***** General ****************************************************************/

//
//  Includes
//
#include "Inits.h"
#include "Setpoint.h"
#include "Encoder.h"
#include "Controle.h"

//
//  Defines
//
#define BUFFER_SIZE 8

//
//  Global Variables
//
extern void UARTStdioIntHandler(void);

//
//  Callbacks
//
void UART0_Handler(void)
{
    UARTStdioIntHandler();
} // UART0_Handler

//void TimerB0Isr(void)
//{
//    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);  // Clear timer interrupt
//    HWREG(TIMER0_BASE + 0x050) = 0xFFFF;  // Reset Timer0A counting
//    
//    // Para testes com o analisador logico
//    //PIN_N5_STATE ^= GPIO_PIN_5;
//    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, PIN_N5_STATE); // Blink PIN N4
//    
//    timerCount = TimerValueGet(TIMER0_BASE, TIMER_B);   //Get first 16-bits
//    int timerPrescaler = TimerPrescaleMatchGet(TIMER0_BASE, TIMER_B);   //Get 8-bits left
//    timerCount = (timerPrescaler<<16) | timerCount;     //Concatenate to make a 24-bits value
//}
//
//void TimerA0Isr(void)
//{
//    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear timer interrupt
////    UARTprintf("Interrupcao do Timer A0\n");
////    flagInterrTimerA0 = 1;
//    // Para testes com o analisador logica
//    PIN_N5_STATE ^= GPIO_PIN_5;
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, PIN_N5_STATE); // Blink PIN N4
//    if (GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5) == 0)
//        UARTprintf("T = ? | f = ? | D = 0 \n");
//    else
//        UARTprintf("T = ? | f = ? | D = 100 \n");
//    while( UARTBusy(UART0_BASE) ){}
//
//    //digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1);         // toggle LED pin
//    TimerEnable(TIMER0_BASE, TIMER_A);
//}

/***** CMSIS-RTOS *************************************************************/

//
// Mutex Attributes
//
const osMutexAttr_t Thread_Mutex_attr = { "myThreadMutex",  // human readable mutex name
                                          osMutexRecursive, // attr_bits
                                          NULL,             // memory for control block   
                                          0U };             // size for control block

//
//  Threads declaration
//
osThreadId_t setpoint_id, controle_id, encoder_id;

//
//  Semaphores declaration
//
osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

/***** Main *******************************************************************/

void main(void){
  
    // System init
    SystemInit();

    // Peripheral init
    FPUEnable();
    FPULazyStackingEnable();
    IntMasterEnable();
    GPIOInit();
    TIMERInit();
    UARTInit();
    PWMInit();
//    LEDInit(LED4 | LED3 | LED2 | LED1);

    // CMSIS-RTOS kernel init
    osKernelInitialize();
    
    // Threads init
    setpoint_id = osThreadNew(xSetPointTask, NULL, NULL);
    encoder_id = osThreadNew(xEncoderTask, NULL, NULL);
    const osThreadAttr_t controle_id_attr = {
        .stack_size = 1024  // Create the thread stack with a size of 1024 bytes
    };
    controle_id = osThreadNew(xControleTask, NULL, &controle_id_attr);
    
    // Mutexes init
    mutexUartDriver_id = osMutexNew(&Thread_Mutex_attr);
    mutexSetPointParams_id = osMutexNew(&Thread_Mutex_attr);  
    mutexMeasurement_id = osMutexNew(&Thread_Mutex_attr);  
    
    UARTprintf("Hello lab3!\n");
    
    // CMSIS-RTOS init 
    if(osKernelGetState() == osKernelReady)
    osKernelStart();

    while(1);
}
