/*
*       Sistemas Embarcados - Lab3
*     
*       Adriano Ricardo de Abreu Gamba
*       Davi Wei Tokikawa
*/

//
// Includes
//
#include "Inits.h"
#include "Setpoint.h"

//
// Defines
//
#define DEBUG_MODE 0
#define BUFFER_SIZE 8

//
// Global Variables
//
extern void UARTStdioIntHandler(void);

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

/***************************************************************/

osThreadId_t setpoint_id;
//osSemaphoreId_t vazio_id, cheio_id;
//uint8_t buffer[BUFFER_SIZE];
//
//void produtor(void *arg){
//  uint8_t index_i = 0, count = 0;
//  
//  while(1){
//    osSemaphoreAcquire(vazio_id, osWaitForever); // há espaço disponível?
//    buffer[index_i] = count; // coloca no buffer
//    osSemaphoreRelease(cheio_id); // sinaliza um espaço a menos
//    
//    index_i++; // incrementa índice de colocação no buffer
//    if(index_i >= BUFFER_SIZE)
//      index_i = 0;
//    
//    count++;
//    count &= 0x0F; // produz nova informação
//    osDelay(500);
//  } // while
//} // produtor
//
//void consumidor(void *arg){
//  uint8_t index_o = 0, state;
//  
//  while(1){
//    osSemaphoreAcquire(cheio_id, osWaitForever); // há dado disponível?
//    state = buffer[index_o]; // retira do buffer
//    osSemaphoreRelease(vazio_id); // sinaliza um espaço a mais
//    
//    index_o++;
//    if(index_o >= BUFFER_SIZE) // incrementa índice de retirada do buffer
//      index_o = 0;
//    
//    LEDWrite(LED4 | LED3 | LED2 | LED1, state); // apresenta informação consumida
//    osDelay(500);
//  } // while
//} // consumidor

void main(void){
  
  SystemInit();
  
  //
  //Inicializa periféricos
  //
  FPUEnable();
  FPULazyStackingEnable();
  IntMasterEnable();
  GPIOInit();
  TIMERInit();
  UARTInit();
  PWMInit();
//  LEDInit(LED4 | LED3 | LED2 | LED1);

  osKernelInitialize();

//  setpoint_id = osThreadNew(xSetpointTask, NULL, NULL);
//  consumidor_id = osThreadNew(consumidor, NULL, NULL);

//  vazio_id = osSemaphoreNew(BUFFER_SIZE, BUFFER_SIZE, NULL); // espaços disponíveis = BUFFER_SIZE
//  cheio_id = osSemaphoreNew(BUFFER_SIZE, 0, NULL); // espaços ocupados = 0
  
  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
} // main
