#include "system_tm4c1294.h"                                                    // CMSIS-Core
#include "driverleds.h"                                                         // device drivers
#include "cmsis_os2.h"                                                          // CMSIS-RTOS
#include "driver_tc.h"
//UART necessary includes:
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"                                                        //UART interrupt
#include "driverlib/interrupt.h"                                                //UART interrupt

//------------------------------------------------------------------------------
typedef struct{
    float derState;                                                             // Last position input
    uint32_t integratState;                                                     // Integrator state
    uint32_t integratMax;                                                       // Maximum and minimum
    uint32_t integratMin;                                                       // allowable integrator state
    float integratGain;                                                         // integral gain
    float propGain;                                                             // proportional gain
    float derGain;                                                              // derivative gain
} SPid_t;
//------------------------------------------------------------------------------
// RTOS Attributes:
// Mutex Attributes:
const osMutexAttr_t Thread_Mutex_attr = {           
  "myThreadMutex",                                                              // human readable mutex name
  osMutexRecursive,                                                             // attr_bits
  NULL,                                                                         // memory for control block   
  0U                                                                            // size for control block
};
//------------------------------------------------------------------------------
// RTOS IDs:
// Timer IDs:
osTimerId_t timer1_id;                                                          //
osTimerId_t timer_display_id;

// Tread IDs:
osThreadId_t controller_id;                                                     //
osThreadId_t pulse_counter_id;                                                  //
osThreadId_t display_control_id;                                                //
osThreadId_t system_config_id;                                                  //

// Mutex IDs:
osMutexId_t mutex1_id_PWM;                                                      //
osMutexId_t mutex1_id_GPIO;                                                     //
osMutexId_t mutex1_id_UART;                                                     //
//------------------------------------------------------------------------------
//Variables declaration:
SPid_t pid_controller;                                                          // PID parameters+
int motor_spd;                                                                  // actual motor speed
int setpoint;                                                                   // desired motor speed
bool direction;                                                                 // 0 ('-') -> clockwise, 1 ('+') -> anticlockwise
char UARTbuffer[20];                                                            // 
extern uint32_t SystemCoreClock;                			        // clock obtido pelo SysCtlClockSet
int u;                                                                          // controller output
bool TIMER_OVERFLOW;                                                            // 
uint32_t ticksPWM;                                                              //                                                                           // 

#define offset                12500                                             // PWM offset
#define resol                 20                                                // encoder resolution
#define freqPWM               500
#define PWM_lim               65536
#define MSGQUEUE_OBJECTS      1                                                 // maximum numbers of messages on the queue 
#define T                     50                                                // sampling time = 10ms
#define system_config_flag    0x0001
#define pulse_counter_flag    0x0002
#define controller_flag       0x0004
#define display_control_flag  0x0008
#define syscfg_flag           0x0010
//------------------------------------------------------------------------------
// System Configuration:
void UARTConfig(void){ 
  // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
  // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  //GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_2);
    UARTDisable(UART0_BASE);
    UARTStdioConfig(0, 115200, SystemCoreClock);
    
    //UART Interrupt: 
    UARTIntEnable(UART0_BASE, UART_INT_RX);
    IntEnable(INT_UART0);
    IntMasterEnable();
}//UARTConfig//-----------------------------------------------------------------

void DCMC_systemInit(void){
  CounterConfig();
  UARTConfig();
  
  // Set PF2 and PF3 as direction control:
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                  // Enable GPIO F
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));                           // Wait for the GPIO F port to be ready.
  
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);              // Pins PF2 and PF3 as output
  GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, 0);                                  // Pin PF3 off
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);                                 // Pin PF2 off
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_12MA,
                   GPIO_PIN_TYPE_STD);  
  
  ticksPWM = (uint32_t)((SystemCoreClock/(4*freqPWM))+1);
  if(ticksPWM>PWM_lim){
    ticksPWM = PWM_lim;
  }
  
  SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
  GPIOPinConfigure(GPIO_PF1_M0PWM1);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DBG_RUN| 
                  PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ticksPWM);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
  PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}//DCMC_systemInit//------------------------------------------------------------
//------------------------------------------------------------------------------
void system_config(void *argument){                                             // receives messages from UART and convert them to uint32_t and a flag which contains
                                                                                // the direction of the DC motor
    int setpoint_usr = 0;
    while(1){      
        osThreadFlagsWait(system_config_flag,osFlagsWaitAny,osWaitForever);
        UARTgets((char*)&UARTbuffer, 20);
        setpoint_usr = atoi((const char*)(&UARTbuffer));
        if(setpoint_usr>=0){
          setpoint = setpoint_usr;
          direction = 1;
        }else{
          setpoint = (0-setpoint_usr);
          direction = 0;
        }
        ResetCounter();
        StartCounter();
        osTimerStart(timer1_id, T);                                          // pulse_counter timer period = T1 ms
        osTimerStart(timer_display_id, 500);                                    // display timer period = 500 ms
        osThreadFlagsSet(display_control_id,syscfg_flag);
    }  
}//systemConfig//---------------------------------------------------------------

void callback(void *arg){
    osThreadFlagsSet(pulse_counter_id,pulse_counter_flag);
}//callback//-------------------------------------------------------------------

void callback_display(void *arg){
    osThreadFlagsSet(display_control_id,display_control_flag);
}//callback_display//-----------------------------------------------------------

void pulse_counter(void *argument){
  motor_spd = 0;
    while(1){
      osThreadFlagsWait(pulse_counter_flag,osFlagsWaitAny,osWaitForever);
      if(!TIMER_OVERFLOW){
        motor_spd = GetCounter();
        ResetCounter();
        motor_spd = motor_spd*1000*60/(T*resol);
      }else{
        ResetCounter();
      }
      osThreadFlagsSet(controller_id,controller_flag);
    }
}//pulse_counter//--------------------------------------------------------------
    
void controller(void *argument){
    pid_controller.derGain = 0;                                                 //
    pid_controller.integratGain = 5.5;                                          //
    pid_controller.propGain = 16;
    pid_controller.integratState = 0;
    pid_controller.integratMax = ticksPWM;
    pid_controller.integratMin = 0;
    float e = 0;
    float pTerm = 0;
    float iTerm = 0;
    u = 0;                                                                      // 
    while(1){
      osThreadFlagsWait(controller_flag,osFlagsWaitAny,osWaitForever);
      e = (float)(setpoint-motor_spd);
      pTerm = e*pid_controller.propGain;
      pid_controller.integratState += e;
      if (pid_controller.integratState > pid_controller.integratMax){
        pid_controller.integratState = pid_controller.integratMax;
      }
      if (pid_controller.integratState < pid_controller.integratMin){
        pid_controller.integratState = pid_controller.integratMin;
      }
      iTerm = pid_controller.integratGain*pid_controller.integratState;
      
      u = (int)(pTerm+iTerm);
      u = u+offset;
      if(u < offset){
        u = offset;
      }
      if(u > ticksPWM-1){
        u = ticksPWM-1;
      }
      osMutexAcquire(mutex1_id_GPIO, osWaitForever);
      if(direction){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);                           //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);                  //
      }else{
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);                  //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);                           //
      }
      osMutexRelease(mutex1_id_GPIO);
      osMutexAcquire(mutex1_id_PWM, osWaitForever);
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, u);
      osMutexRelease(mutex1_id_PWM);
    } // while
}//controller//-----------------------------------------------------------------

void display_control(void *argument){
  osMutexAcquire(mutex1_id_UART,osWaitForever);
  UARTprintf("\n===================================INSTRUCOES===================================\n");
  UARTprintf("Sentido do motor:\n - -> sentido horario.\n + -> sentido anti horario.\nColoque o sentido e a velocidade de interesse, em seguida pressione Enter para confirmar o valor de velocidade.\n");
  UARTprintf("================================================================================\n");
  UARTprintf("Insira o setpoint do motor:\n");
  osMutexRelease(mutex1_id_UART);
  osThreadFlagsWait(syscfg_flag,osFlagsWaitAny,osWaitForever);
  osMutexAcquire(mutex1_id_UART,osWaitForever);
  UARTprintf("================================================================================\n");
  UARTprintf("Setpoint [RPM]\tVelocidade Atual [RPM]\t   Sentido\n");
  osMutexRelease(mutex1_id_UART);
  while(1){
    osThreadFlagsWait(display_control_flag,osFlagsWaitAny,osWaitForever);
    osMutexAcquire(mutex1_id_UART,osWaitForever);
    UARTprintf("                                           \r");              // limpa a linha de valores da tela
    UARTprintf("    %i    \t\t",setpoint);
    UARTprintf("%i",motor_spd);
    if(setpoint==0){
      UARTprintf("\r");
    }else{
      if(direction){
        UARTprintf("\t\t   Horario\r");
      }else{
       UARTprintf("\t\t Anti Horario\r");
      }
    }
    osMutexRelease(mutex1_id_UART);
  }  
}//displayControl//-------------------------------------------------------------

void main(void){
    SystemInit();
    DCMC_systemInit();
    osKernelInitialize();
    
    //create mutexes:
    mutex1_id_PWM = osMutexNew(&Thread_Mutex_attr);                             //
    mutex1_id_GPIO = osMutexNew(&Thread_Mutex_attr);                            //
    mutex1_id_UART = osMutexNew(&Thread_Mutex_attr);                            //
    
    //create threads:
    controller_id = osThreadNew(controller, NULL, NULL);                        //
    osThreadSetPriority(controller_id , osPriorityRealtime7); 
    pulse_counter_id = osThreadNew(pulse_counter, NULL, NULL);                  //
    osThreadSetPriority(pulse_counter_id, osPriorityISR);
    display_control_id = osThreadNew(display_control, NULL, NULL);              //
    system_config_id = osThreadNew(system_config, NULL, NULL);                  //
    
    //create timer:
    timer1_id = osTimerNew(callback, osTimerPeriodic, NULL, NULL);
    timer_display_id = osTimerNew(callback_display, osTimerPeriodic, NULL, NULL);
    
    if(osKernelGetState() == osKernelReady)
        osKernelStart();

    while(1);
}//main//-----------------------------------------------------------------------