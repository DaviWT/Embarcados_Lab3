//*****************************************************************************
//
// 	Authors:	Adriano Ricardo de Abreu Gamba
//				Erika Maria Capote Both
//
// 	Subject: Sistemas Operacionais
//
// 	Date: 07/2019
//
// 	Final Project of the discipline
//
// 	Description: 
//  This project aims to control the water level of a tank by using PI 
//  controller with an ultrassonic sensor and a voltage controlled water pump. 
//
//  Ultrassonic Sensor:
//  - Pins:
//      VCC to 5V;
//      GND to GND;
//      Trigger to PG0 (out);
//      Echo to PG1 (in);
//
//  PWM peripheral:
//  - To do the configuration, were used PWM0_BASE base with PWM_GEN_0;
//  - The PWM implemented has 10kHZ;
//  - Output pins:
//      F0 and F1.
//
//  Socket Communication:
//  @TODO ERIKA se achar interessante, coloque algo aki! =P
//
//	Obs:
//  1) Based on CMSIS-RTOS v2 (RTX5);
//	2) Based on TivaWare_C_Series-2.1.4.178;
//	3) Uses a serial terminal with 115,200.
//
//*****************************************************************************

#include "system_tm4c1294.h" // CMSIS-Core
#include "cmsis_os2.h" // CMSIS-RTOS
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
//------------------------------------------------------------------------------
// PI Strucutre
typedef struct{
    uint32_t integratState; // Integrator state
    uint32_t integratMax;   // Maximum and minimum
    uint32_t integratMin;   // allowable integrator state
    float ki;               // integral gain
    float kp;               // proportional gain
    int ref;                // reference value
    int h;                  // Sensor Value
    int y;                  // out PWM value
    int e;                  // Current error
} StructPI;
//------------------------------------------------------------------------------
// RTOS Attributes:
// Mutex Attributes:
const osMutexAttr_t Thread_Mutex_attr = { "myThreadMutex",  // human readable mutex name
                                          osMutexRecursive, // attr_bits
                                          NULL,             // memory for control block   
                                          0U };             // size for control block
// Message queue Attributes:
#define MSGQUEUE_OBJECTS 1
//------------------------------------------------------------------------------
// RTOS IDs:

// Thread IDs:
osThreadId_t controller_id;                                                     
osThreadId_t distanceSensor_id; 
osThreadId_t UARTcommunication_id;
osThreadId_t SOCKETcommunication_id;

// Mutex IDs:
osMutexId_t mutex1_id_PWM;                                                      
osMutexId_t mutex1_id_GPIO;
osMutexId_t mutex1_id_Distancia;
osMutexId_t mutex1_id_UART;

// Message Queue IDs:
osMessageQueueId_t MsgQueueDistance_id;
//------------------------------------------------------------------------------
//Variables declaration:
extern uint32_t SystemCoreClock;   // clock obtido pelo SysCtlClockSet
StructPI PI;
int distancia;
int flagParaMotor;
#define PWMOFFSET       10000
#define DEFAULTHIGHT    1564       // Value to set the hight read by sensor to 0
#define INTENTEDHIGHT   600        // Water HIght to be controlled to
#define PWMTICKS        30000      // Number of PWM ticks that define the PWM period

//*****************************************************************************
//
// 1ms Delay
//
//*****************************************************************************
void delay_ms(int ms)
{
    SysCtlDelay( (SystemCoreClock/(3*1000))*ms );
}

//*****************************************************************************
//
// Configure system peripherals and GPIO
//
//*****************************************************************************

//
// Configure the GPIO pins and its devices.
//
void ConfigureGPIO(void)
{
    // Configure the device pins and enable Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Set GPIO N (LED D1 = PN1)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Wait until done
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Set GPIO J (push-button SW1 = PJ0, push-button SW2 = PJ1)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Wait until done
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); // Set GPIO G
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)); // Wait until done
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Set GPIO F (PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Wait until done

    // Set pin PN1 to out for the LED D1
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0); // Initialize LED D1 turned off
    GPIOPadConfigSet( GPIO_PORTN_BASE,
                      GPIO_PIN_0 | GPIO_PIN_1,
                      GPIO_STRENGTH_12MA,
                      GPIO_PIN_TYPE_STD );
    
    // Set pins PJ0 and PJ1 to in for buttons SW1 and SW2
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet( GPIO_PORTJ_BASE,
                      GPIO_PIN_0 | GPIO_PIN_1,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU);
    
    // Set pin PG0 to out for the Trigger pin of the ultrassonic
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0); // Initialize with 0
    GPIOPadConfigSet( GPIO_PORTN_BASE,
                      GPIO_PIN_0 | GPIO_PIN_1,
                      GPIO_STRENGTH_12MA,
                      GPIO_PIN_TYPE_STD );
    
    // Set pin PG1 to in for the Echo pin of the ultrassonic
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPadConfigSet( GPIO_PORTG_BASE,
                      GPIO_PIN_1,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPD );
    
    // Set pins PF0 and PF1 to out for PWM
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
}

//
// Configure the UART and its pins.
//
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, SystemCoreClock);
}

//
//  Configure the PWM peripheral
//
void ConfigurePWM (void)
{
    // Enable the PWM0 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)); // Wait until done 
    
    // Set the PWM clock as an division of the system clock. In this example, 
    // the PWM clock mathces the system clock.
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    
    // Configure the PWM generator for count down mode
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    // Set the Period (expressed in clock ticks). For Example, in order to make
    // a PWM clock with 10kHZ, is used 12000 clock ticks.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMTICKS);
    
    // Set the pulse width of PWM0 for a 50% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 100);

    // Set the pulse width of PWM1 for a 10% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 100);
    
    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);    
}

//
// Call of the configuration functions
//
void ConfigureSystem(void)
{
    ConfigureGPIO();
    ConfigureUART();
    ConfigurePWM();
}

//*****************************************************************************
//
// Thread controller_id
//
//*****************************************************************************
void controller(void *arg)
{
    float P = 0; // Parte proporcional do controle
    float I = 0; // Parte integrativa do controle
    while(1)
    {
        // UARTprintf("Estou na thread controller\n"); //Para DEBUG --Adriano
      
        // Botão para parar o Motor
        osMutexAcquire(mutex1_id_GPIO, osWaitForever);
        if(!(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0)) // Testa estado do push-button SW2 (Retoma o motor)
        {
            flagParaMotor = 1;
        }
        if(!(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == GPIO_PIN_1)) // Testa estado do push-button SW1 (Parar o motor)
        {
            flagParaMotor = 0;
        }
        osMutexRelease(mutex1_id_GPIO);
        
        // Se o sistema estiver executando, faz o controle, senão não faz e mantém o estado do controle constante
        if (flagParaMotor == 0)
        {
            // Obter a distância atual
            osMutexAcquire(mutex1_id_Distancia, osWaitForever);
            PI.h = DEFAULTHIGHT-distancia;
            osMutexRelease(mutex1_id_Distancia);
            
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
        }
        
        // Set new PWM value
        osMutexAcquire(mutex1_id_PWM, osWaitForever);
        if (flagParaMotor == 1)
        {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 100);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 100);
        }
        else
        {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PI.y);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PI.y);
        }
        osMutexRelease(mutex1_id_PWM);
        //UARTprintf("Altura = %i  |  PWM = %i\n",PI.h,PI.y);  //Para DEBUG --Adriano
        osDelay(10);
    } // while
} // controller_id

//*****************************************************************************
//
// Thread distanceSensor_id
//
//*****************************************************************************
void distanceSensor(void *arg)
{
    uint32_t counter = 0; // varible to hold the distance related to clock counts
    while(1)
    {
        // UARTprintf("Estou na thread distanceSensor\n");  Para DEBUG --Adriano
        
        // Reset counter
        counter = 0;
        
        // Pull trigger high
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);
        
        // wait appropriately
        SysCtlDelay( SystemCoreClock/(3*10) ); //Delay 10uS
        
        // Pull trigger low
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0);
        
        // Monitor echo for rising edge
        while (GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1) == 0) {};
        
        // loop counter checking for falling edge; MAX_TIME make sure we don't miss it
        while (GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1) != 0)
        {
            if (counter >= 15000)
                break;
            counter++;
        }
        
        //UARTprintf("Valor Enviado = %i\n",counter);  //Para DEBUG --Adriano
        osMutexAcquire(mutex1_id_Distancia, osWaitForever);
        // Inicio da seção crítica
        distancia = counter;
        // Fim da seção crítica
        osMutexRelease(mutex1_id_Distancia);
        osDelay(10);
    } // while
} // distanceSensor

//*****************************************************************************
//
// Thread UARTcommunication_id
//
//*****************************************************************************
void UARTcommunication(void *arg)
{
    int altura = 0;
    while(1)
    {
        // Obter a altura atual
        osMutexAcquire(mutex1_id_Distancia, osWaitForever);
        altura = DEFAULTHIGHT-distancia;
        osMutexRelease(mutex1_id_Distancia);
        
        // Envia o dado da Altura por UART
        UARTprintf("%i\n",altura);
        osDelay(10);
    } // while
} // UARTcommunication

//*****************************************************************************
//
// Thread SOCKETcommunication
//
//*****************************************************************************
void SOCKETcommunication(void *arg)
{
    int a = 0;
    while(1)
    {
        a++;
        if(a>5)
        {
            a = 0;
        }
        osDelay(10);
    } // while
} // SOCKETcommunication

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
void main(void)
{
    // PI Controller initialization
    PI.ref = INTENTEDHIGHT;
    PI.kp = 10;
    PI.ki = 0.5;
    PI.h = 0;
    PI.y = 0;
    PI.integratState = 0;
    PI.integratMax = PWMTICKS;
    PI.integratMin = 0;
    
    // system initialization
    SystemInit();
    ConfigureSystem();
    osKernelInitialize();
    
    // create mutexes:
    mutex1_id_PWM = osMutexNew(&Thread_Mutex_attr);                             
    mutex1_id_GPIO = osMutexNew(&Thread_Mutex_attr);
    mutex1_id_Distancia = osMutexNew(&Thread_Mutex_attr);  
    mutex1_id_UART = osMutexNew(&Thread_Mutex_attr);
    
    // create message queue
    MsgQueueDistance_id = osMessageQueueNew( MSGQUEUE_OBJECTS,
                                             sizeof(int),
                                             NULL );
    if (!MsgQueueDistance_id)
    {
        UARTprintf("Erro na Criação da Fila!\n"); // Message Queue object not created, handle failure
    }
    
    // create threads:
    controller_id = osThreadNew(controller, NULL, NULL);                        
    osThreadSetPriority(controller_id , osPriorityRealtime7); 
    distanceSensor_id = osThreadNew(distanceSensor, NULL, NULL);                  
    osThreadSetPriority(distanceSensor_id, osPriorityISR);
    UARTcommunication_id = osThreadNew(UARTcommunication, NULL, NULL);                  
    osThreadSetPriority(UARTcommunication_id, osPriorityISR);
    SOCKETcommunication_id = osThreadNew(SOCKETcommunication, NULL, NULL);
    osThreadSetPriority(SOCKETcommunication_id, osPriorityISR);
    
    // Inicializate variables
    flagParaMotor = 1;
    
    // Beginning
    UARTprintf("Início\n");
      
    if(osKernelGetState() == osKernelReady)
        osKernelStart();

    while(1);
} // main
