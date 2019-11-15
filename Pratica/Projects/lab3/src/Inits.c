#include "Inits.h"

Configs configs;
Measurement measurement;

//******************************************************************************
//
// UART Configuration
//
//******************************************************************************
void UARTInit(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 9600, SystemCoreClock);
} // UARTInit

//******************************************************************************
//
//  Configure the PWM peripheral
//
//******************************************************************************
void PWMInit (void)
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
//    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, SystemCoreClock/(8*PWM_CLOCK));
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMTICKS);    //1kHz
    
    // Set the pulse width of PWM0 for a 30% duty cycle.
//    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (PWM_DUTY/100)*SystemCoreClock/(8*PWM_CLOCK));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 4500);

    // Set the pulse width of PWM1 for a 30% duty cycle.
//    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (PWM_DUTY/100)*SystemCoreClock/(8*PWM_CLOCK));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 4500);
    
    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);    
}

//******************************************************************************
//
//  TIMER Configuration
//
//******************************************************************************
void TIMERInit()
{
    // Enable the Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    // Wait for the Timer0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}

    // Configure TimerA as a half-width one-shot timer, and TimerB as a
    // half-width edge capture counter.
    TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP |
    TIMER_CFG_B_CAP_TIME_UP));
    
    // Set the count time for the the one-shot timer (TimerA).
//    TimerLoadSet(TIMER0_BASE, TIMER_A, SystemCoreClock*TIMER_MS/1000);
    
    // Set the prescaler for TimerA
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 10);
    
    // Set the prescaler for TimerB
    TimerPrescaleSet(TIMER0_BASE, TIMER_B, 0xFF);       //8-bit prescaler
    
    //TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);
    //TimerMatchSet(TIMER0_BASE, TIMER_B, 0x0);
    
    // Configure the counter (TimerB) to count both edges.
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    
    // Registering ISRs
//    TimerIntRegister(TIMER0_BASE, TIMER_A, TimerA0Isr);
//    TimerIntRegister(TIMER0_BASE, TIMER_B, TimerB0Isr);
    
    // Interrupt priorities
//    IntPrioritySet(INT_TIMER0A, 0);
//    IntPrioritySet(INT_TIMER0B, 1);
    
    // Enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    
    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
}

//******************************************************************************
//
// GPIO Configuration
//
//******************************************************************************
void GPIOInit()
{
    // Configure the device pins and enable Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Set GPIO N (PN4 - PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Wait until done
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); // Set GPIO L (PL5 - PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)); // Wait until done
    
    // Set pin PN4 to in for the PWM signal
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPadConfigSet( GPIO_PORTN_BASE,
                      GPIO_PIN_4,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU );
    
    // Configure output pin for debug purposes
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_5); // PIN N5 como saída
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0); // PIN N5 apagados
    GPIOPadConfigSet(GPIO_PORTN_BASE, 
                     GPIO_PIN_5, 
                     GPIO_STRENGTH_12MA, 
                     GPIO_PIN_TYPE_STD);        //Configuration
    
    // Set pin PL5 to timer capture for signal in
    GPIOPinConfigure(GPIO_PL5_T0CCP1);  
    GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_5);
    
    // Set pins PF0 and PF1 to out for PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Set GPIO F (PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Wait until done
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

//TODO: DEFINIÇÃO DE QEI INIT (TUDO MENOS VELOCITY ENABLE)