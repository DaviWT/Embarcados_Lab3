#ifndef INITS_H
#define INITS_H

//
//  Includes
//
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "system_tm4c1294.h"    // CMSIS-Core
#include "driverleds.h"         // device drivers
#include "cmsis_os2.h"          // CMSIS-RTOS

//
//  Defines
//
#define PWM_FREQ 300
#define PWMTICKS ((int)(SystemCoreClock/8)/PWM_FREQ)        // Number of PWM ticks that define the PWM period
#define PWMOFFSET 10000         // Minimum number of PWM ticks to keep the motor working properly
#define KP 1                    // Proporcional constant of the PI controller
#define KI 1                  // Integrative constant of the PI controller
#define PERIOD_MS 5             // Sampling period for the QEI component
#define PRINT_TIME_MS 500

#define DEBUG_MODE 1

//
//  Structs
//
typedef struct{
    int velocidade;
    bool sentido;
} Configs;

extern Configs configs;

typedef struct{
    int velocidade;
    bool sentido;
} Measurement;

extern Measurement measurement;

//
//  Prototypes
//
void UARTInit(void);
void PWMInit (void);
void TimerA0Isr(void);
void TIMERInit();
void GPIOInit();
void QEI_Init();


#endif