#ifndef INITS_H
#define INITS_H

//
//  Includes
//
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
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
#include "system_tm4c1294.h"    // CMSIS-Core
#include "driverleds.h"         // device drivers
#include "cmsis_os2.h"          // CMSIS-RTOS

//
//  Defines
//
#define DEBUG_MODE 0
#define PWMTICKS 15000      // Number of PWM ticks that define the PWM period
#define PWMOFFSET 10000     // Minimum number of PWM ticks to keep the motor working properly
#define REF 0               // Refence value of stabilization
#define KP 0                // Proporcional constant of the PI controller
#define KI 0                // Integrative constant of the PI controller

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


#endif