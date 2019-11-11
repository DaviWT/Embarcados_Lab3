#ifndef SETPOINT_H
#define SETPOINT_H

#include "Inits.h"

void xSetPointTask(void *arg);
int validateUartInput(char* in, bool* wise);

#endif