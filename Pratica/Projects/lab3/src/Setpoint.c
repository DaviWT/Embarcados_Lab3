#include "Setpoint.h"

extern osMutexId_t mutexUartDriver_id, mutexSetPointParams_id, mutexMeasurement_id;

void xSetPointTask(void *arg){
    
    //SHOW INSTRUCTION MENU
    osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
    UARTprintf("Welcome to the DC Motor controller made by:\r\n-Adriano Gamba\r\n-Davi Tokikawa\r\n");
    UARTprintf("Instructions:\r\n-Type a \"plus (+)\" or \"minus (-)\" signal in front of the desired speed\r\n");
    UARTprintf("\t-PLUS sets motor to clockwise\r\n\t-MINUS sets motor to counter-clockwise\r\n");
    UARTprintf("-Followed by the desired motor speed in RPM\r\n");
    UARTprintf("Example: \"+1500\"\r\n\r\nType in below:\r\n");
    osSemaphoreRelease(mutexUartDriver_id);
    
    char UART_input[8];
    while(1) {
        //LOCAL VARIABLES
        int inSpeed;
        bool wise;
        
        //READS FROM UART
        do
        {
          UARTgets(UART_input,7);       //Internal loop until rcv from UART
          inSpeed = validateUartInput(UART_input,&wise);      //check if is valid input
        }while(inSpeed == -1);
        
        //PRINTS SUCCESS MESSAGE
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf(">>> Success! Input was: \"%s\"\n",UART_input);
        osSemaphoreRelease(mutexUartDriver_id);

        //UPDATES SETPOINT VARIABLES
        osSemaphoreAcquire(mutexSetPointParams_id, osWaitForever);
        configs.velocidade = inSpeed;
        configs.sentido = wise;
        osSemaphoreRelease(mutexSetPointParams_id);
        
#if DEBUG_MODE
        osSemaphoreAcquire(mutexUartDriver_id, osWaitForever);
        UARTprintf("Vel: %d | Sent: %d\r\n",configs.velocidade, configs.sentido);
        osSemaphoreRelease(mutexUartDriver_id);
#endif
        
    }
}

/*
Validates the UART input for the project
-Speed must be over 666 and under 12500 RPM
-Check if character upfront is "+" or "-"
Returns the input RPM speed or -1 for error
*/
int validateUartInput(char* in, bool* wise)
{
  if(strlen(in) > 6)
  {
    return -1;
  }
  if(in[0] != '+')
  {
    if(in[0] != '-')
    {
      return -1;
    }
  }
  
  //SETS CLOCKWISE or COUNTER-CLOCKWISE
  if(in[0] == '+')
    *wise = true;
  else
    *wise = false;
  
  char* speedStr = in + 1;
  char *resto;
  int speed = strtol(speedStr,&resto,10);
  if(speed == 0)
    return -1;
  if(speed >= 666 && speed < 12500)
    return speed;
  return -1;
}