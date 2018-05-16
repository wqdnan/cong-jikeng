#ifndef __FUCTIONCONTROL_H
#define __FUCTIONCONTROL_H


#include "main.h"
#include "UARTHandle.h"

typedef struct
{
  uint8_t temp;
}fctnCtrlStruct;

extern fctnCtrlStruct ctrlData;

e_state regularFctnHandle(uartCtntStruct * uartData,fctnCtrlStruct * ctrlData);
void regularTimeDo(uartCtntStruct * uartData,fctnCtrlStruct * ctrlData);


#endif
