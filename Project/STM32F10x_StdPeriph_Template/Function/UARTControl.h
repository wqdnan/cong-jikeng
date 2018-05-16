#ifndef __UARTCONTROL_H
#define __UARTCONTROL_H

//#include "main.h"
#include "UARTHandle.h"

//typedef struct
//{

//	slaveCommStruct * slaveComm;                 //从机通信控制结构体
//	slaveCtntStruct * slaveData;               //从机通信内容结构体


//}ctrlCommStruct;


///////////////////////////////////////////////////////////////
typedef struct
{
	uint8_t temp;


}commCtntStruct;//自定义的的功能数据结构

//////////////////////////////////////////////////////////////

extern commCtntStruct commData;

//功能函数
e_state setParaData(uartCtntStruct * uartData,commCtntStruct * commData);
e_state setFlashData(uartCtntStruct *uartData);
////////////////////////////////需要重新改写的功能函数/////////////////////////////////////////
/* 接收到的数据内容进行功能处理 */
e_state UartRxFctnCtntHandle(uartCtntStruct *uartData,commCtntStruct * commData);
/* 待发送的数据帧准备处理 */
e_state UartTxFctnCtntHandle(uartCtntStruct *uartData,commCtntStruct * commData);
/*  */


////////////////////////////////////////////////////////////////////////

void UartTxHandle(USART_TypeDef* USARTx,uint8_t * txStr,uartCtntStruct * uartData,commCtntStruct * commData);
e_state UartRcvHandle(char * rxStr,uint8_t length,uartCtntStruct * uartData,commCtntStruct * commData);









#endif
