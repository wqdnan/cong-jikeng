#ifndef __UARTHandle_H
#define __UARTHandle_H

#include "UART.h"
#include <String.h>


#define FUCMAXNUM 20



#define SLAVEMAXNUM 32          //设定最大的工作从机数量
//#define CHECKALIVETIME 5        //单位1s
#define CHECKDATATIME  1        //单位0.5s
#define OVERTIMECNT  2          //设定超时时间间隔，单位100ms
#define CHECKOVERTIMENUM  1     //设定自动检测最大超时次数
#define SETOVERTIMENUM    2     //设定手动控制命令的最大超时次数



////////////////////////////////////////////新加 start///////////////////////////////////////////////////
typedef struct
{
	uint8_t fucNum;             //功能码
	uint8_t masterNum;          //主机编号
	uint8_t rxSlaveType;          //接收到的从机类型
	uint8_t slaveNum;           //从机编号
	uint8_t rxFrameCtntNum;       //接收数据内容的个数
	uint8_t rxFrameCtnt[40];     //接收提取完成数据内容
	uint8_t txFrameCtntNum;       //发送数据内容字节长度
	uint8_t txFrameCtnt[80];      //发送数据内容
//	uint16_t pressureReal;           //实时压力值
	uint16_t waterLevelReal;      //实时水位值，单位：0.01m
	uint16_t waterFlowReal;       //实时流量值，单位：0.1立方米
}uartCtntStruct;

extern uartCtntStruct uartData;


////////////////////////////////////////////新加 end////////////////////////////////////////////////////////////
void analyzeFrameContent(uint8_t * rxStr,uartCtntStruct * uartData);
e_state uartRcvContentHandle(char * rxStr,uint8_t length,uartCtntStruct * uartData);
void getDataToFrame(uint8_t *dest,uint8_t * destLength,uint8_t fctnNum,uint8_t masterNum,uint8_t slaveType,uint8_t slaveNum,uint8_t dataLength,uint8_t * dataCtnt);


#endif
