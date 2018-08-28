#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"


typedef enum
{

	enFlag = 0x35,
	rstFlag = 0x55

}e_state;

//#define WATER_DEPTH_CHECK 1  //水位监测
#define STRAIN_CHECK 3       //应力计检测

#define SLAVE_TYPE   STRAIN_CHECK//WATER_DEPTH_CHECK// ANGLE_CHECK//
//#define DEBUG//调试模式，从机ID固定

#ifdef DEBUG
	#define SLAVE_NUM 13//定义了从机号
#endif

//#define FLASH_WRITE//使用flash写入

#ifdef FLASH_WRITE

#define FLASH_ID 3

#endif

#include "ADC.h"
#include "DAC.h"
#include "INOUT.h"
#include "TIM.h"
#include "UART.h"
//#include "UARTHandle.h"
//#include "UARTControl.h"

#endif
