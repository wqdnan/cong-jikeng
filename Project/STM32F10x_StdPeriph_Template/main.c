/**
  ******************************************************************************
  * @file    main.c
  * @author  wqdnan
  * @version V3.5.0
  * @date    29-9-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * 初始化硬件外设，主循环中定期检测标志位并处理事务
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FuctionControl.h"
#include "UARTHandle.h"
#include "UARTControl.h"
#include "flash.h"



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  

uint8_t temp = 0;

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
float data = 0;
int main(void)
{
	e_state fucFlag = rstFlag;
	e_state slaveFucFlag = enFlag;
	TIM2Init();
#ifdef STRAIN_CHECK
  TIM4Init();
#elif  WATER_DEPTH_CHECK
  ADCInit();
#elif  ANGLE_CHECK

#endif
  UART1Init();
//  UART3Init();
  pinTest();
  INOUTInit();
	OUTReset();
//  DACInit();



#ifndef DEBUG
  SLAVE_NUM = READ_SLAVE_NUMBER();
  if(SLAVE_NUM == 0)
	  SLAVE_NUM = 25;
#endif
  
  while (1)
  {
	  if(mainT_state.t10ms_flag == enFlag)
	  {
		  mainT_state.t10ms_flag = rstFlag;
		  if(rx1Flag == enFlag)
		  {
			  rx1Flag = rstFlag;
			  fucFlag = UartRcvHandle(&rxBuf1[0],rxLength1,&uartData,&commData);
			  if(fucFlag == enFlag)//需要回复
			  {
				  UartTxHandle(USART1,&txBuf1[0],&uartData,&commData);
			  }
		  }
	  }
	  if(mainT_state.t100ms_flag == enFlag)
	  {
		  mainT_state.t100ms_flag = rstFlag;
		  regularTimeDo(&uartData,&ctrlData);
	  }
	  if(mainT_state.t1s_flag == enFlag)
	  {
		  mainT_state.t1s_flag = rstFlag;
		  if(slaveFucFlag == enFlag)
			  Q6_TURN();

	  }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
