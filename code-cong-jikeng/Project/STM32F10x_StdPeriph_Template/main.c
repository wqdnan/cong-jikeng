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
#include "wwdg.h"


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  

uint8_t temp = 0;

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint16_t data  = 0;
	e_state fucFlag = rstFlag;
	e_state slaveFucFlag = enFlag;
	TIM2Init();

  UART1Init();
  pinTest();
  INOUTInit();
  OUTReset();
////  DACInit();
#ifdef STRAIN_CHECK
  TIM4Init();
#elif  WATER_DEPTH_CHECK
  ADCInit();
#elif  ANGLE_CHECK
  UART4Init();
#endif
  RESET_Q5();
  myDelay_ms(5);
  SET_Q5();
  
#ifndef DEBUG
//  SLAVE_NUM = READ_SLAVE_NUMBER();
  if(flashHandle(&data,0)==enFlag)//从flash中读取数据
  {
	 SLAVE_NUM = (uint8_t)data;
  }
  if(SLAVE_NUM == 0)
	  SLAVE_NUM = 25;
#endif
//  wwdgInit();
  while (1)
  {
//		myDelay_ms(20);
//		IWDG_ReloadCounter();
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
//		  SendStr(SLAVE_UART,&txBuf1[0],6);

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
