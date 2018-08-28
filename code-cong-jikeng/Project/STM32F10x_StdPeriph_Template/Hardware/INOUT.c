#include "INOUT.h"
/**
******************************************************************************
* @file    INOUT.c
* @author  wqdnan
* @version V3.5.0
* @date    22-01-2018
* @brief   产生主循环的定时时间间隔，定时产生步进电机的脉冲信号，定时产生光电调制脉冲信号
******************************************************************************
* @attention
*
******************************************************************************
*/

#ifndef DEBUG
	uint8_t SLAVE_NUM = 0;
#endif

uint32_t pulse_cnt = 0;

/**
  * @brief  测试功能引脚初始化
  * @param  None
  * @retval None
  */
void pinTest(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//绑在单片机端的发光LED引脚
	RCC_APB2PeriphClockCmd(GPIO_CLK_TEST1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_TEST1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_TEST1, &GPIO_InitStructure);
	SET_TEST1();
}
/**
  * @brief  IO输入输出功能引脚初始化
  * @param  None
  * @retval None
  *
  */
void INOUTInit(void)
{

	GPIO_InitTypeDef   GPIO_InitStructure;

	//拨码开关S1~S4
	RCC_APB2PeriphClockCmd(GPIO_CLK_SLAVE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SLAVE_NUMBER1 | GPIO_PIN_SLAVE_NUMBER2 | GPIO_PIN_SLAVE_NUMBER3 | GPIO_PIN_SLAVE_NUMBER4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//上拉输入
	GPIO_Init(GPIO_SLAVE, &GPIO_InitStructure);
	//数字输出  测试LED-PA6 OUT1-PA7，OUT2-PC4，OUT3-PC5，OUT4-PB10，OUT5-PB11，OUT6-PB12
	RCC_APB2PeriphClockCmd(GPIO_CLK_CTRLA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED0 | GPIO_PIN_CTRL1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CTRLA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(GPIO_CLK_CTRLC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CTRL2 | GPIO_PIN_CTRL3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CTRLC, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(GPIO_CLK_CTRLB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CTRL4 | GPIO_PIN_CTRL5 | GPIO_PIN_CTRL6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CTRLB, &GPIO_InitStructure);

//	RCC_APB2PeriphClockCmd(GPIO_CLK_MC, ENABLE);
//	GPIO_InitStructure.GPIO_Pin =/* GPIO_PIN_MC1 | GPIO_PIN_MC2 | */GPIO_PIN_IN3 | GPIO_PIN_IN4;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
//	GPIO_Init(GPIO_MC, &GPIO_InitStructure);
}

//void mcExtiInit(FunctionalState flag)
//{
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	GPIO_InitTypeDef   GPIO_InitStructure;
//	NVIC_InitTypeDef   NVIC_InitStructure;

//	RCC_APB2PeriphClockCmd(GPIO_CLK_MC, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_MC1/* | GPIO_PIN_MC2*/;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
//	GPIO_Init(GPIO_MC, &GPIO_InitStructure);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_EXTILineConfig(GPIO_PortSource_MC, GPIO_PinSource_MC1);//启动MC1的外部中断

//	EXTI_InitStructure.EXTI_Line = EXTI_Line_MC1;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = flag;
//	EXTI_Init(&EXTI_InitStructure);

//	/* Enable and set EXTI9_5 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel_MC1;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = flag;

//	NVIC_Init(&NVIC_InitStructure);
//}
/**
  * @brief  初始化输出引脚，使其在上电不工作
  * @param  None
  * @retval None
  *
  */
void OUTReset(void)
{
	SET_Q4();
	SET_Q5();
	DRCNVALVE_OUTCLOSE();
	SET_Q6();
	DRCNVALVE_INCLOSE();
	MSTVALVE_CLOSE();
	LED_OFF();

}

void readINInfo(void)
{
	uint8_t inTemp[5] = {0};
//	inTemp[0] =  READ_MC1();
//	inTemp[1] =  READ_MC2();
//	inTemp[2] =  READ_IN3();
//	inTemp[3] =  READ_IN4();
	inTemp[0] =  GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER1);
	inTemp[1] =  GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER2);
	inTemp[2] =  GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER3);
	inTemp[3] =  GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER4);
	inTemp[4] = (inTemp[0]<<3)|(inTemp[1]<<2)|(inTemp[2]<<1)|(inTemp[3]);
}
/**
  * @brief  启动计算流量
  * @param  None
  * @retval None
  */
//void startFluxHandle(void)
//{
//	//打开脉冲引脚的外部中断，清零脉冲计数值
//	mcExtiInit(ENABLE);
//	pulse_cnt = 0;
//}
/**
  * @brief  停止计算流量
  * @param  None
  * @retval None
  */
//void endFluxHandle(void)
//{
//	mcExtiInit(DISABLE);
//}
/**
  * @brief  获取通过脉冲计算的流量值 ，后续带入公式
  * @param  None
  * @retval None
  */
//uint32_t getFluxData(void)
//{
//	return pulse_cnt;
//}

//void EXTI9_5_IRQHandler(void)
//{

//	if(EXTI_GetITStatus(EXTI_Line_MC1) != RESET)
//	{
//		TURN_TEST1();
//		pulse_cnt ++;
//		EXTI_ClearITPendingBit(EXTI_Line_MC1);
//	}

//}
