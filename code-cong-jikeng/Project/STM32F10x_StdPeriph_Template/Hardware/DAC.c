#include "DAC.h"

/*
 * DAC.c
 *
 *  Created on: 2018-1-8
 *      Author: wqdnan
 */


/**
  * @brief  DAC外设初始化
  * @param  None
  * @retval None
  */
void DACInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;

	RCC_APB2PeriphClockCmd(GPIO_CLK_DAC, ENABLE);
	RCC_APB1PeriphClockCmd(FUC_CLK_DAC, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_PIN_DAC1 | GPIO_PIN_DAC2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIO_DAC, &GPIO_InitStructure);

	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;//DAC_Trigger_None
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;//DAC_TriangleAmplitude_4095
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
}

/**
  * @brief  设定模拟通道输出电压值
  * @param  voltage-设定的输出电压值，在0~3.3V之间
  *         channel- 1 通道1   2 通道2
  * @retval None
  */
void outputVoltage(float voltage,uint8_t channel)
{
	float temp = voltage/3.05;
	uint16_t data = 0;
	if(temp > 3.3)
		temp = 3.3;
	else if(temp < 0)
		temp = 0;
	data = (uint16_t)((temp/3.3)*4096);
	//data = 4000;
	if(channel == 1)
	{
		DAC_SetChannel1Data(DAC_Align_12b_R,data);
		DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
	}
	else
	{
		DAC_SetChannel2Data(DAC_Align_12b_R,data);
		DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
	}

}
