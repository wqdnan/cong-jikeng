#include "ADC.h"


/*
 * ADC.c
 *
 *  Created on: 2018-1-8
 *      Author: wqdnan
 *      模拟采样输入，配置为DMA接收，与外部接口为ADCConvertedValue 数组
 */

uint16_t ADCConvertedValue[ADCBUF_LENGTH] = {0};
/**
  * @brief  ADC外设初始化
  * @param  None
  * @retval None
  */
void ADCInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ADCCLK = PCLK2/2 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(FUC_CLK_ADC1 | GPIO_CLK_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ADC1_8;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ADC1_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIO_ADC1, &GPIO_InitStructure);

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADCBUF_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

/**
  * @brief  从外设获取到实时的压力值
  * @param  pdata-指向AD采样数组的首地址
  * @retval 返回实时的压力值，单位:MPa
  * 映射关系为：Y=Ax+B  单片机的取值为：x,压力值为：y
  */
float getPressureData(uint16_t *pdata)
{
	float result = 0;
	uint8_t i = 0;
	uint16_t temp1 = 0,temp2 = 0;
	for(i=0; i<ADCBUF_LENGTH; i++)
	{
		temp1 += pdata[i++];
		temp2 += pdata[i];
	}
	result = (float)(temp1*2.0/ADCBUF_LENGTH)/4096*3.3*PROPOTIONFACTOR_A + PROPOTIONFACTOR_B;
	return result;
}

