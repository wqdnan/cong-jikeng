/*
 * ADC.h
 *
 *  Created on: 2018-1-8
 *      Author: wqdnan
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"


#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADCBUF_LENGTH    10

#define GPIO_PIN_ADC1_8 GPIO_Pin_0
#define GPIO_PIN_ADC1_9 GPIO_Pin_1
#define	GPIO_CLK_ADC1 RCC_APB2Periph_GPIOB
#define FUC_CLK_ADC1  RCC_APB2Periph_ADC1
#define	GPIO_ADC1     GPIOB

#define PROPOTIONFACTOR_A 3.0625   //校正输入电压值与实际压力值的关系 比例数值A
#define PROPOTIONFACTOR_B -2       //校正输入电压值与实际压力值的关系 比例数值B

#define AMP_MULTIPLE  1.3    //用于抵消掉硬件运算的放大比例，使得单片机输出的电压值和实际硬件输出的电压值一致

extern uint16_t ADCConvertedValue[ADCBUF_LENGTH];

void ADCInit(void);
float getPressureData(uint16_t *pdata);

#endif /* ADC_H_ */
