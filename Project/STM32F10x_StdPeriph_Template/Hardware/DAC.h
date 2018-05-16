/*
 * DAC.h
 *
 *  Created on: 2018-1-8
 *      Author: wqdnan
 */

#ifndef DAC_H_
#define DAC_H_

#include "stm32f10x.h"

#define GPIO_PIN_DAC1 GPIO_Pin_4
#define GPIO_PIN_DAC2 GPIO_Pin_5
#define	GPIO_CLK_DAC RCC_APB2Periph_GPIOA
#define FUC_CLK_DAC RCC_APB1Periph_DAC
#define	GPIO_DAC GPIOA

void DACInit(void);
void outputVoltage(float voltage,uint8_t channel);

#endif /* DAC_H_ */
