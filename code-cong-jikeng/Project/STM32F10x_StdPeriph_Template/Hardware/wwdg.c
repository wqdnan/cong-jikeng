#include "wwdg.h"


uint32_t LsiFreq = 40000;
uint16_t CaptureNumber;

uint16_t IC1ReadValue1 = 0, IC1ReadValue2 = 0;
uint32_t Capture = 0;

/**
  * @brief  使用芯片内部的LSI (约为32kHz,配合TIM5-CH4引脚中断进行了频率修正)
  *         设定进行了250ms的长时间不喂狗则复位 使用的是独立开门狗
  * @param  None
  * @retval
  *
  *         超时计算为：value = 250ms / (LSI/32) (value为写入的值)
  *
  */
void wwdgInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);
	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	 /* Enable TIM5 clocks */
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	 /* Enable the TIM5 Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 /* Configure TIM5 prescaler */
	TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);
	/* Connect internally the TM5_CH4 Input Capture to the LSI clock output */
	GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI, ENABLE);
	/* TIM5 configuration: Input Capture mode ---------------------
	   The LSI oscillator is connected to TIM5 CH4
	   The Rising edge is used as active edge,
	   The TIM5 CCR4 is used to compute the frequency value
	------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	/* TIM10 Counter Enable */
	TIM_Cmd(TIM5, ENABLE);
	/* Reset the flags */
	TIM5->SR = 0;
	/* Enable the CC4 Interrupt Request */
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
	/* Disable TIM5 CC4 Interrupt Request */
	 TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
	  dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
	  Counter Reload Value = 250ms/IWDG counter clock period
						   = 250ms / (LSI/32)
						   = 0.25s / (LsiFreq/32)
						   = LsiFreq/(32 * 4)
						   = LsiFreq/128
	*/
	IWDG_SetReload(LsiFreq/128);
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
  {    
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC1ReadValue1 = TIM_GetCapture4(TIM5);
    }
    else if(CaptureNumber == 1)
    {
      /* Get the Input Capture value */
      IC1ReadValue2 = TIM_GetCapture4(TIM5); 
      
      /* Capture computation */
      if (IC1ReadValue2 > IC1ReadValue1)
      {
        Capture = (IC1ReadValue2 - IC1ReadValue1); 
      }
      else
      {
        Capture = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2); 
      }
      /* Frequency computation */ 
      LsiFreq = (uint32_t) SystemCoreClock / Capture;
      LsiFreq *= 8;
    }
    
    CaptureNumber++;
    
    /* Clear TIM5 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
  }
}
