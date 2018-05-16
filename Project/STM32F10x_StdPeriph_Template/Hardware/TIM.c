#include "TIM.h"
//#include "motor.h"
//#include "MotorHandle.h"

/**
******************************************************************************
* @file    TIM.c
* @author  wqdnan
* @version V3.5.0
* @date    28-09-2017
* @brief   产生主循环的定时时间间隔，定时产生步进电机的脉冲信号，定时产生光电调制脉冲信号
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/


TIM2_STATE mainT_state = {rstFlag,rstFlag,rstFlag};

volatile uint16_t tickTime_10ms = 0;

//////////////////fuction start///////////////////////////
FreqCheck_struct freq1Data =
{
	CHECK_FREQ_INIT,  //.sendFreq =
	E_CHECK_RST,      //.checkFlag =
	CHANNEL1,	      //.checkChannel =
	CHECK_FREQ_STEP,  //.actionStep
	CHECK_FREQ_INIT   //.actionFreqData
};
FreqCheck_struct freq2Data =
{
	CHECK_FREQ_INIT,   //.sendFreq =
	E_CHECK_RST,       //.checkFlag =
	CHANNEL2,          //.checkChannel =
	CHECK_FREQ_STEP,   //.actionStep
	CHECK_FREQ_INIT    //.actionFreqData
};
//////////////////fuction end///////////////////////////

/**
  * @brief  主定时器，产生最快为10ms的定时中断
  * @param  None
  * @retval None
  *
  */
void TIM2Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue =  0;
	
	RCC_PCLK1Config(RCC_HCLK_Div4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//使用第0组，组优先级最高
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//主优先级0，从优先级0，最高的优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 2000000) - 1;//设定计数单位频率1MHz
	TIM_TimeBaseStructure.TIM_Period = 65535;//该值没有实际作用
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM2_CCR1_VAL;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  设定TIM4作为控制采集应力计工作频率的控制定时器  定时间隔为1ms
  * @param  None
  * @retval None
  *
  */
void TIM4Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue =  0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//使用第0组，组优先级最高
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//主优先级0，从优先级0，最高的优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 2000000) - 1;//设定计数单位频率1MHz
	TIM_TimeBaseStructure.TIM_Period = 65535;//该值没有实际作用
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM4_CCR1_VAL;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief TIM3初始化,对应于激励电路的频率发生信号，对应通道是TIM3-CH1(PC6),TIM3_CH3(PC8)
  * @param  None
  * @retval None
  *
  */
void TIM3OUTInit(CheckChannelFlag_enum chnel,uint16_t fre1)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	uint16_t PrescalerValue =  0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);//TIM3-CH3如果完全重映射的对应PC8

	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;//12MHz的频率
	/* Time base configuration */
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 12000000/fre1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 6000000/fre1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	if(chnel == CHANNEL1)
	{
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	}
	else
	{
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	}
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
/**
  * @brief  停止TIM3对应通道的PWM输出，TIM3-CH1(PC6),TIM3_CH3(PC8)
  * @param  None
  * @retval None
  */
void ChannelOUTStop(CheckChannelFlag_enum chnel)
{
	if(chnel == CHANNEL1)
		TIM_ForcedOC1Config(TIM3, TIM_ForcedAction_InActive);
	else
		TIM_ForcedOC3Config(TIM3, TIM_ForcedAction_InActive);
}

/**
  * @brief TIM8初始化,对应于拾振电路的TIM8捕获功能，对应通道是TIM8_CH2(PC7) TIM8-CH4(PC9)
  * @param  None
  * @retval None
  * 设定为捕获通道，捕获下降沿  设定在需要捕获时，开启通道，同时只能有一个通道在工作
  */
void TIM8INInit(CheckChannelFlag_enum chnel)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable the TIM3 global Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//NVIC_PriorityGroup_0 优先级组最高
	NVIC_InitStructure.NVIC_IRQChannel =  TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//设定最高的优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIM8);

	TIM_TimeBaseStructure.TIM_Period = 65536-1;//必须配置
	TIM_TimeBaseStructure.TIM_Prescaler=4-1;//4分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV2;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	if(chnel == CHANNEL1)
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//配置输入分频,分频
		TIM_ICInitStructure.TIM_ICFilter = 0x05;//0x0;//配置输入滤波器 不滤波
		TIM_ICInit(TIM8, &TIM_ICInitStructure);
		TIM_Cmd(TIM8, ENABLE);
		TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
	}
	else
	{
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//配置输入分频,不分频
		TIM_ICInitStructure.TIM_ICFilter = 0x05;//0x0;//配置输入滤波器 不滤波
		TIM_ICInit(TIM8, &TIM_ICInitStructure);
		TIM_Cmd(TIM8, ENABLE);
		TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
	}



}
/**
  * @brief 对应于拾振电路的TIM8捕获功能的关闭，对应通道是TIM8_CH2(PC7) TIM8-CH4(PC9)
  * @param  None
  * @retval None
  * 设定在需要捕获时，开启通道，同时只能有一个通道在工作
  */
void channelCaptureCtrl(CheckChannelFlag_enum chnel,FunctionalState flag)
{
	if(chnel == CHANNEL1)
		TIM_ITConfig(TIM8, TIM_IT_CC2, flag);
	else
		TIM_ITConfig(TIM8, TIM_IT_CC4, flag);
}


/**
  * @brief 和TIM2定时配合使用，计数变化间隔为10ms
  * @param  None
  * @retval None
  * tickTime_10ms为全局易变 变量
  */
void myDelay_ms(uint16_t cnt)
{
	uint16_t tick = tickTime_10ms;
	while(1)
	{
		if((tickTime_10ms-tick)>cnt)
			break;
	}
}
/**
  * @brief  TIM2对应的中断函数，用于产生定时中断，定时为10ms，100ms,1s
  * @param  None
  * @retval None
  * 通过修改 TIM2_CCR1_VAL、TIM2_CNT_100MS、TIM2_CNT_1S，可以改变定时间隔
  */
void TIM2_IRQHandler(void)
{
	static uint16_t capture = 0;
	static uint16_t timerNum_1S = 0;
	static uint16_t timerNum_100MS = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		//GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12)));
		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + TIM2_CCR1_VAL);
		//功能函数添加
		mainT_state.t10ms_flag = enFlag;
		timerNum_100MS ++;
		timerNum_1S ++;
		if(timerNum_100MS >= TIM2_CNT_100MS)
		{
			timerNum_100MS = 0;
			mainT_state.t100ms_flag = enFlag;
		}
		if(timerNum_1S >= TIM2_CNT_1S)
		{
			timerNum_1S = 0;
			mainT_state.t1s_flag = enFlag;
		}
		tickTime_10ms ++;//使用来做延时 在delay_ms()中使用
	}

}

/**
  * @brief  TIM4对应的中断函数，在中断中对应力计获取频率功能步骤进行控制
  * @param  None
  * @retval None
  * TIM4_CCR1_VAL = 1000,中断间隔为1ms
  */
void TIM4_IRQHandler(void)
{
	uint16_t capture = 0;
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM4);
		TIM_SetCompare1(TIM4, capture + TIM4_CCR1_VAL);
		//功能函数添加
//		TURN_TEST1();
		if(freq1Data.checkFlag != E_CHECK_RST)//channel1
			channelFreqCheck(&freq1Data);
		if(freq2Data.checkFlag != E_CHECK_RST)
			channelFreqCheck(&freq2Data);
	}

}
/**
  * @brief  TIM3对应的中断函数，用于捕获TIM3-ch4(PC9)的周期频率 ，进入中断为上升沿电平
  * @param  None
  * @retval None
  * 为了提高频率精度，将值修改为float类型
  */

void TIM3_IRQHandler(void)
{
	static uint16_t IC3ReadValue21 = 0, IC3ReadValue22 = 0;
	static uint16_t CaptureNumber2 = 0;
	static uint8_t freTempBias = 0;
	uint32_t Capture = 0;
	float TIM3Freq2 = 0;
  if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET)
  {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		if(CaptureNumber2 == 1)
		{
			IC3ReadValue22 = TIM_GetCapture4(TIM3);
			if (IC3ReadValue22 > IC3ReadValue21)
			{
				Capture = (IC3ReadValue22 - IC3ReadValue21);
			}
			else
			{
				Capture = ((0xFFFF - IC3ReadValue21) + IC3ReadValue22);
			}
			TIM3Freq2 =  SystemCoreClock * 0.5 / Capture ;
			freq2Data.captureCnt[freTempBias] = IC3ReadValue22;
			freq2Data.captureFreq[freTempBias++] = TIM3Freq2;
			if(freTempBias >= CAPTURE_NUM)
				freTempBias = 0;
			IC3ReadValue21 = IC3ReadValue22;
		}
		else if(CaptureNumber2 == 0)
		{
			IC3ReadValue21 = TIM_GetCounter(TIM2);
			CaptureNumber2 = 1;
		}
  }

}
/**
  * @brief  TIM8对应的中断函数，用于捕获TIM8-ch2(PC7)、TIM8-ch4(PC9)的周期频率 ，进入中断为上升沿电平
  * @param  None
  * @retval None
  * 修改TIM8专门做捕获，包括TIM8-ch2,TIM8-ch4 捕获的频率为72MHz 。
  */
void TIM8_CC_IRQHandler(void)
{
	static uint16_t IC3ReadValue11 = 0, IC3ReadValue12 = 0,IC3ReadValue21 = 0, IC3ReadValue22 = 0;
	static uint16_t CaptureNumber1 = 0,CaptureNumber2 = 0;
	static uint8_t freTempBias1 = 0,freTempBias2 = 0;
	float TIM8Freq1 = 0;
	float TIM8Freq2 = 0;
	uint32_t Capture = 0;
	if(TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
		if(CaptureNumber1 == 1)
		{
			IC3ReadValue12 = TIM_GetCapture2(TIM8);
			if (IC3ReadValue12 > IC3ReadValue11)
			{
				Capture = (IC3ReadValue12 - IC3ReadValue11);
			}
			else
			{
				Capture = ((0xFFFF - IC3ReadValue11) + IC3ReadValue12);
			}
			TIM8Freq1 = SystemCoreClock*0.25 / Capture;//时钟4分频了
			freq1Data.captureCnt[freTempBias1] = IC3ReadValue12;
			freq1Data.captureFreq[freTempBias1++] = TIM8Freq1;
			if(freTempBias1 >= CAPTURE_NUM)
				freTempBias1 = 0;
			IC3ReadValue11 = IC3ReadValue12;
		}
		else if(CaptureNumber1 == 0)
		{
			IC3ReadValue11 = TIM_GetCapture2(TIM8);
			CaptureNumber1 = 1;
		}
	}
	if(TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
		if(CaptureNumber2 == 1)
		{
			IC3ReadValue22 = TIM_GetCapture4(TIM8);
			if (IC3ReadValue22 > IC3ReadValue21)
			{
				Capture = (IC3ReadValue22 - IC3ReadValue21);
			}
			else
			{
				Capture = ((0xFFFF - IC3ReadValue21) + IC3ReadValue22);
			}
			TIM8Freq2 = SystemCoreClock*0.25 / Capture;//时钟4分频了
			freq2Data.captureCnt[freTempBias2] = IC3ReadValue22;
			freq2Data.captureFreq[freTempBias2++] = TIM8Freq2;
			if(freTempBias2 >= CAPTURE_NUM)
				freTempBias2 = 0;
			IC3ReadValue21 = IC3ReadValue22;
		}
		else if(CaptureNumber2 == 0)
		{
			IC3ReadValue21 = TIM_GetCapture4(TIM8);
			CaptureNumber2 = 1;
		}
	}
}

//////////////////fuction/////////////////////////////////
/**
  * @brief  应力计检测频率的功能函数的启动和停止，主要用于启动TIM4中断函数
  * @param  flag ENABLE-启动  DISABLE-停止
  * @retval None
  *
  */
void channelFreqChckCtrl(FreqCheck_struct * freqData,FunctionalState flag)
{
	if(flag == ENABLE)
	{
		freqData->checkFlag = E_CHECK_TEST;
	}
	else
	{
		freqData->checkFlag = E_CHECK_RST;
	}
}
/**
  * @brief  应力计检测频率的的方差计算，并返回是否满足要求
  * @param  freqData-数值结构体  averageValue-平均值
  * @retval None  enFlag-计算方差满足要求，rstFlag-计算方差不满足要求
  * 在计算时剔除明显不满足要求的数值，不计入到计算中
  * 使用的公式为 y^2 = ((x1-x平均值)^2+(x2-x平均值)^2+……+(xn-x平均值)^2)/(n-1)
  */
e_state freqErrorBiasCaculate(FreqCheck_struct * freqData,float averageValue)
{
	e_state result = rstFlag;
	uint8_t i = 0;
	uint8_t checkValidCnt = CAPTURE_NUM;
	float caculateNum = 0;
	static const float ERRORBIAS = 10000;//1000;//400;//100;//50;//30;//20;方差过小了后会出现测不到的现象
	for(i=0;i<CAPTURE_NUM;i++)
	{
		if((freqData->captureFreq[i]>(freqData->sendFreq-FREQ_BIAS_RANGE)) \
		 &&(freqData->captureFreq[i]<(freqData->sendFreq+FREQ_BIAS_RANGE)))//
			caculateNum += ((freqData->captureFreq[i]-averageValue)*(freqData->captureFreq[i]-averageValue));
		else
			checkValidCnt --;
	}
	if(checkValidCnt >= 2)
	{
		caculateNum /= (checkValidCnt-1);
		if(caculateNum < ERRORBIAS)
		{
			result = enFlag;
		}
	}
	return result;
}
/**
  * @brief  计算测量到的频率值的平均值，并返回
  * @param  freq-测量到的频率值  cnt-有效的频率值个数
  * @retval None  返回计算到的频率的平均值
  */
float freqAverCacultate(float * freq,uint8_t cnt)
{
	uint8_t i = 0;
	float result = 0;
	if(cnt>0)
	{
		for(i=0;i<cnt;i++)
		{
			result += freq[i];
		}
		result /= cnt;
	}
	return result;

}
/**
  * @brief  应力计检测频率的功能函数，包括了全部功能，固定时间间隔调用
  * @param  None
  * @retval None
  * 现在为连续地获取到频率值，中间不间隔
  */
void channelFreqCheck(FreqCheck_struct * freqData)
{
	uint8_t i = 0;
	float captureFreTemp = 0;
	uint8_t checkValidCnt = 0;
	e_state flag = rstFlag;
	static uint16_t checkTestCnt = 0;
	static e_state checkTestFlag = rstFlag;
	static float captureFreAver[CHECK_FREQ_BIAS] = {0};
	static uint8_t captureFreAverNum = 0;
	static uint8_t freqErrorCnt = 0;//记录频率计算错误的次数，到连续超过三次则需要进行频率修改操作
	static int8_t freqBias = 1;//用于计算扫频的实际步长  有正负 负数频率减小扫频，正数频率增大扫频
	if(freqData->checkFlag != E_CHECK_RST)
	{
		freqData->checkCnt ++;
		if(freqData->checkCnt == CHECK_SEND_TIME)
		{
			//启动发送频率
			if(freqData->checkChannel == CHANNEL1)//通道1，则启动TIM8
			{
				TIM3OUTInit(CHANNEL1,freqData->sendFreq);//TIM3-CH1(PC6)
			}
			else//通道2 ，则启动TIM3
			{
				TIM3OUTInit(CHANNEL2,freqData->sendFreq);//TIM3_CH3(PC8)
			}
		}
		else if(freqData->checkCnt == CHECK_CAPTURE_TIME)
		{
			//停止发送频率，并启动捕获频率
			if(freqData->checkChannel == CHANNEL1)
			{
				ChannelOUTStop(CHANNEL1);//TIM3-CH1(PC6)
			}
			else//通道2 ，则启动TIM3
			{
				ChannelOUTStop(CHANNEL2);//TIM3_CH3(PC8)
			}
		}
		else if(freqData->checkCnt == (CHECK_CAPTURE_TIME+5))
		{
			for(i=0;i<CAPTURE_NUM;i++)
				freqData->captureFreq[i] = 0;
			if(freqData->checkChannel == CHANNEL1)
			{
				TIM8INInit(CHANNEL1);//TIM8_CH2(PC7)
				TIM8INInit(CHANNEL1);//有时候发现打不开
			}
			else//通道2 ，则启动TIM3
			{
				TIM8INInit(CHANNEL2);//TIM8-CH4(PC9)
				TIM8INInit(CHANNEL2);
			}
		}
		else if(freqData->checkCnt == CHECK_CALCULATE_TIME)
		{
			//计算，并且判断是扫频粗侧还是复测
			freqData->checkCnt = 245;//为后面连续检测做准备
			if(freqData->checkChannel == CHANNEL1)//通道1，则关闭TIM8
				channelCaptureCtrl(CHANNEL1,DISABLE);//TIM8_CH2(PC7)
			else
				channelCaptureCtrl(CHANNEL2,DISABLE);//TIM8-CH4(PC9)
			checkValidCnt = CAPTURE_NUM;
			for(i=0;i<CAPTURE_NUM;i++)
			{
				if((freqData->captureFreq[i]>(FREQ_MIN)) \
				 &&(freqData->captureFreq[i]<(FREQ_MAX)))//freqData->sendFreq+FREQ_BIAS_RANGE
					captureFreTemp+= freqData->captureFreq[i];
				else
					checkValidCnt --;
			}
			if(checkValidCnt>0)
				captureFreTemp /= checkValidCnt;
			else
				captureFreTemp = 0;
			if(freqData->checkFlag == E_CHECK_TEST)//粗侧
			{
				//判断是否合理 条件1 频率的平均值  条件2 频率的方差值
				if((captureFreTemp > FREQ_MIN)&&(captureFreTemp < FREQ_MAX))
				{
					//判断方差是否对
					flag = freqErrorBiasCaculate(freqData,captureFreTemp);
					if(flag == enFlag)//方差合理，则进入复测模式,并计算新频率
					{
						captureFreAver[captureFreAverNum++] = captureFreTemp;//记录合法的测量值
						checkTestCnt ++;
						freqErrorCnt = 0;//频率计算错误清零
					}
					else//方差不合理，则进入到粗侧模式，并修改频率值
					{
						checkTestFlag = enFlag;
					}
				}
				else
				{
					checkTestFlag = enFlag;
				}
				if(checkTestFlag == enFlag)//启动扫频还是平均数带入 新的频率值
				{
					checkTestFlag = rstFlag;
					checkTestCnt ++;
					freqErrorCnt ++;
					if(freqErrorCnt >= 3)//同一频率尝试三次都不满足条件，则修改频率
					{
						freqErrorCnt = 0;
						if(captureFreAverNum >= 3)//使用平均值带入扫频
						{
							captureFreTemp = freqAverCacultate(&captureFreAver[0],captureFreAverNum);
							if(captureFreTemp > 0)
							{
								freqData->sendFreq = captureFreTemp;
							}
							else//扫频
							{
								checkTestFlag = enFlag;
							}
						}
						else//扫频
						{
							checkTestFlag = enFlag;
						}
						if(checkTestFlag == enFlag)//启动扫频
						{
							checkTestFlag = rstFlag;
							freqData->sendFreq += (freqBias * (freqData->actionStep));//左右两侧来回扫频
							if(freqBias>0)
								freqBias = (freqBias+1)*(-1);
							else
								freqBias = (0-freqBias) + 1;
							if(freqData->sendFreq>FREQ_MAX)
								freqData->sendFreq = FREQ_MAX;
							else if(freqData->sendFreq<FREQ_MIN)
								freqData->sendFreq = FREQ_MIN;

						}
					}
					freqData->checkFlag = E_CHECK_TEST;
				}
				if(checkTestCnt > CHECK_FREQ_BIAS)//超过了设定次数，则结束，并判断是否测量出频率值
				{
					captureFreTemp = freqAverCacultate(&captureFreAver[0],captureFreAverNum);
					if(captureFreTemp > 0)
					{
						freqData->actionFreqData = captureFreTemp;
						freqData->sendFreq = captureFreTemp;
						freqData->calculateFreq = captureFreTemp;
					}
					else//没有测量的频率值
					{
						//freqData->calculateFreq = 0;  //没有测到值，则保持上次的值不变
						freqData->sendFreq = freqData->actionFreqData;
						freqData->actionStep += CHECK_FREQ_STEP;
						if(freqData->actionStep > 250)//(2800-600)/25  (FREQ_MAX-FREQ_MIN)/CHECK_FREQ_BIAS/2
							freqData->actionStep = CHECK_FREQ_STEP;
					}
					freqData->checkFlag = E_CHECK_RST;
					channelFreqChckCtrl(freqData,DISABLE);
					checkTestCnt = 0;
					freqData->checkCnt = 0;
					freqBias = 1;
					for(i=0;i<captureFreAverNum;i++)
						captureFreAver[i] = 0;
					captureFreAverNum = 0;
				}
			}
		}
	}
}



