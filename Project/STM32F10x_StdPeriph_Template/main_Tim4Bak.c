/**
  ******************************************************************************
  * @file    TIM/TimeBase/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"//#include "stm32f10x_lib.h"
 
void RCC_cfg();
void GPIO_cfg();
void TIMER_cfg();
void PWM_cfg();
//占空比，取值范围为0-100
int dutyfactor = 10;
 
int main()
{
     int Temp;
       RCC_cfg();
       GPIO_cfg();
       TIMER_cfg();
       PWM_cfg();
 
       //使能TIM3计时器，开始输出PWM
       TIM_Cmd(TIM4, ENABLE);
 
       while(1);
}
 
void RCC_cfg()
{
       //定义错误状态变量
       //ErrorStatus HSEStartUpStatus;
      
       //将RCC寄存器重新设置为默认值
       //RCC_DeInit();
 
//       //打开外部高速时钟晶振
//       RCC_HSEConfig(RCC_HSE_ON);
// 
//       //等待外部高速时钟晶振工作
//       HSEStartUpStatus = RCC_WaitForHSEStartUp();
//       if(HSEStartUpStatus == SUCCESS)
//       {
//              //设置AHB时钟(HCLK)为系统时钟
//              RCC_HCLKConfig(RCC_SYSCLK_Div1);
// 
//              //设置高速AHB时钟(APB2)为HCLK时钟
//              RCC_PCLK2Config(RCC_HCLK_Div1);
// 
//              //设置低速AHB时钟(APB1)为HCLK的2分频
//              RCC_PCLK1Config(RCC_HCLK_Div2);
//             
//              //设置FLASH代码延时
//              FLASH_SetLatency(FLASH_Latency_2);
// 
//              //使能预取指缓存
//              FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
// 
//              //设置PLL时钟，为HSE的9倍频 8MHz * 9 = 72MHz
//              RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
// 
//              //使能PLL
//              RCC_PLLCmd(ENABLE);
// 
//              //等待PLL准备就绪
//              while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
// 
//              //设置PLL为系统时钟源
//              RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
// 
//              //判断PLL是否是系统时钟
//              while(RCC_GetSYSCLKSource() != 0x08);
//       }
 
       //开启TIM3的时钟
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
       //开启GPIOB的时钟和复用功能
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
 
}
 
void GPIO_cfg()
{
       GPIO_InitTypeDef GPIO_InitStructure;
 
      
       //部分映射，将TIM3_CH2映射到PB5
//     GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
       //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
 
       //选择引脚5
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
       //输出频率最大50MHz                                                        
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       //复用推挽输出                                              
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
 
       GPIO_Init(GPIOB,&GPIO_InitStructure);
}
 
void TIMER_cfg()
{
       TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 
       //重新将Timer设置为缺省值
       TIM_DeInit(TIM4);
       //采用内部时钟给TIM3提供时钟源
       TIM_InternalClockConfig(TIM4);
       //预分频系数为0，即不进行预分频，此时TIMER的频率为72MHz
       TIM_TimeBaseStructure.TIM_Prescaler = 0;
       //设置时钟分割
       TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
       //设置计数器模式为向上计数模式
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       //设置计数溢出大小，每计7200个数就产生一个更新事件，即PWM的输出频率为10kHz
       TIM_TimeBaseStructure.TIM_Period = 7200 - 1;
       //将配置应用到TIM3中
       TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
}
 
void PWM_cfg()
{
       TIM_OCInitTypeDef TimOCInitStructure;
       //设置缺省值
       TIM_OCStructInit(&TimOCInitStructure);
       //PWM模式1输出
       TimOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
       //设置占空比，占空比=(CCRx/ARR)*100%或(TIM_Pulse/TIM_Period)*100%
       TimOCInitStructure.TIM_Pulse = dutyfactor * 7200 / 100;
       //TIM输出比较极性高
       TimOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
       //使能输出状态
       TimOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
       //TIM3的CH2输出
       TIM_OC2Init(TIM4, &TimOCInitStructure);
       //设置TIM3的PWM输出为使能
       TIM_CtrlPWMOutputs(TIM4,ENABLE);
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
  {}
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
