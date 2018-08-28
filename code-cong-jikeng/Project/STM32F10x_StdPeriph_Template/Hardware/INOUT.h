#ifndef __INOUT_H
#define __INOUT_H


#include "stm32f10x.h"

//output test
#define GPIO_PIN_TEST1 GPIO_Pin_13
#define	GPIO_CLK_TEST1 RCC_APB2Periph_GPIOC
#define	GPIO_TEST1 GPIOC

#define TURN_TEST1() GPIO_WriteBit(GPIO_TEST1, GPIO_PIN_TEST1, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIO_TEST1, GPIO_PIN_TEST1)))
#define SET_TEST1()  GPIO_SetBits(GPIO_TEST1,GPIO_PIN_TEST1)
#define RESET_TEST1() GPIO_ResetBits(GPIO_TEST1,GPIO_PIN_TEST1)


//input function 拨码开关S1~S4 对应PA0~PA3   脉冲输入 In_mc1-PC9,In_mc2-PC8,in3-PC7,in4-PC6
#define GPIO_PIN_SLAVE_NUMBER1 GPIO_Pin_0
#define GPIO_PIN_SLAVE_NUMBER2 GPIO_Pin_1
#define GPIO_PIN_SLAVE_NUMBER3 GPIO_Pin_2
#define GPIO_PIN_SLAVE_NUMBER4 GPIO_Pin_3
#define	GPIO_CLK_SLAVE RCC_APB2Periph_GPIOA
#define	GPIO_SLAVE GPIOA

#define READ_SLAVE_NUMBER() ((GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER1))<<3| \
                           ((GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER2))<<2)| \
                           ((GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER3))<<1)| \
                           (GPIO_ReadInputDataBit(GPIO_SLAVE, GPIO_PIN_SLAVE_NUMBER4)))

//激励端  OUT
//#define GPIO_PIN_MC1 GPIO_Pin_6
//#define GPIO_PIN_MC2 GPIO_Pin_8
////拾振端  IN
//#define GPIO_PIN_IN3 GPIO_Pin_7
//#define GPIO_PIN_IN4 GPIO_Pin_9
//#define	GPIO_CLK_MC RCC_APB2Periph_GPIOC
//#define	GPIO_MC GPIOC

//#define READ_MC1() GPIO_ReadInputDataBit(GPIO_MC, GPIO_PIN_MC1)
//#define READ_MC2() GPIO_ReadInputDataBit(GPIO_MC, GPIO_PIN_MC2)
//#define READ_IN3() GPIO_ReadInputDataBit(GPIO_MC, GPIO_PIN_IN3)
//#define READ_IN4() GPIO_ReadInputDataBit(GPIO_MC, GPIO_PIN_IN4)

//output function  数字输出  测试LED-PA6 OUT1-PA7，OUT2-PC4，OUT3-PC5，OUT4-PB10，OUT5-PB11，OUT6-PB12

#define GPIO_PIN_LED0 GPIO_Pin_6
#define GPIO_PIN_CTRL1 GPIO_Pin_7
#define	GPIO_CLK_CTRLA RCC_APB2Periph_GPIOA
#define	GPIO_CTRLA GPIOA

#define GPIO_PIN_CTRL2 GPIO_Pin_4
#define GPIO_PIN_CTRL3 GPIO_Pin_5
#define	GPIO_CLK_CTRLC RCC_APB2Periph_GPIOC
#define	GPIO_CTRLC GPIOC

#define GPIO_PIN_CTRL4 GPIO_Pin_10
#define GPIO_PIN_CTRL5 GPIO_Pin_11
#define GPIO_PIN_CTRL6 GPIO_Pin_12
#define	GPIO_CLK_CTRLB RCC_APB2Periph_GPIOB
#define	GPIO_CTRLB GPIOB

#define SET_Q5()  GPIO_SetBits(GPIO_CTRLA,GPIO_PIN_LED0)
#define RESET_Q5() GPIO_ResetBits(GPIO_CTRLA,GPIO_PIN_LED0)

#define SET_Q4()  GPIO_SetBits(GPIO_CTRLA,GPIO_PIN_CTRL1)
#define RESET_Q4() GPIO_ResetBits(GPIO_CTRLA,GPIO_PIN_CTRL1)

#define DRCNVALVE_OUTCLOSE()  GPIO_SetBits(GPIO_CTRLC,GPIO_PIN_CTRL2)//换向阀 出口   Q3
#define DRCNVALVE_OUT() GPIO_ResetBits(GPIO_CTRLC,GPIO_PIN_CTRL2)

#define SET_Q6()  GPIO_SetBits(GPIO_CTRLC,GPIO_PIN_CTRL3)
#define RESET_Q6() GPIO_ResetBits(GPIO_CTRLC,GPIO_PIN_CTRL3)
#define Q6_TURN() GPIO_WriteBit(GPIO_CTRLC, GPIO_PIN_CTRL3, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIO_CTRLC, GPIO_PIN_CTRL3)))

#define DRCNVALVE_INCLOSE()  GPIO_SetBits(GPIO_CTRLB,GPIO_PIN_CTRL4)//换向阀 进口    Q2
#define DRCNVALVE_IN() GPIO_ResetBits(GPIO_CTRLB,GPIO_PIN_CTRL4)

#define MSTVALVE_CLOSE()  GPIO_SetBits(GPIO_CTRLB,GPIO_PIN_CTRL5)//系统阀   Q1
#define MSTVALVE_OPEN() GPIO_ResetBits(GPIO_CTRLB,GPIO_PIN_CTRL5)

#define LED_OFF()  GPIO_SetBits(GPIO_CTRLB,GPIO_PIN_CTRL6)
#define LED_ON() GPIO_ResetBits(GPIO_CTRLB,GPIO_PIN_CTRL6)
#define LED_TURN() GPIO_WriteBit(GPIO_CTRLB, GPIO_PIN_CTRL6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIO_CTRLB, GPIO_PIN_CTRL6)))

#ifndef DEBUG
  extern uint8_t SLAVE_NUM;
#endif


void pinTest(void);
void INOUTInit(void);
//void mcExtiInit(FunctionalState flag);
void OUTReset(void);
void readINInfo(void);
//void startFluxHandle(void);
//void endFluxHandle(void);
//uint32_t getFluxData(void);
#endif
