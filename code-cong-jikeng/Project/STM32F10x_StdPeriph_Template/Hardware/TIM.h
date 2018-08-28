#ifndef __TIM_H
#define __TIM_H



#include "stm32f10x.h"
#include "main.h"


#define TIM2_CCR1_VAL 10000 //定时单位为1us，定时长度为10ms
#define TIM2_CNT_100MS 10   //TIM2定时100ms所需时间
#define TIM2_CNT_1S 100		//TIM2定时1s所需时间

#define TIM4_CCR1_VAL 1000 //定时单位为1us，定时长度为1ms

typedef  struct
{
	e_state t10ms_flag;
	e_state t100ms_flag;
	e_state t1s_flag;
}TIM2_STATE;

/////////////////fuction add////////////////////////////

#define CAPTURE_NUM 20
#define CHECK_FREQ_INIT  2000   //初始频率
#define CHECK_FREQ_STEP  30      //初测扫频步长
#define FREQ_MIN  600           //工作有效频率的最小值
#define FREQ_MAX  2800          //工作有效频率的最大值
#define CHECK_FREQ_BIAS  45    //检测频率范围 CHECK_FREQ_INIT-0.5*CHECK_FREQ_BIAS*CHECK_FREQ_STEP
                               //         ~ CHECK_FREQ_INIT+0.5*CHECK_FREQ_BIAS*CHECK_FREQ_STEP
#define FREQ_BIAS_RANGE  500   //设定的检测到的频率范围
#define CHECK_FREQ_MIN  (CHECK_FREQ_INIT-400)  //检测到的频率有效最小值
#define CHECK_FREQ_MAX  (CHECK_FREQ_INIT+400)  //检测到的频率有效最大值
#define CHECK_SEND_TIME 1              //传感器频率发送通道开启时间计数
#define CHECK_CAPTURE_TIME 20          //传感器频率发送通道关闭时间计数
#define CHECK_CALCULATE_TIME 50        //传感器频率捕获通道关闭时间计数  ，默认捕获通道开启时间计数为：CHECK_CAPTURE_TIME+10

#define _TEST_CHANNEL2
typedef enum
{
	E_CHECK_RST,     //检测初始化
	E_CHECK_TEST,    //粗测（精度不高）
	E_CHECK_REAL     //复测（精度高）
}CheckFlag_enum;

typedef enum
{
	CHANNEL1,
	CHANNEL2
}CheckChannelFlag_enum;

typedef struct
{
	uint16_t sendFreq;               //发送的频率，单位Hz
	CheckFlag_enum checkFlag;        //当前工作的状态指示，用于区分粗侧和复测
	CheckChannelFlag_enum checkChannel;   //当前结构体对应的通道
	uint16_t actionStep;             //实际工作的扫频步长
	uint16_t actionFreqData;         //记录上一次的初始扫频频率值
	uint8_t sweepFreqNum;            //扫频超过满步长次数 ，最大为5
	uint8_t checkCnt;                //记录每个步骤的时间间隔 用于指示每个功能流程的时间长度
	float captureFreq[CAPTURE_NUM];  //捕获的频率 ，单位Hz
	float captureCnt[CAPTURE_NUM];
	uint16_t calculateFreq;          //计算的频率，单位Hz


}FreqCheck_struct;

extern FreqCheck_struct freq1Data;
extern FreqCheck_struct freq2Data;

/////////////////fuction end///////////////////////////

extern TIM2_STATE mainT_state;
extern volatile uint16_t tickTime_10ms;

void TIM2Init(void);
void TIM4Init(void);

void TIM3OUTInit(CheckChannelFlag_enum chnel,uint16_t fre1);
void ChannelOUTStop(CheckChannelFlag_enum chnel);
void TIM8INInit(CheckChannelFlag_enum chnel);
void channelCaptureCtrl(CheckChannelFlag_enum chnel,FunctionalState flag);

void myDelay_ms(uint16_t cnt);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM8_CC_IRQHandler(void);

e_state freqErrorBiasCaculate(FreqCheck_struct * freqData,float averageValue);
float freqAverCacultate(float * freq,uint8_t cnt);
void channelFreqChckCtrl(FreqCheck_struct * freqData,FunctionalState flag);
void channelFreqCheck(FreqCheck_struct * freqData);



#endif
