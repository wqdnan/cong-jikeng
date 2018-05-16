#include "FuctionControl.h"


fctnCtrlStruct ctrlData;


/**
  * @brief  按照设定时间间隔执行相应
  * @param  ctrlComm-从机通信数据交互结构体
  * @retval None
  * 格式：帧头|功能码|主机编号|从机编号|数据长度|数据内容|校验和|帧尾
  * 字节数    1    1     1       1       1      不定        1     1
  */
e_state regularFctnHandle(uartCtntStruct * uartData,fctnCtrlStruct * ctrlData)
{
	return rstFlag;
}
/**
  * @brief  每隔相同时间则检查一次固定时间操作行为是否需要执行
  * @param  ctrlComm-从机通信数据交互结构体
  * @retval None
  * 格式：帧头|功能码|主机编号|从机编号|数据长度|数据内容|校验和|帧尾
  * 字节数    1    1     1       1       1      不定        1     1
  */
void regularTimeDo(uartCtntStruct * uartData,fctnCtrlStruct * ctrlData)
{
	static uint8_t cnt = 0;
	static const uint8_t MAXCNTNUM = 30;
	e_state fucFlag = rstFlag;
	static uint8_t flag = 0;
	if(cnt < MAXCNTNUM)
	{
		cnt ++;
		if(cnt == MAXCNTNUM)
		{
			cnt = 0;
			fucFlag = enFlag;
		}
	}
	if(fucFlag == enFlag)
	{
#ifdef STRAIN_CHECK
	flag = 1-flag;
	if(flag == 1)
		channelFreqChckCtrl(&freq1Data,ENABLE);
	else
		channelFreqChckCtrl(&freq2Data,ENABLE);
#elif  WATER_DEPTH_CHECK
    uartData->waterLevelReal = (uint16_t)(getPressureData(&ADCConvertedValue[0]) * 100);//单位：0.01m
	//uartData->waterFlowReal  = (uint16_t)getFluxData();// * 0.001;//单位：0.1立方米  10脉冲=0.001立方米
#elif  ANGLE_CHECK

#endif

		fucFlag = rstFlag;
		//执行操作
	}
}
