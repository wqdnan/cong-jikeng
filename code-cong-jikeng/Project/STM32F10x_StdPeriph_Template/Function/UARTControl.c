#include "UARTControl.h"

/**
******************************************************************************
* @file    UARTHandle.c
* @author  wqdnan
* @version V3.5.0
* @date    26-12-2017
* @brief   完成主机与从机的通信数据交互，和主机与上位机的通信交互
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/


commCtntStruct commData;

/**
  * @brief  对数据帧中提取的数据内容，进行处理，并根据相应内容决定是否要回复
  * @param  uartData-数据帧内容结构体
  *         commData-通信数据内容结构体（需要用户自定义）
  * @retval enFlag-需要回复  rstFlag-不需要回复
  * 注：该函数需要根据实际功能进行重写
  */
e_state UartRxFctnCtntHandle(uartCtntStruct * uartData,commCtntStruct * commData)
{
	e_state result = rstFlag;//不需要回复

	switch(uartData->fucNum)
	{
		case 0x0D:
		{
			result = setParaData(uartData,commData);
			break;
		}
		case 0x77://ID 写入帧格式
		case 0x78://ID 读出帧格式
		{
			result = setFlashData(uartData);
			break;
		}
//		case 0x21:case 0x22:
//		{
//			result = setFlashData(uartData);
//			break;
//		}
	}
	return result;
}
/**
  * @brief  对待发送的数据帧的数据内容进行准备，并反馈是否准备好
  * @param  uartData-待写入的数据帧内容结构体
  *         commData-通信数据内容结构体（需要用户自定义）
  * @retval enFlag-数据帧准备好  rstFlag-数据帧未准备好，不做发送操作
  * 注：该函数需要根据实际功能进行重写
  */
e_state UartTxFctnCtntHandle(uartCtntStruct * uartData,commCtntStruct * commData)
{
	e_state result = rstFlag;
	uint8_t i = 0;
	uint8_t bias = 0;
	//根据标志位进行数据帧准备
	switch(uartData->fucNum)
	{
		case 0x0D:
		{
#ifdef STRAIN_CHECK
			uartData->txFrameCtnt[bias] =    (freq1Data.calculateFreq/1000%10)<<4;//单位：0.01m
			uartData->txFrameCtnt[bias++] += (freq1Data.calculateFreq/100%10);
			uartData->txFrameCtnt[bias] =    (freq1Data.calculateFreq/10%10)<<4;
			uartData->txFrameCtnt[bias++] += (freq1Data.calculateFreq%10);
			uartData->txFrameCtnt[bias] =    (freq2Data.calculateFreq/1000%10)<<4;//单位：0.1立方米
			uartData->txFrameCtnt[bias++] += (freq2Data.calculateFreq/100%10);
			uartData->txFrameCtnt[bias] =    (freq2Data.calculateFreq/10%10)<<4;
			uartData->txFrameCtnt[bias++] += (freq2Data.calculateFreq%10);
#elif  WATER_DEPTH_CHECK
			uartData->txFrameCtnt[bias] =    (uartData->waterLevelReal/1000%10)<<4;//单位：0.01m
			uartData->txFrameCtnt[bias++] += (uartData->waterLevelReal/100%10);
			uartData->txFrameCtnt[bias] =    (uartData->waterLevelReal/10%10)<<4;
			uartData->txFrameCtnt[bias++] += (uartData->waterLevelReal%10);
			uartData->txFrameCtnt[bias] =    (uartData->waterFlowReal/1000%10)<<4;//单位：0.1立方米
			uartData->txFrameCtnt[bias++] += (uartData->waterFlowReal/100%10);
			uartData->txFrameCtnt[bias] =    (uartData->waterFlowReal/10%10)<<4;
			uartData->txFrameCtnt[bias++] += (uartData->waterFlowReal%10);
#elif  ANGLE_CHECK

#endif

			uartData->txFrameCtntNum = bias;
			uartData->masterNum = 0;
			result = enFlag;
			break;
		}
		case 0x77:case 0x78://回复读写ID号
		{
			uartData->txFrameCtnt[bias] =    (SLAVE_NUM/1000%10)<<4;
			uartData->txFrameCtnt[bias++] += (SLAVE_NUM/100%10);
			uartData->txFrameCtnt[bias] =    (SLAVE_NUM/10%10)<<4;
			uartData->txFrameCtnt[bias++] += (SLAVE_NUM%10);
			uartData->txFrameCtntNum = bias;
			uartData->masterNum = 0;
			result = enFlag;
			break;
		}
	}
	return result;
}

/**
  * @brief  根据当前的状态情况，确定需要发送的数据内容
  * @param  txStr-发送数据帧的首地址
  *         length-发送数据帧字节长度
  *         commData-通信数据内容结构体（需要用户自定义）
  * @retval 无
  */
void UartTxHandle(USART_TypeDef* USARTx,uint8_t * txStr,uartCtntStruct * uartData,commCtntStruct * commData)
{
	e_state fucFlag = rstFlag;
	uint8_t txLength = 0;
	fucFlag = UartTxFctnCtntHandle(uartData,commData);
	if(fucFlag == enFlag)
	{
		getDataToFrame(&txStr[0],&txLength,\
				       uartData->fucNum,uartData->masterNum,SLAVE_TYPE,uartData->slaveNum,\
				       uartData->txFrameCtntNum,&uartData->txFrameCtnt[0]);
		SendStr(USARTx,&txStr[0],txLength);
	}
}

/**
  * @brief  串口接收中断函数，负责处理串口接收到的数据帧的提取、数据内容、相应的回应等.
  * @param  rxSuf-接收数据缓存数组首地址，
  *         length-接收数据的实际长度（不包括帧头和帧尾）
  *         commData-通信数据内容结构体（需要用户自定义）
  * @retval None
  */
e_state UartRcvHandle(char * rxStr,uint8_t length,uartCtntStruct * uartData,commCtntStruct * commData)
{
	e_state fucFlag = rstFlag;
	fucFlag = uartRcvContentHandle(rxStr,length,uartData);//调用串口内容提取函数  返回是否正确
	if(fucFlag == enFlag)//成功提取出数据帧中的数据内容
	{
		//进行数据处理，此处内容需要复写
		fucFlag = UartRxFctnCtntHandle(uartData,commData);//调用自定义的串口内容处理函数 返回是否需要回应
	}
	return fucFlag;
}


/**
  * @brief  当接收到设定从机号的数据设定参数，记录下从机的相关参数，并回复主机信息
  * @param  ctrlComm-从机通信数据交互结构体
  * @retval None
  */
e_state setParaData(uartCtntStruct * uartData,commCtntStruct * commData)
{
	uint8_t i = 0;
	uint8_t temp = 0;
	e_state result = rstFlag;
	float tempf = 0;
	if((uartData->slaveNum==SLAVE_NUM) && (uartData->rxSlaveType==SLAVE_TYPE))
	{
		result = enFlag;
	}
	return result;
}

/**
  * @brief  接收到读写ID帧格式，如果是写入需要验证是否为本从机的ID帧，若是，则需要将新的ID号写入到flash中保存
  *                          如果是读出，则读出相应ID号的值
  * @param  ctrlComm-从机通信数据交互结构体
  * @retval None
  */
e_state setFlashData(uartCtntStruct *uartData)
{
	e_state result = rstFlag;
	uint16_t data1 = 0;
	uint8_t oldNum = 0;
	uint8_t newNum = 0;
	uint8_t temp[3] = {0};
	if((uartData->rxFrameCtntNum==6) && (uartData->fucNum==0x77))//写
	{
		oldNum = ((uartData->rxFrameCtnt[0]&0xF0)>>4)*10+(uartData->rxFrameCtnt[0]&0x0F);
		oldNum = oldNum*100 + ((uartData->rxFrameCtnt[1]&0xF0)>>4)*10+(uartData->rxFrameCtnt[1]&0x0F);
		newNum = ((uartData->rxFrameCtnt[2]&0xF0)>>4)*10+(uartData->rxFrameCtnt[2]&0x0F);
		newNum = newNum*100 + ((uartData->rxFrameCtnt[3]&0xF0)>>4)*10+(uartData->rxFrameCtnt[3]&0x0F);
		temp[0] = ((oldNum^newNum)/100)%10;
		temp[1] = ((oldNum^newNum)/10)%10;
		temp[1] = temp[1]*10 + (oldNum^newNum)%10;
		temp[2] = (uartData->rxFrameCtnt[5]&0xF0)>>4;
		temp[2] = temp[2]*10 + (uartData->rxFrameCtnt[5]&0x0F);
		if((temp[0] == uartData->rxFrameCtnt[4])&&(temp[1]==temp[2]))//校验正确了
		{
			//判断是否可以更新flash的值，写入新的ID号
			if((uartData->rxSlaveType == SLAVE_TYPE) && (oldNum == SLAVE_NUM))
			{
				SLAVE_NUM = newNum;
				data1 = newNum;
				result = flashHandle(&data1,1);
			}
		}
	}
	else if((uartData->rxFrameCtntNum==2) && (uartData->fucNum==0x78))//读
	{
		if((uartData->rxFrameCtnt[0]==0xDD) && (uartData->rxFrameCtnt[1]==0xDD))
		{
			flashHandle(&data1,0);
			if(data1!=0)
			{
				//uartData->slaveNum = (uint8_t)data1;
				result = enFlag;
			}
		}
	}
	return result;
}




