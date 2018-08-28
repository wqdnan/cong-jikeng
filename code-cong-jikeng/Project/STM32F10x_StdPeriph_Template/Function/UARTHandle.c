#include "UARTHandle.h"
//#include "INOUT.h"
//#include "UARTControl.h"

/**
******************************************************************************
* @file    UARTHandle.c
* @author  wqdnan
* @version V3.5.0
* @date    25-12-2017
* @brief   处理接收到的串口数据，并进行数据提取；完成串口发送数据的组帧，并发送
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/

//add
uartCtntStruct uartData;
//end

/**
  * @brief  主机端根据协议内容进行组帧，包括主从机协议和主机和中心控制室协议内容
  * @param  dest-组帧完成的数据帧的存放地址
  *         destLength-数据帧长度
  *         fctnNum-功能码
  *         masterNum-主机号
  *         slaveNum-从机号
  *         dataLength-数据内容字节长度
  *         dataCtnt-数据内容存放的首地址
  * @retval None
  * 格式：帧头|功能码|主机编号|从机类型|从机编号|数据长度|数据内容|校验和|帧尾
  * 字节数    1    1     1       1        1       1      不定        1     1
  */
void getDataToFrame(uint8_t *dest,uint8_t * destLength,uint8_t fctnNum,uint8_t masterNum,uint8_t slaveType,uint8_t slaveNum,uint8_t dataLength,uint8_t * dataCtnt)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t temp = 0;
	dest[i++] = FRAME_START;
	dest[i++] = fctnNum;
	dest[i++] = masterNum;
	dest[i++] = slaveType;
	dest[i++] = slaveNum;
	dest[i++] = dataLength;
	for(j=0; j<dataLength; j++)//当dataLength==0时同样适用
		dest[i++] = dataCtnt[j];
	for(j=1; j<i; j++)
		temp ^=  dest[j];//异或校验不包括帧头和帧尾
	if((temp == FRAME_START) || (temp == FRAME_END))//校验位与帧头帧尾进行规避
		temp += 1;
	dest[i++] = temp;
	dest[i] = FRAME_END;
	*destLength = i+1;//记录数据帧的长度
}

/**
  * @brief  提取分解出不同数据帧中的数据内容
  * @param  dataCtnt-数据内容存放的首地址
  *         rxStr-处理数据帧的首地址（去掉了帧头和帧尾）
  *         sFctnType-从机的工作状态指示
  *         mFctnType-主机的工作状态指示
  * @retval None
  * 格式：功能码|主机编号|从机编号|数据长度|数据内容|校验和
  * 字节数      1     1       1       1      不定        1
  */
void analyzeFrameContent(uint8_t * rxStr,uartCtntStruct * uartData)
{
	uint8_t dataLen = 0;
	uint8_t i = 0, j = 0;
	uint16_t dataTemp[40] = {0};
	uint8_t bias = 0;//用于记录手动控制命令数据内容个数的偏移量
	uint8_t * pData = &rxStr[5];//指向数据内容的首地址

	uartData->fucNum = rxStr[0];
	uartData->rxSlaveType = rxStr[2];
	uartData->slaveNum = rxStr[3];
	if((rxStr[3] <= SLAVEMAXNUM) && (rxStr[1] == 0x1F) && (rxStr[2] == SLAVE_TYPE))//有效的从机号,并且是主机的询问帧
	{
		//开始提取从机中的有效数据了
		dataLen = rxStr[4];
		for(i=0; i<dataLen; i++)
		{
//			while((pData[i] != ',') &&(i<dataLen)) //,ASCII码为0x2C
//			{
//				dataTemp[bias] = dataTemp[bias] * 10 + pData[i++];
//			}
			//一个字节一个从机的功能数据
			uartData->rxFrameCtnt[bias++] = pData[i];//rxStr[i+4];修改了偏移
		}
		//放入到数据结构体中
		uartData->masterNum = rxStr[1];
		uartData->rxFrameCtntNum = bias;

	}
}

/**
  * @brief  用于在刚接收到一帧数据后，按照帧格式对数据内容进行提取
  * @param  rxStr-接收数据内容的首地址
  *         length-接收数据的实际长度（去掉了帧头和帧尾）
  *         uartData-串口按照数据帧提取的数据结构体
  * @retval
  *  格式：帧头|功能码|主机编号|从机编号|数据长度|数据内容|校验和|帧尾
  * 字节数    1    1     1       1       1      不定        1     1
  */
e_state uartRcvContentHandle(char * rxStr,uint8_t length,uartCtntStruct * uartData)
{
	e_state result = rstFlag;
	uint8_t i = 0;
	uint8_t tempCheck = 0;
	for(i=0;i<(length-1);i++)
	{
		tempCheck ^= rxStr[i];
	}
	if((tempCheck == FRAME_START) || (tempCheck == FRAME_END))
		tempCheck += 1;
	if(tempCheck == rxStr[length-1])//校验和正确 开始提取数据
	{
		result = enFlag;
		analyzeFrameContent(rxStr,uartData);
	}
	return result;
}
