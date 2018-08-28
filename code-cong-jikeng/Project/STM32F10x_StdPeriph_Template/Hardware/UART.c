#include "UART.h"

/**
******************************************************************************
* @file    UART.c
* @author  wqdnan
* @version V3.5.0
* @date    27-09-2017
* @brief   初始化串口，使能串口收发，定义串口接收函数和发送函数
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/


char rxBuf1[RXBUF1_LENGTH] = {0};
char rxBuf2[RXBUF2_LENGTH] = {0};
char rxBuf4[RXBUF4_LENGTH] = {0};

char txBuf1[TXBUF1_LENGTH] = {0};
char txBuf2[TXBUF2_LENGTH] = {0};
char txBuf4[TXBUF4_LENGTH] = {0};

uint8_t rxLength1 = 0;
uint8_t rxLength2 = 0;
uint8_t rxLength4 = 0;

e_state rx1Flag = rstFlag;
e_state rx2Flag = rstFlag;
e_state rx4Flag = rstFlag;



/**
  * @brief  UART1初始化，使能接收中断，中断函数为USART1_IRQn，使能发送.
  * @param  None
  * @retval None
  */
void UART1Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(USART1_GPIO_CLK | USART1_CLK | RCC_APB2Periph_AFIO, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//NVIC_PriorityGroup_0 优先级组最高，NVIC_PriorityGroup_1次之
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//最高的优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//设定为主优先级是1，从优先级 是1  ，从优先级之间不能相互打断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = U485_DEPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);
	RX_DE1();//使开始时串口处于接收状态
	GPIO_InitStructure.GPIO_Pin = USART1_RxPin;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPD;//
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USART1_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

}

/**
  * @brief  UART2初始化，使能接收中断，中断函数为USART2_IRQn，使能发送.
  * @param  None
  * @retval None
  */
void UART2Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(USART2_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(USART2_CLK, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//NVIC_PriorityGroup_0 优先级组最高，NVIC_PriorityGroup_1次之
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//设定为主优先级是1，从优先级 是1  ，从优先级之间不能相互打断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART2_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART2_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USART2_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART2_GPIO, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  UART3初始化，使能接收中断，中断函数为USART3_IRQn，使能发送.
  * @param  None
  * @retval None
  */
void UART3Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(USART3_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(USART3_CLK, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//NVIC_PriorityGroup_0 优先级组最高，NVIC_PriorityGroup_1次之
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//设定为主优先级是1，从优先级 是1  ，从优先级之间不能相互打断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART3_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USART3_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  UART4初始化，使能接收中断，中断函数为USART4_IRQn，使能发送.
  * @param  None
  * @retval None
  */
void UART4Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(UART4_GPIO_CLK | UART4_DE_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(UART4_CLK, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //将PA15映射为普通的IO口工作
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//NVIC_PriorityGroup_0 优先级组最高，NVIC_PriorityGroup_1次之
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;//设定为主优先级是1，从优先级 是1  ，从优先级之间不能相互打断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART4_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = UART4_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = UART4_DEPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(UART4_DE_GPIO, &GPIO_InitStructure);
//	TX_DE4();

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);


}

/**
  * @brief  UART5初始化，使能接收中断，中断函数为USART5_IRQn，使能发送.
  * @param  None
  * @retval None
  */
void UART5Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(UART5_GPIO_CLK_T | UART5_GPIO_CLK_R | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(UART5_CLK, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//NVIC_PriorityGroup_0 优先级组最高，NVIC_PriorityGroup_1次之
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;//设定为主优先级是1，从优先级 是1  ，从优先级之间不能相互打断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART5_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART5_GPIO_R, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = UART5_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UART5_GPIO_T, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5, ENABLE);
}

/**
  * @brief  选择某个串口发送指定的字符串.
  * @param  USARTx-选取的串口号，str-待发送的字符首地址，length-待发送的字符字节长度
  * @retval None
  */
void SendStr(USART_TypeDef* USARTx,uint8_t *str,uint8_t length)
{
	uint8_t i = 0,j = 0;
	if(ANGLE_UART == USARTx)
		TX_DE4();
	else if(SLAVE_UART == USARTx)
		TX_DE1();
	for(i=0;i<20;i++)//等待使能信号稳定,如果是i<10,则不能稳定
		for(j=0; j<5; j++);
	for(i=0;i<length;i++)
	{
		USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);//等待发送完成
	}
	for(i=0;i<200;i++)//等待使能信号稳定,如果是i<10,则不能稳定
		for(j=0; j<45; j++);
	if(ANGLE_UART == USARTx)
		RX_DE4();
	else if(SLAVE_UART == USARTx)
		RX_DE1();

}
/**
  * @brief  选择某个串口发送指定的字符串.
  * @param  USARTx-选取的串口号，str-待发送的字符首地址，length-待发送的字符字节长度 固定帧头为‘$’，帧尾为'\n''\r' 为0A，0D
  * @retval None
  */
void USART1_IRQHandler(void)
{
	static uint8_t rxCnt1 = 0;
	static uint8_t rcvFlag = 0;
	uint8_t temp = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	  {
		 USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		 temp = USART_ReceiveData(USART1);
		 if((temp == FRAME_START) && (rx1Flag == rstFlag))//找到了帧头
		 {
			 rcvFlag = 1;
			 rxCnt1 = 0;
		 }
		 else if((temp == FRAME_END) && (rcvFlag == 1))//找到帧尾
		 {
			 //置标志位
			 rxLength1 = rxCnt1;//这个地方的长度不包括帧尾在内的字节数
			 rx1Flag = enFlag;
			 rcvFlag = 0;
		 }
		 else if(rcvFlag == 1)
		 {
			 rxBuf1[rxCnt1++] = temp;//不记录帧头和帧尾，只有内容
		 }

	  }
}
/**
  * @brief  选择某个串口发送指定的字符串.  主机与从机的通信中断
  * @param  USARTx-选取的串口号，str-待发送的字符首地址，length-待发送的字符字节长度
  * @retval None
  */
void USART2_IRQHandler(void)
{
	static uint8_t rxCnt1 = 0;
	static uint8_t rcvFlag = 0;
	uint8_t temp = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	  {
		 USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		 temp = USART_ReceiveData(USART2);
//		 USART_SendData(USART2,temp);
//		 if((temp == 0xA7) && (rx1Flag == rstFlag))//找到了帧头
//		 {
//			 rcvFlag = 1;
//			 rxCnt1 = 0;
//		 }
//		 else if((temp == 0xB7) && (rcvFlag == 1))//找到帧尾
//		 {
//			 //置标志位
//			 rxLength2 = rxCnt1;//这个地方的长度不包括帧尾在内的字节数
//			 rx2Flag = enFlag;
//			 rcvFlag = 0;
//		 }
//		 else if(rcvFlag == 1)
//		 {
//			 rxBuf2[rxCnt1++] = temp;//不记录帧头和帧尾，只有内容
//		 }

	  }

}
/**
  * @brief  选择某个串口发送指定的字符串.
  * @param  USARTx-选取的串口号，str-待发送的字符首地址，length-待发送的字符字节长度
  * @retval None
  */
void USART3_IRQHandler(void)
{
	uint8_t temp = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		temp = USART_ReceiveData(USART3);
//		USART_SendData(USART3,temp);
	}
}

void UART4_IRQHandler(void)
{
	uint8_t temp = 0;
	static uint8_t rxCnt1 = 0;
	static uint8_t rcvFlag = 0;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		temp = USART_ReceiveData(UART4);
		if((temp == 0xA7) && (rx4Flag == rstFlag))//找到了帧头
		{
		 rcvFlag = 1;
		 rxCnt1 = 0;
		}
		else if((temp == 0xB7) && (rcvFlag == 1))//找到帧尾
		{
		 //置标志位
		 rxLength4 = rxCnt1;//这个地方的长度不包括帧尾在内的字节数
		 rx4Flag = enFlag;
		 rcvFlag = 0;
		}
		else if(rcvFlag == 1)
		{
		 rxBuf4[rxCnt1++] = temp;//不记录帧头和帧尾，只有内容
		}
	}
}

void UART5_IRQHandler(void)
{
	uint8_t temp = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		temp = USART_ReceiveData(UART5);
//		USART_SendData(UART5,temp);
	}
}
