/**********************************************************************************************************
 * @文件     uart4.c			//贾宇凡魔改uart1.c	注意dma请求映像
 * @说明     uart4初始化(裁判系统通信)
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2020.3
**********************************************************************************************************/
#include "main.h"

unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeSend[SEND_MAX_SIZE];
/**********************************************************************************************************
*函 数 名: uart1_Configuration
*功能说明: USART1初始化(裁判系统通信)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void uart1_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_9;  
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);

	USART_DeInit(USART1);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART1,&usart);

	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); 
	
	
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel5);

		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)JudgeReceiveBuffer;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = JudgeBufBiggestSize;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel5, &dma);
		DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel5, ENABLE);
		
	nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	}
  
//    nvic.NVIC_IRQChannel = DMA2_Channel5_IRQn;
//		nvic.NVIC_IRQChannelPreemptionPriority = 3;
//		nvic.NVIC_IRQChannelSubPriority = 3;
//		nvic.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&nvic);
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//		{
//		  DMA_InitTypeDef 	dma;
//			DMA_DeInit(DMA2_Channel5);
//			dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//			dma.DMA_MemoryBaseAddr = (uint32_t)JudgeSend;
//			dma.DMA_DIR = DMA_DIR_PeripheralDST;
//			dma.DMA_BufferSize = 30;
//			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//			dma.DMA_Mode = DMA_Mode_Circular;
//			dma.DMA_Priority = DMA_Priority_VeryHigh;
//			dma.DMA_M2M = DMA_M2M_Disable;

//			DMA_Init(DMA2_Channel5,&dma);
//			DMA_ITConfig(DMA2_Channel5,DMA_IT_TC,ENABLE);
//			DMA_Cmd(DMA2_Channel5,DISABLE);
//	 }		
}

void DMA1_Channel5_IRQHandler()
{	
	if(DMA_GetITStatus(DMA1_FLAG_TC5) != RESET)
	{
		JudgeBuffReceive(JudgeReceiveBuffer,0); 
		DMA_ClearITPendingBit(DMA1_FLAG_TC5);
	}
}


//void DMA2_Channel5_IRQHandler(void)
//{
//	if(DMA_GetITStatus(DMA2_IT_TC5)!=RESET)
//	{
//		DMA_ClearFlag(DMA2_FLAG_TC5);
//		DMA_ClearITPendingBit(DMA2_IT_TC5);
//		DMA_Cmd(DMA2_Channel5,DISABLE);
//	}	
//}
