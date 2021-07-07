//#ifndef __USART_H__
//#define __USART_H__

//#include "stm32f10x_usart.h"
//#include "stm32f10x_dma.h"

//#define RX_USART3_BUFFER 30

//void USART3_Config(void);

//#endif
#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"

#define RX_USART3_BUFFER 30

typedef struct UART_HandleTypeDef
{	
	USART_TypeDef *UART_BASEx;
	USART_InitTypeDef UARTx;
	GPIO_InitTypeDef  UARTx_Tx;
	GPIO_InitTypeDef  UARTx_Rx;
}UART_HandleTypeDef;

void usart_Config(void);
void UART_Config(UART_HandleTypeDef *uart,uint32_t USART_BaudRate, 
	           uint16_t USART_WordLength,uint16_t USART_StopBits,
               uint16_t USART_Parity,uint16_t USART_Mode);
void UART_GPIO_RX_Config(UART_HandleTypeDef *uart, 
	                uint16_t GPIO_PinRx,GPIO_TypeDef *GPIORx);
void UART_GPIO_TX_Config(UART_HandleTypeDef *uart, 
	                uint16_t GPIO_PinTx,GPIO_TypeDef *GPIOTx);
void master2slave_task(void);
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);
void ReturnState(void);
					
#endif

