/*
 * FileName:
 * Author:         YuanYin  Version: QXW-V1.x  Date: 2010-3-11
 * Description: ���ļ���ĺ���������1.Uart�����һ���ģ�����Ͳ����ظ�ע���ˡ�
 * Version:
 * Function List:
 *                 1.
 * History:
 *     <author>   <time>    <version >   <desc>
 */
#include "Drivers.h"
#include <stdio.h>

void COM1_Init( void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ��1���� ����GPIO  TX = PA9   RX = PA10 */
    /* �� GPIO ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* �� UART ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* �� PA9 ӳ��Ϊ USART1_TX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

    /* �� PA10 ӳ��Ϊ USART1_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    /* ���� USART Tx Ϊ���ù��� */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ���� USART Rx Ϊ���ù��� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;	//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;		//��żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//ģʽ

    /* ��2���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = 115200;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //��żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //ģʽ
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	//ʹ�ܴ����жϣ����������ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	//���ṹ�嶪�����ú�������д�뵽��Ӧ�Ĵ�����

	/* Enable USART1 Receive interrupts */
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

void COM3_Init( void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* �� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* �� UART ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* �� PB10 ӳ��Ϊ USART3_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* �� PB11 ӳ��Ϊ USART3_RX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ��2���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = 115200;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */

	//ʹ�ܴ����жϣ����������ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART3_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	//���ṹ�嶪�����ú�������д�뵽��Ӧ�Ĵ�����

	/* Enable USART3 Receive interrupts */
    //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
}

void PrintUart3(const u8 *Str)
{
	while(*Str)
	{
		USART_SendData(USART3, (unsigned char)*Str++);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
}

//��ʹ�ð�����ģʽ
#if 1 //���û����Σ�����Ҫ��targetѡ����ѡ��ʹ��USE microLIB
#pragma import(__use_no_semihosting)
struct __FILE
{
	int handle;
};
FILE __stdout;

_sys_exit(int x)
{
	x = x;
}
#endif

extern u8 UseUsbOutputInfoFlag;
extern void USB_Send_Data(u8 Byte);
int fputc(int ch, FILE *f)
{
	if(UseUsbOutputInfoFlag)
		USB_Send_Data((u8)ch);
	else
	{
		USART_SendData(USART1, (unsigned char)ch);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}

	return ch;
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

