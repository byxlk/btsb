/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "bsp.h"

/* Demo application includes. */
#include "debug_serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
//static QueueHandle_t xRxedChars;
//static QueueHandle_t xCharsForTx;

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );

/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    xComPortHandle xReturn;

	/* Create the queues used to hold Rx/Tx characters. */
	//xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	//xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	//if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	//{
#if 0
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;

        /* æ‰“å¼€ GPIO æ—¶é’Ÿ */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		/* æ‰“å¼€ UART æ—¶é’Ÿ */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		/* å°† PA9 æ˜ å°„ä¸º USART1_TX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

		/* å°† PA10 æ˜ å°„ä¸º USART1_RX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

		/* é…ç½® USART Tx ä¸ºå¤ç”¨åŠŸèƒ½ */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* è¾“å‡ºç±»åž‹ä¸ºæŽ¨æŒ½ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* å†…éƒ¨ä¸Šæ‹‰ç”µé˜»ä½¿èƒ½ */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* å¤ç”¨æ¨¡å¼ */

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* é…ç½® USART Rx ä¸ºå¤ç”¨åŠŸèƒ½ */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* ç¬¬2æ­¥ï¼š é…ç½®ä¸²å£ç¡¬ä»¶å‚æ•° */
		USART_InitStructure.USART_BaudRate = 115200;	/* æ³¢ç‰¹çŽ‡ */
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ä½¿èƒ½æŽ¥æ”¶ä¸­æ–­ */

		/* ä½¿èƒ½ä¸²å£1ä¸­æ–­ */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		æ³¨æ„: ä¸è¦åœ¨æ­¤å¤„æ‰“å¼€å‘é€ä¸­æ–­
		å‘é€ä¸­æ–­ä½¿èƒ½åœ¨SendUart()å‡½æ•°æ‰“å¼€
		*/
		USART_Cmd(USART1, ENABLE);		/* ä½¿èƒ½ä¸²å£ */

		/* CPUçš„å°ç¼ºé™·ï¼šä¸²å£é…ç½®å¥½ï¼Œå¦‚æžœç›´æŽ¥Sendï¼Œåˆ™ç¬¬1ä¸ªå­—èŠ‚å‘é€ä¸å‡ºåŽ»
		å¦‚ä¸‹è¯­å¥è§£å†³ç¬¬1ä¸ªå­—èŠ‚æ— æ³•æ­£ç¡®å‘é€å‡ºåŽ»çš„é—®é¢˜ */
		USART_ClearFlag(USART1, USART_FLAG_TC);     /* æ¸…å‘é€å®Œæˆæ ‡å¿—ï¼ŒTransmission Complete flag */
#endif
	//}
	//else
	//{
		xReturn = ( xComPortHandle ) 0;
	//}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;
	( void ) pxPort;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports UART1. */
	( void ) pxPort;

	/* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;
	while( *pxNext )
	{
		xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
		pxNext++;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
signed portBASE_TYPE xReturn;

	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
	{
		xReturn = pdPASS;
		USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/
#if 0
void vUARTInterruptHandler( void )
//void USART1_IRQHandler(void)
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			USART_SendData( USART1, cChar );
		}
		else
		{
			USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
		}
	}

	if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART1 );
		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: fputc
*	¹¦ÄÜËµÃ÷: ÖØ¶¨Òåputcº¯Êý£¬ÕâÑù¿ÉÒÔÊ¹ÓÃprintfº¯Êý´Ó´®¿Ú1´òÓ¡Êä³ö
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
#if USING_FIFO_EN == 1	/* ½«ÐèÒªprintfµÄ×Ö·ûÍ¨¹ý´®¿ÚÖÐ¶ÏFIFO·¢ËÍ³öÈ¥£¬printfº¯Êý»áÁ¢¼´·µ»Ø */
	comSendChar(COM1, ch);

	return ch;
#else	/* ²ÉÓÃ×èÈû·½Ê½·¢ËÍÃ¿¸ö×Ö·û,µÈ´ýÊý¾Ý·¢ËÍÍê±Ï */

	/* µÈ´ý·¢ËÍ½áÊø */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)	{}

	/* Ð´Ò»¸ö×Ö½Úµ½USART1 */
	USART_SendData(USART1, (uint8_t) ch);

	/* µÈ´ý·¢ËÍ½áÊø */
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}

	return ch;
#endif
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: fgetc
*	¹¦ÄÜËµÃ÷: ÖØ¶¨Òågetcº¯Êý£¬ÕâÑù¿ÉÒÔÊ¹ÓÃgetcharº¯Êý´Ó´®¿Ú1ÊäÈëÊý¾Ý
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
int fgetc(FILE *f)
{

#if USING_FIFO_EN == 1	/* ´Ó´®¿Ú½ÓÊÕFIFOÖÐÈ¡1¸öÊý¾Ý, Ö»ÓÐÈ¡µ½Êý¾Ý²Å·µ»Ø */
	uint8_t ucData;

	while(comGetChar(COM1, &ucData) == 0);

	return ucData;
#else
	/* µÈ´ý´®¿Ú1ÊäÈëÊý¾Ý */
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
#endif
}
#endif

