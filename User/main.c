/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.0
*	˵    �� : ��ʵ����Ҫʵ��FreeRTOS++STemWin+FatFS�ۺ�
*              ʵ��Ŀ�ģ�
*                1. ѧϰFreeRTOS++STemWin+FatFS�ۺ�
*              ʵ�����ݣ�
*                1. ���°���K1����ͨ�����ڴ�ӡ����ִ�������������115200������λ8����żУ��λ�ޣ�ֹͣλ1��
*                   =================================================
*                   ������      ����״̬ ���ȼ�   ʣ��ջ �������
*                   vTaskUserIF     R       2       270     2
*                   vTaskGUI        R       1       150     1
*                   IDLE            R       0       114     6
*                   vTaskLED        B       3       484     3
*                   vTaskStart      B       5       490     5
*                   vTaskMsgPro     S       4       480     4
*
*
*                   ������       ���м���         ʹ����
*                   vTaskUserIF     4611            <1%
*                   vTaskGUI        152759          14%
*                   IDLE            884172          83%
*                   vTaskLED        0               <1%
*                   vTaskStart      16259           1%
*                   vTaskMsgPro     1               <1%
*                  �����������ʹ��SecureCRT��V4���������д�������鿴��ӡ��Ϣ��
*                  ��������ʵ�ֵĹ������£�
*                   vTaskGUI        ����: emWin����
*                   vTaskTaskUserIF ����: �ӿ���Ϣ����
*                   vTaskLED        ����: LED��˸
*                   vTaskMsgPro     ����: ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD����
*                   vTaskStart      ����: ��������Ҳ����������ȼ���������ʵ�ְ���ɨ��ʹ������
*                2. ��������״̬�Ķ������£������洮�ڴ�ӡ��ĸB, R, D, S��Ӧ��
*                    #define tskBLOCKED_CHAR		( 'B' )  ����
*                    #define tskREADY_CHAR		    ( 'R' )  ����
*                    #define tskDELETED_CHAR		( 'D' )  ɾ��
*                    #define tskSUSPENDED_CHAR	    ( 'S' )  ����
*                3. K2�������£�ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD����PicSave�ļ����С�
*              ע�����
*                1. ��ʵ���Ƽ�ʹ�ô������SecureCRT��Ҫ�����ڴ�ӡЧ�������롣�������
*                   V4��������������С�
*                2. ��ؽ��༭��������������TAB����Ϊ4���Ķ����ļ���Ҫ��������ʾ�����롣
*
*	�޸ļ�¼ :
*		�汾��    ����         ����            ˵��
*       V1.0    2016-03-15   Eric2013    1. ST�̼��⵽V3.6.1�汾
*                                        2. BSP������V1.2
*                                        3. FreeRTOS�汾V8.2.3
*                                        4. FatFS�汾V0.11
*                                        5. STemWin�汾V5.28
*
*	Copyright (C), 2016-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include <includes.h>
#include "FreeRTOS_CLI.h"
#include "MainTask.h"
#include "spi_flash_fatfs.h"

/* Dimentions a buffer to be used by the UART driver, if the UART driver uses a
buffer at all. */
#define cmdQUEUE_LENGTH			1024

/* DEL acts as a backspace. */
#define cmdASCII_DEL		    ( 0x7F )

/* The maximum time to wait for the mutex that guards the UART to become
available. */
#define cmdMAX_MUTEX_WAIT		pdMS_TO_TICKS( 300 )

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE		50

/* Misc defines. */
#define serINVALID_QUEUE		( ( QueueHandle_t ) 0 )
#define serNO_BLOCK				( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME		( 40 / portTICK_PERIOD_MS )
/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/


/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUserKeyIF = NULL;
//static TaskHandle_t xHandleTaskFsDebug = NULL;
//static TaskHandle_t xHandleTaskMsgPro = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
//static TaskHandle_t xHandleTaskAdcProc = NULL;

static SemaphoreHandle_t  xMutex = NULL;
static SemaphoreHandle_t  xSemaphore_key_interupt = NULL;

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

/* Used to guard access to the UART in case messages are sent to the UART from
more than one task. */
static SemaphoreHandle_t xTxMutex = NULL;

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage1 = "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n";
static const char * const pcWelcomeMessage2 = "[Press ENTER to execute the previous command again]\r\n\r\n";
static const char * const pcEndOfOutputMessage = "cli@FreeRTOS->";
static const char * const pcNewLine = "\r\n";

SLEEP_DATA_T gSleep_Data = {2017,1,1,7,12,0,0,0,0,0};

/*
*********************************************************************************************************
*	�� �� ��: vTaskGUI
*	����˵��: emWin����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 1   (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
static void vTaskGUI(void *pvParameters)
{
    printf("Main GUI Thread start.\r\n");
	while (1)
	{
		MainTask();
		vTaskDelay(1000);
	}
    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    //vTaskDelete( NULL );

}

/*
*********************************************************************************************************
*	�� �� ��: vTaskAdcProc
*	����˵��: ADת������
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2
*********************************************************************************************************
*/
//static void vTaskAdcProc(void *pvParameters)
//{

    //float uwVBATVoltage;    /* ���ص��ӵ�ѹ */
    //float ufVoltage_PA0;    /* PA0���ŵ�ѹ  */
    //float ufVoltage_PC0;    /* PC0���ŵ�ѹ  */
    //printf("ADC capture Thread start.\r\n");
    //while(1)
    //{
    //    vTaskDelay(1000);
        //uwVBATVoltage = ADC_ConvertedValue[1] * 3.3 / 4095;
        //ufVoltage_PA0 = ADC_ConvertedValue[2] * 3.3 / 4095;
        //ufVoltage_PC0 = ADC_ConvertedValue[3] * 3.3 / 4095;
       // GetTemp(ADC_ConvertedValue[0]);
    //}

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    //vTaskDelete( NULL );
//}

/*
*********************************************************************************************************
*	�� �� ��: vTaskTaskUserIF
*	����˵��: ������Ϣ����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2
*********************************************************************************************************
*/
static void vTaskUserKeyIF(void *pvParameters)
{
	uint8_t ucKeyCode;
	uint8_t pcWriteBuffer[500];
    BaseType_t xResult;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000); /* �������ȴ�ʱ��Ϊ 300ms */

    printf("User Key Process Thread start.\r\n");

    while(1)
    {
        xResult = xSemaphoreTake(xSemaphore_key_interupt, (TickType_t)xMaxBlockTime);
        if(xResult == pdFALSE)
            continue;

        bsp_TouchKeyCodeValueProcess();   /* �жϵ������ȡ������KeyCodeֵ*/

        /* �����˲��ͼ���ɺ�̨systick�жϷ������ʵ�֣�����ֻ��Ҫ����bsp_GetKey��ȡ��ֵ���ɡ� */
		ucKeyCode = bsp_GetKey();  /* bsp_GetKey()��ȡ��ֵ, �޼�����ʱ���� KEY_NONE = 0 */

		if (ucKeyCode != KEY_NONE)
		{
		    //printf("\r\nPress Key Value %04x   ",ucKeyCode);
			switch (ucKeyCode)
			{
				/* K1������ ��ӡ����ִ����� */
                case KEY_DOWN_DEBUG_LONG:
                    printf("Press Key: DEBUG LONG \r\n");
                    //GUI_SendKeyMsg(GUI_KEY_F2, 1);
                    //break;
				case KEY_DOWN_DEBUG:
                    if(ucKeyCode == KEY_DOWN_DEBUG)
                        printf("Press Key: DEBUG \r\n");
					printf("=================================================\r\n");
					printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
					vTaskList((char *)&pcWriteBuffer);
					printf("%s\r\n", pcWriteBuffer);

					printf("\r\n������       ���м���         ʹ����\r\n");
					vTaskGetRunTimeStats((char *)&pcWriteBuffer);
					printf("%s\r\n", pcWriteBuffer);
					printf("��ǰ��̬�ڴ�ʣ���С = %d�ֽ�\r\n", xPortGetFreeHeapSize());
                    //GUI_SendKeyMsg(GUI_KEY_F2, 1);
					break;

				case KEY_DOWN_MUX:/* ���� */
                    printf("Press Key: MUX \r\n");
                    GUI_SendKeyMsg(GUI_KEY_LockScreen, 1);
                    break;

                case KEY_DOWN_MUX_LONG:/* ���� */
                    printf("Press Key: MUX LONG\r\n");
                    GUI_SendKeyMsg(GUI_KEY_UnLock, 1);
                    break;

				/* K2�����£�ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD���� */
				case KEY_DOWN_VOL_DOWN:
                    printf("Press Key: VOL- \r\n");
                    GUI_SendKeyMsg(GUI_KEY_Vol_Dec, 1);
                    break;

                case KEY_DOWN_VOL_UP:
                    printf("Press Key: VOL+ \r\n");
					GUI_SendKeyMsg(GUI_KEY_Vol_Plus, 1);
					break;

                case KEY_DOWN_PLAY_PAUSE:/* ���š�ȷ�� */
                    printf("Press Key: PLAY/PAUSE \r\n");
                    GUI_SendKeyMsg(GUI_KEY_PlayPause, 1);
                    break;

                case KEY_DOWN_PLAY_PAUSE_LONG:/* ѡ���� */
                    printf("Press Key: PLAY/PAUSE LONG\r\n");
                    GUI_SendKeyMsg(GUI_KEY_PlayPause_Long, 1);
                    break;

                case KEY_DOWN_MENU:/* �˵� */
                    printf("Press Key: MENU \r\n");
                    GUI_SendKeyMsg(GUI_KEY_Menu, 1);
                    break;

                case KEY_DOWN_MENU_LONG:/* ֱ�ӽ���˯��ģʽ */
                    printf("Press Key: MENU LONG\r\n");
                    GUI_SendKeyMsg(GUI_KEY_Menu_Long, 1);
                    break;

                case KEY_DOWN_UP:/* ��һ�� */
                    printf("Press Key: UP \r\n");
                    GUI_SendKeyMsg(GUI_KEY_Direction_Up, 1);
                    break;

                case KEY_DOWN_UP_LONG:/* ���� */
                    printf("Press Key: UP LONG\r\n");
                    GUI_SendKeyMsg(GUI_KEY_Direction_Left, 1);
                    break;

                case KEY_DOWN_DOWN:/* ��һ�� */
                    printf("Press Key: DOWN \r\n");
                    GUI_SendKeyMsg(GUI_KEY_Direction_Down, 1);
                    break;

                case KEY_DOWN_DOWN_LONG:/* ��� */
                    printf("Press Key: DOWN LONG\r\n");
                    GUI_SendKeyMsg(GUI_KEY_Direction_Right, 1);
                    break;

				/* �����ļ�ֵ������ */
				default:
				    printf("Press Key: UNKNOW \r\n");
					break;
			}
		}

		vTaskDelay(20);
	}

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    //vTaskDelete( NULL );

}

/*
*********************************************************************************************************
*	�� �� ��: vTaskStart
*	����˵��: ��������Ҳ����������ȼ�������Ҫʵ�ְ������ʹ�����⡣
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 5
*********************************************************************************************************
*/
static void vTaskStart(void *pvParameters)
{
    TickType_t tick_current = 0;
    TickType_t tick_backup = 0;
    TickType_t td_tick_backup = 0;

    printf("vTaskStart Thread start.\r\n");

    tick_backup = xTaskGetTickCount();
    while(1)
    {
        tick_current = xTaskGetTickCount();
		/* 10msһ�ΰ������ */

		if(tick_current - tick_backup >= 10)
		{
		    tick_backup = tick_current;

            //gSleep_Data.TempVal += getCurent_IntTempValue();
            gSleep_Data.TempVal = getCurent_ExtTempValue();
            gSleep_Data.LightVal = getLightVLuxValue();
            gSleep_Data.NoisVal = getMicAmp_dBValue();
		}

        if(tick_current - td_tick_backup >= 250)
		{
		    td_tick_backup = tick_current;

			bsp_RTC_GetClock();
            gSleep_Data.Year = g_tRTC.Year;
            gSleep_Data.Mon = g_tRTC.Mon;
            gSleep_Data.Day = g_tRTC.Day;
            gSleep_Data.Week = g_tRTC.Week;
            gSleep_Data.Hour = g_tRTC.Hour;
            gSleep_Data.Min = g_tRTC.Min;
            gSleep_Data.Sec = g_tRTC.Sec;
		}

		vTaskDelay(50);
	}

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    //vTaskDelete( NULL );
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskFsDebug
*	����˵��: FatFs����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2
*********************************************************************************************************
*/
//static void vTaskFsDebug(void *pvParameters)
//{
//    printf("vTaskFsDebug Thread start.\r\n");

//    DemoFatFS();

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
//    vTaskDelete( NULL );
//}

/*
*********************************************************************************************************
*	�� �� ��: vTaskAdcProc
*	����˵��: ADת������
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2
*********************************************************************************************************
*/
#if 0
static void vTaskTest(void *pvParameters)
{
    printf("vTaskTest Thread start.\r\n");

    while(1)
    {
        vTaskDelay(100);
  #ifdef LCD_DRIVER_TEST
        LCD_Fill_Rect(0, 0, 320, 240, CL_BLUE);
        vTaskDelay(1000);
        LCD_Fill_Rect(0, 0, 320, 240, CL_YELLOW);
  #endif
        //GuiTaskTest();
    }

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    //vTaskDelete( NULL );
}
#endif

static signed portBASE_TYPE xSerialGetChar( signed char *pcRxedChar, TickType_t xBlockTime )
{
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

static signed portBASE_TYPE xSerialPutChar( signed char cOutChar, TickType_t xBlockTime )
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

static void vSerialPutString( const signed char * const pcString, unsigned short usStringLength )
{
    signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

    /* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;
	while( *pxNext )
	{
		xSerialPutChar( *pxNext, serNO_BLOCK );
		pxNext++;
	}
}
/*-----------------------------------------------------------*/



/*
*********************************************************************************************************
*	�� �� ��: vTaskCmdLineConsole
*	����˵��: Command Line Interface Thread
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 1
*********************************************************************************************************
*/

static void vTaskCmdLineConsole( void *pvParameters )
{
    signed char cRxedChar;
    uint8_t ucInputIndex = 0;
    char *pcOutputString;
    static char cInputString[cmdMAX_INPUT_SIZE], cLastInputString[cmdMAX_INPUT_SIZE];
    BaseType_t xReturned;

	( void ) pvParameters;

    vRegisterSampleCLICommands();

	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console interface
	will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* Initialise the UART. */
	//xPort = xSerialPortInitMinimal( configCLI_BAUD_RATE, cmdQUEUE_LENGTH );

	/* Send the welcome message. */
	vSerialPutString( ( signed char * ) pcWelcomeMessage1, ( unsigned short ) strlen( pcWelcomeMessage1 ) );
	vSerialPutString( ( signed char * ) pcWelcomeMessage2, ( unsigned short ) strlen( pcWelcomeMessage2 ) );

	for( ;; )
	{
		/* Wait for the next character.  The while loop is used in case
		INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
		be a genuine block time rather than an infinite block time. */
		while( xSerialGetChar( &cRxedChar, portMAX_DELAY ) != pdPASS );

		/* Ensure exclusive access to the UART Tx. */
		if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
		{
			/* Echo the character back. */
			xSerialPutChar( cRxedChar, portMAX_DELAY );

			/* Was it the end of the line? */
			if( cRxedChar == '\n' || cRxedChar == '\r' )
			{
				/* Just to space the output from the input. */
				vSerialPutString( ( signed char * ) pcNewLine, ( unsigned short ) strlen( pcNewLine ) );

				/* See if the command is empty, indicating that the last command
				is to be executed again. */
				if( ucInputIndex == 0 )
				{
					/* Copy the last command back into the input string. */
					strcpy( cInputString, cLastInputString );
				}

				/* Pass the received command to the command interpreter.  The
				command interpreter is called repeatedly until it returns
				pdFALSE	(indicating there is no more output) as it might
				generate more than one string. */
				do
				{
					/* Get the next output string from the command interpreter. */
					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

					/* Write the generated string to the UART. */
					vSerialPutString( ( signed char * ) pcOutputString, ( unsigned short ) strlen( pcOutputString ) );

				} while( xReturned != pdFALSE );

				/* All the strings generated by the input command have been
				sent.  Clear the input string ready to receive the next command.
				Remember the command that was just processed first in case it is
				to be processed again. */
				strcpy( cLastInputString, cInputString );
				ucInputIndex = 0;
				memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );

				vSerialPutString( ( signed char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );
			}
			else
			{
				if( cRxedChar == '\r' )
				{
					/* Ignore the character. */
				}
				else if( ( cRxedChar == '\b' ) || ( cRxedChar == cmdASCII_DEL ) )
				{
					/* Backspace was pressed.  Erase the last character in the
					string - if any. */
					if( ucInputIndex > 0 )
					{
						ucInputIndex--;
						cInputString[ ucInputIndex ] = '\0';
					}
				}
				else
				{
					/* A character was entered.  Add it to the string entered so
					far.  When a \n is entered the complete	string will be
					passed to the command interpreter. */
					if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
					{
						if( ucInputIndex < cmdMAX_INPUT_SIZE )
						{
							cInputString[ ucInputIndex ] = cRxedChar;
							ucInputIndex++;
						}
					}
				}
			}

			/* Must ensure to give the mutex back. */
			xSemaphoreGive( xTxMutex );
		}
	}
}
/*-----------------------------------------------------------*/

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    /* Create that task that handles the console itself. */
	xTaskCreate((TaskFunction_t )vTaskCmdLineConsole,	/* The task that implements the command console. */
				 (const char*    )"CLI",						/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				 (uint16_t       )1024,				/* The size of the stack allocated to the task. */
				 (void*          )NULL,						/* The parameter is not used, so NULL is passed. */
				 (UBaseType_t    )3,					/* The priority allocated to the task. */
				 (TaskHandle_t*  )NULL );						/* A handle is not required, so just pass NULL. */

    /* �����¼����� */
    xTaskCreate((TaskFunction_t )vTaskUserKeyIF,    /* ������  */
                (const char*    )"vTaskUserKeyIF",      /* ������    */
                (uint16_t       )512,                   /* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,                  /* �������  */
                (UBaseType_t    )4,                     /* �������ȼ�*/
                (TaskHandle_t*  )&xHandleTaskUserKeyIF );  /* ������  */

    /* GUI ������� */
	xTaskCreate((TaskFunction_t )vTaskGUI,             /* ������  */
                (const char*    )"vTaskGUI",           /* ������    */
                (uint16_t       )1024,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,                 /* �������  */
                (UBaseType_t    )3,                    /* �������ȼ�*/
                (TaskHandle_t*  )NULL );               /* ������  */
#if 0
	xTaskCreate((TaskFunction_t )vTaskFsDebug,    		/* ������  */
                (const char*    )"vTaskFsDebug",  		/* ������    */
                (uint16_t       )1024,         		/* stack��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,        		/* �������  */
                (UBaseType_t    )3,           		/* �������ȼ�*/
                (TaskHandle_t*  )&xHandleTaskFsDebug ); /* ������  */
#endif
	/* ��ʱ���� */
	xTaskCreate((TaskFunction_t )vTaskStart,     		/* ������  */
                (const char*    )"vTaskStart",   		/* ������    */
                (uint16_t       )512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,           		/* �������  */
                (UBaseType_t    )3,              		/* �������ȼ�*/
                (TaskHandle_t*  )&xHandleTaskStart );   /* ������  */

    /* ADC ������ */
	//xTaskCreate((TaskFunction_t )vTaskAdcProc,     		/* ������  */
    //            (const char*    )"vTaskAdcProc",   		/* ������    */
    //            (uint16_t       )512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
    //            (void*          )NULL,           		/* �������  */
    //            (UBaseType_t    )4,              		/* �������ȼ�*/
    //            (TaskHandle_t*  )&xHandleTaskAdcProc );   /* ������  */
#if 0
    /* vTaskTest */
    xTaskCreate((TaskFunction_t )vTaskTest,     		/* ������  */
                (const char*    )"vTaskTest",   		/* ������    */
                (uint16_t       )512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,           		/* �������  */
                (UBaseType_t    )1,              		/* �������ȼ�*/
                (TaskHandle_t*  )NULL );                /* ������  */
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: AppObjCreate
*	����˵��: ��������ͨ�Ż���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppObjCreate (void)
{
	/* ���������ź��� */
    xMutex = xSemaphoreCreateMutex();
    //configASSERT( xMutex );
	if(xMutex == NULL)
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
    }

    xSemaphore_key_interupt = xSemaphoreCreateBinary();
    //configASSERT( xSemaphore_key_interupt );
    if(xSemaphore_key_interupt == NULL)
    {
         /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
    }

    /* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate( 512, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx = xQueueCreate( 512 + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

    /* Create the semaphore used to access the UART Tx. */
	xTxMutex = xSemaphoreCreateMutex();
	//configASSERT( xTxMutex );
}

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
	/*
	  ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
	  �������ĺô��ǣ�
	  1. ��ִֹ�е��жϷ����������FreeRTOS��API������
	  2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
	  3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
	  ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
	  ��cpsie i�ǵ�Ч�ġ�
     */
    __set_PRIMASK(1);

    /* Ӳ����ʼ�� */
	bsp_Init();

	/* 1. ��ʼ��һ����ʱ���жϣ����ȸ��ڵδ�ʱ���жϣ������ſ��Ի��׼ȷ��ϵͳ��Ϣ ��������Ŀ�ģ�ʵ����
		  Ŀ�в�Ҫʹ�ã���Ϊ������ܱȽ�Ӱ��ϵͳʵʱ�ԡ�
	   2. Ϊ����ȷ��ȡFreeRTOS�ĵ�����Ϣ�����Կ��ǽ�����Ĺر��ж�ָ��__set_PRIMASK(1); ע�͵���
	*/
	//vSetupSysInfoTest();

    /* ��������ͨ�Ż��� */
    AppObjCreate();

	/* �������� */
	AppTaskCreate();

    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();

	/*
	  ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
	  heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while(1);
}

/*
*********************************************************************************************************
*	�� �� ��: EXTI2_IRQHandler
*	����˵��: �ⲿ�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EXTI1_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* ����ͬ���ź� */
        xSemaphoreGiveFromISR(xSemaphore_key_interupt, &xHigherPriorityTaskWoken);

        /* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

		//EXTI->IMR&=~(1<<1);	                /* �ر��ж�       */
		EXTI_ClearITPendingBit(EXTI_Line1); /* ����жϱ�־λ */
	}

}

void Debug_Uart_Cli_Handle(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    char cChar;

    if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
    {
        /* The interrupt was caused by the THR becoming empty.  Are there any
        more characters to transmit? */
        if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
        {
            /* A character was retrieved from the queue so can be sent to the THR now. */
            USART_SendData( USART1, cChar );
        }
    }

    if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
    {
        cChar = USART_ReceiveData( USART1 );
        xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
    }

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}
/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
