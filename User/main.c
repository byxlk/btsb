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
#include "MainTask.h"
#include "spi_flash_fatfs.h"
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
static TaskHandle_t xHandleTaskFsDebug = NULL;
//static TaskHandle_t xHandleTaskMsgPro = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
static TaskHandle_t xHandleTaskAdcProc = NULL;

static SemaphoreHandle_t  xMutex = NULL;
static SemaphoreHandle_t  xSemaphore_key_interupt = NULL;

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
    vTaskDelete( NULL );

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
static void vTaskAdcProc(void *pvParameters)
{

    //float uwVBATVoltage;    /* ���ص��ӵ�ѹ */
    //float ufVoltage_PA0;    /* PA0���ŵ�ѹ  */
    //float ufVoltage_PC0;    /* PC0���ŵ�ѹ  */
    printf("ADC capture Thread start.\r\n");
    while(1)
    {
        vTaskDelay(1000);
        //uwVBATVoltage = ADC_ConvertedValue[1] * 3.3 / 4095;
        //ufVoltage_PA0 = ADC_ConvertedValue[2] * 3.3 / 4095;
        //ufVoltage_PC0 = ADC_ConvertedValue[3] * 3.3 / 4095;
       // GetTemp(ADC_ConvertedValue[0]);
    }

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    vTaskDelete( NULL );
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskTaskUserIF
*	����˵��: ������Ϣ����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2
*********************************************************************************************************
*/
static void vTaskTaskUserKeyIF(void *pvParameters)
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
    vTaskDelete( NULL );

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
    vTaskDelete( NULL );
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
static void vTaskFsDebug(void *pvParameters)
{
    printf("vTaskFsDebug Thread start.\r\n");

    DemoFatFS();

    /* �������ľ���ʵ�ֻ������������ѭ���������������ں���������֮ǰɾ����
    ����NULL������ʾɾ�� ���ǵ�ǰ���� */
    vTaskDelete( NULL );
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
    vTaskDelete( NULL );
}
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
    /* GUI ������� */
	xTaskCreate((TaskFunction_t )vTaskGUI,             /* ������  */
                (const char*    )"vTaskGUI",           /* ������    */
                (uint16_t       )1024,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,                 /* �������  */
                (UBaseType_t    )3,                    /* �������ȼ�*/
                (TaskHandle_t*  )NULL );               /* ������  */

    /* �����¼����� */
    xTaskCreate((TaskFunction_t )vTaskTaskUserKeyIF,   	/* ������  */
                (const char*    )"vTaskUserKeyIF",     	/* ������    */
                (uint16_t       )512,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,              	/* �������  */
                (UBaseType_t    )4,                 	/* �������ȼ�*/
                (TaskHandle_t*  )&xHandleTaskUserKeyIF );  /* ������  */


	xTaskCreate((TaskFunction_t )vTaskFsDebug,    		/* ������  */
                (const char*    )"vTaskFsDebug",  		/* ������    */
                (uint16_t       )1024,         		/* stack��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,        		/* �������  */
                (UBaseType_t    )2,           		/* �������ȼ�*/
                (TaskHandle_t*  )&xHandleTaskFsDebug ); /* ������  */

	/* �����Ͱ������ */
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

    /* vTaskTest */
    xTaskCreate((TaskFunction_t )vTaskTest,     		/* ������  */
                (const char*    )"vTaskTest",   		/* ������    */
                (uint16_t       )512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                (void*          )NULL,           		/* �������  */
                (UBaseType_t    )1,              		/* �������ȼ�*/
                (TaskHandle_t*  )NULL );   /* ������  */
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

    vUARTCommandConsoleStart(1024, 1);
    vRegisterSampleCLICommands();

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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
