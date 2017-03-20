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
#include "guitasktest.h"
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
//static TaskHandle_t xHandleTaskLED = NULL;
//static TaskHandle_t xHandleTaskMsgPro = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
static TaskHandle_t xHandleTaskAdcProc = NULL;

static SemaphoreHandle_t  xMutex = NULL;

SLEEP_DATA_T gSleep_Data;

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
	while (1) 
	{
		//MainTask();
		vTaskDelay(1000);
	}
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

    while(1)
    {
        vTaskDelay(1000);
        //uwVBATVoltage = ADC_ConvertedValue[1] * 3.3 / 4095;
        //ufVoltage_PA0 = ADC_ConvertedValue[2] * 3.3 / 4095;
        //ufVoltage_PC0 = ADC_ConvertedValue[3] * 3.3 / 4095;
       // GetTemp(ADC_ConvertedValue[0]);
    }
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
	

    while(1)
    {
        /* �����˲��ͼ���ɺ�̨systick�жϷ������ʵ�֣�����ֻ��Ҫ����bsp_GetKey��ȡ��ֵ���ɡ� */
		ucKeyCode = bsp_GetKey();  /* bsp_GetKey()��ȡ��ֵ, �޼�����ʱ���� KEY_NONE = 0 */
		
		if (ucKeyCode != KEY_NONE)
		{
		    //printf("Press Key %d\r\n",ucKeyCode);
			switch (ucKeyCode)
			{
				/* K1������ ��ӡ����ִ����� */
				case KEY_DOWN_DEBUG:			 
					printf("=================================================\r\n");
					printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
					vTaskList((char *)&pcWriteBuffer);
					printf("%s\r\n", pcWriteBuffer);
				
					printf("\r\n������       ���м���         ʹ����\r\n");
					vTaskGetRunTimeStats((char *)&pcWriteBuffer);
					printf("%s\r\n", pcWriteBuffer);
					printf("��ǰ��̬�ڴ�ʣ���С = %d�ֽ�\r\n", xPortGetFreeHeapSize());
					break;

				case KEY_DOWN_MUX:
                    break;

				/* K2�����£�ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD���� */
				case KEY_DOWN_VOL0:
                case KEY_DOWN_VOL1:
                case KEY_DOWN_VOL2:
                case KEY_DOWN_VOL3:
                case KEY_DOWN_VOL4:
                case KEY_DOWN_VOL5:
					//xTaskNotifyGive(xHandleTaskMsgPro);
					break;
                
                case KEY_DOWN_PLAY_PAUSE:
                    break; 

                case KEY_DOWN_MENU:
                    break;

                case KEY_DOWN_UP:
                    break;

                case KEY_DOWN_DOWN:
                    break;

				/* �����ļ�ֵ������ */
				default:                     
					break;
			}
		}
		
		vTaskDelay(20);
	}
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

    tick_backup = xTaskGetTickCount();
    while(1)
    {		
        tick_current = xTaskGetTickCount();
		/* 10msһ�ΰ������ */

		if(tick_current - tick_backup >= 10)
		{
		    tick_backup = tick_current;
			bsp_TouchKeyScan();
		}

		vTaskDelay(5);	
	}
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
    while(1)
    {
        vTaskDelay(100);
  #ifdef LCD_DRIVER_TEST
        LCD_Fill_Rect(0, 0, 320, 240, CL_BLUE);
        vTaskDelay(1000);
        LCD_Fill_Rect(0, 0, 320, 240, CL_YELLOW);
   #endif
        GuiTaskTest();
        //bsp_RTC_Test();
        //DemoFatFS();
    }  
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
	xTaskCreate(  vTaskGUI,             /* ������  */
                  "vTaskGUI",           /* ������    */
                  1024,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                  NULL,                 /* �������  */
                  1,                    /* �������ȼ�*/
                  NULL );               /* ������  */

    /* �����¼����� */
    xTaskCreate( vTaskTaskUserKeyIF,   	/* ������  */
                 "vTaskUserKeyIF",     	/* ������    */
                 512,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 2,                 	/* �������ȼ�*/
                 &xHandleTaskUserKeyIF );  /* ������  */
	
	
	//xTaskCreate( vTaskLED,    		/* ������  */
        //         "vTaskLED",  		/* ������    */
        //         512,         		/* stack��С����λword��Ҳ����4�ֽ� */
        //         NULL,        		/* �������  */
        //         3,           		/* �������ȼ�*/
        //         &xHandleTaskLED ); /* ������  */

        /* ��ͼ���� */
	//xTaskCreate( vTaskMsgPro,     		/* ������  */
        //         "vTaskMsgPro",   		/* ������    */
        //         512,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
        //         NULL,           		/* �������  */
        //         4,               		/* �������ȼ�*/
        //         &xHandleTaskMsgPro );  /* ������  */
	
	/* �����Ͱ������ */
	xTaskCreate( vTaskStart,     		/* ������  */
                 "vTaskStart",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 5,              		/* �������ȼ�*/
                 &xHandleTaskStart );   /* ������  */

    /* ADC ������ */
	xTaskCreate( vTaskAdcProc,     		/* ������  */
                 "vTaskAdcProc",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 5,              		/* �������ȼ�*/
                 &xHandleTaskAdcProc );   /* ������  */
    /* vTaskTest */
    xTaskCreate( vTaskTest,     		/* ������  */
                 "vTaskTest",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 5,              		/* �������ȼ�*/
                 NULL );   /* ������  */
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
	
	if(xMutex == NULL)
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
	
	/* Ӳ����ʼ�� */
	bsp_Init(); 
	
	/* 1. ��ʼ��һ����ʱ���жϣ����ȸ��ڵδ�ʱ���жϣ������ſ��Ի��׼ȷ��ϵͳ��Ϣ ��������Ŀ�ģ�ʵ����
		  Ŀ�в�Ҫʹ�ã���Ϊ������ܱȽ�Ӱ��ϵͳʵʱ�ԡ�
	   2. Ϊ����ȷ��ȡFreeRTOS�ĵ�����Ϣ�����Կ��ǽ�����Ĺر��ж�ָ��__set_PRIMASK(1); ע�͵��� 
	*/
	//vSetupSysInfoTest();
	
	/* �������� */
	AppTaskCreate();

	/* ��������ͨ�Ż��� */
	AppObjCreate();
	
    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();

	/* 
	  ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
	  heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while(1);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
