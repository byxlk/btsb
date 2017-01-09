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
#include "includes.h"
#include "MainTask.h"


/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static void vTaskGUI(void *pvParameters);
static void vTaskTaskUserIF(void *pvParameters);
//static void vTaskLED(void *pvParameters);
//static void vTaskMsgPro(void *pvParameters);
//static void vTaskStart(void *pvParameters);
static void AppTaskCreate (void);
static void AppObjCreate (void);
static void  App_Printf(char *format, ...);

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUserIF = NULL;
//static TaskHandle_t xHandleTaskLED = NULL;
static TaskHandle_t xHandleTaskMsgPro = NULL;
//static TaskHandle_t xHandleTaskStart = NULL;
static SemaphoreHandle_t  xMutex = NULL;

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
	vSetupSysInfoTest();
	
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
		MainTask();
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
static void vTaskTaskUserIF(void *pvParameters)
{
	uint8_t ucKeyCode;
	uint8_t pcWriteBuffer[500];
	

    while(1)
    {
		ucKeyCode = bsp_GetKey();
		
		if (ucKeyCode != KEY_NONE)
		{
			switch (ucKeyCode)
			{
				/* K1������ ��ӡ����ִ����� */
				case KEY_DOWN_K1:			 
					App_Printf("=================================================\r\n");
					App_Printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
					vTaskList((char *)&pcWriteBuffer);
					App_Printf("%s\r\n", pcWriteBuffer);
				
					App_Printf("\r\n������       ���м���         ʹ����\r\n");
					vTaskGetRunTimeStats((char *)&pcWriteBuffer);
					App_Printf("%s\r\n", pcWriteBuffer);
					printf("��ǰ��̬�ڴ�ʣ���С = %d�ֽ�\r\n", xPortGetFreeHeapSize());
					break;
				
				/* K2�����£�ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD���� */
				case KEY_DOWN_K2:
					xTaskNotifyGive(xHandleTaskMsgPro);
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
*	�� �� ��: vTaskLED
*	����˵��: LED��˸
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 3 
*********************************************************************************************************
*/
#if 0  //not need
static void vTaskLED(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200;

	/* ��ȡ��ǰ��ϵͳʱ�� */
    xLastWakeTime = xTaskGetTickCount();
	
    while(1)
    {
       	bsp_LedToggle(2);
		bsp_LedToggle(3);
		
		/* vTaskDelayUntil�Ǿ����ӳ٣�vTaskDelay������ӳ١�*/
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskMsgPro
*	����˵��: ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskMsgPro(void *pvParameters)
{
	uint8_t	 Pic_Name = 0;
	uint32_t ulStart, ulEnd;
	char buf[20];
	
    while(1)
    {
		ulTaskNotifyTake( pdTRUE,          /* �˲�������ΪpdTRUE�����յ���notification value���� */
						  portMAX_DELAY ); /* ���޵ȴ� */
		
		sprintf(buf,"0:/PicSave/%d.bmp",Pic_Name);
		
		/* ��¼��ͼǰ��ʼʱ�� */
		ulStart = xTaskGetTickCount();
		
		/* ���������� */	
		vTaskSuspendAll();
		
		/* ���SD����û��PicSave�ļ�������д��� */
		result = f_mkdir("0:/PicSave");
		/* ������ͼ */
		result = f_open(&file, buf, FA_WRITE|FA_CREATE_ALWAYS);
		/* ��SD������BMPͼƬ */
		GUI_BMP_Serialize(_WriteByte2File, &file);
		
		/* ������ɺ�ر�file */
		result = f_close(&file);
		
		/* �رյ����� */	
		xTaskResumeAll ();
		
		/* ��¼��ͼ��ʱ�䲢��ȡ��ͼ���̺�ʱ */
		ulEnd = xTaskGetTickCount();
		ulEnd -= ulStart;
		
		App_Printf("��ͼ��ɣ���ʱ = %dms\r\n", ulEnd);
		Pic_Name++; 	
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
	uint8_t  ucCount = 0;
	uint8_t  ucCount1 = 0;
	
    while(1)
    {
		/* 1msһ�δ���ɨ�裬���败���� */
		if(g_tTP.Enable == 1)
		{
			TOUCH_Scan();	
		}
		/* 10msһ�δ���ɨ�裬���ݴ����� */
		else
		{
			ucCount1++;
			if(ucCount1 == 10)
			{
				ucCount1 = 0;
				GT811_OnePiontScan();
			}
		}

		/* 10msһ�ΰ������ */
		ucCount++;
		if(ucCount == 10)
		{
			ucCount = 0;
			bsp_KeyScan();
		}

		vTaskDelay(1);
    }
}
#endif				
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
    xTaskCreate( vTaskTaskUserIF,   	/* ������  */
                 "vTaskUserIF",     	/* ������    */
                 512,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 2,                 	/* �������ȼ�*/
                 &xHandleTaskUserIF );  /* ������  */
	
	
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
	//xTaskCreate( vTaskStart,     		/* ������  */
        //         "vTaskStart",   		/* ������    */
        //         512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
        //         NULL,           		/* �������  */
        //         5,              		/* �������ȼ�*/
        //         &xHandleTaskStart );   /* ������  */
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
*	�� �� ��: App_Printf
*	����˵��: �̰߳�ȫ��printf��ʽ		  			  
*	��    ��: ͬprintf�Ĳ�����
*             ��C�У����޷��г����ݺ���������ʵ�ε����ͺ���Ŀʱ,������ʡ�Ժ�ָ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void  App_Printf(char *format, ...)
{
    char  buf_str[200 + 1];
    va_list   v_args;


    va_start(v_args, format);
   (void)vsnprintf((char       *)&buf_str[0],
                   (size_t      ) sizeof(buf_str),
                   (char const *) format,
                                  v_args);
    va_end(v_args);

	/* �����ź��� */
	xSemaphoreTake(xMutex, portMAX_DELAY);

    printf("%s", buf_str);

   	xSemaphoreGive(xMutex);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
