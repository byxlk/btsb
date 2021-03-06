/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : 本实验主要实现FreeRTOS++STemWin+FatFS综合
*              实验目的：
*                1. 学习FreeRTOS++STemWin+FatFS综合
*              实验内容：
*                1. 按下按键K1可以通过串口打印任务执行情况（波特率115200，数据位8，奇偶校验位无，停止位1）
*                   =================================================
*                   任务名      任务状态 优先级   剩余栈 任务序号
*                   vTaskUserIF     R       2       270     2
*                   vTaskGUI        R       1       150     1
*                   IDLE            R       0       114     6
*                   vTaskLED        B       3       484     3
*                   vTaskStart      B       5       490     5
*                   vTaskMsgPro     S       4       480     4
*                   
*                   
*                   任务名       运行计数         使用率
*                   vTaskUserIF     4611            <1%
*                   vTaskGUI        152759          14%
*                   IDLE            884172          83%
*                   vTaskLED        0               <1%
*                   vTaskStart      16259           1%
*                   vTaskMsgPro     1               <1%
*                  串口软件建议使用SecureCRT（V4光盘里面有此软件）查看打印信息。
*                  各个任务实现的功能如下：
*                   vTaskGUI        任务: emWin任务
*                   vTaskTaskUserIF 任务: 接口消息处理	
*                   vTaskLED        任务: LED闪烁
*                   vTaskMsgPro     任务: 实现截图功能，将图片以BMP格式保存到SD卡中
*                   vTaskStart      任务: 启动任务，也就是最高优先级任务，这里实现按键扫描和触摸检测
*                2. 任务运行状态的定义如下，跟上面串口打印字母B, R, D, S对应：
*                    #define tskBLOCKED_CHAR		( 'B' )  阻塞
*                    #define tskREADY_CHAR		    ( 'R' )  就绪
*                    #define tskDELETED_CHAR		( 'D' )  删除
*                    #define tskSUSPENDED_CHAR	    ( 'S' )  挂起
*                3. K2按键按下，实现截图功能，将图片以BMP格式保存到SD卡的PicSave文件夹中。
*              注意事项：
*                1. 本实验推荐使用串口软件SecureCRT，要不串口打印效果不整齐。此软件在
*                   V4开发板光盘里面有。
*                2. 务必将编辑器的缩进参数和TAB设置为4来阅读本文件，要不代码显示不整齐。
*
*	修改记录 :
*		版本号    日期         作者            说明
*       V1.0    2016-03-15   Eric2013    1. ST固件库到V3.6.1版本
*                                        2. BSP驱动包V1.2
*                                        3. FreeRTOS版本V8.2.3
*                                        4. FatFS版本V0.11
*                                        5. STemWin版本V5.28    
*
*	Copyright (C), 2016-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include <includes.h>
#include "MainTask.h"
#include "guitasktest.h"
#include "spi_flash_fatfs.h"
/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/


/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
//static TaskHandle_t xHandleTaskUserIF = NULL;
//static TaskHandle_t xHandleTaskLED = NULL;
//static TaskHandle_t xHandleTaskMsgPro = NULL;
//static TaskHandle_t xHandleTaskStart = NULL;
static TaskHandle_t xHandleTaskAdcProc = NULL;

static SemaphoreHandle_t  xMutex = NULL;

SLEEP_DATA_T gSleep_Data;

/*
*********************************************************************************************************
*	函 数 名: vTaskGUI
*	功能说明: emWin任务
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 1   (数值越小优先级越低，这个跟uCOS相反)
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
*	函 数 名: vTaskAdcProc
*	功能说明: AD转换处理		
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2 
*********************************************************************************************************
*/
static void vTaskAdcProc(void *pvParameters)
{

    //float uwVBATVoltage;    /* 板载电子电压 */
    //float ufVoltage_PA0;    /* PA0引脚电压  */
    //float ufVoltage_PC0;    /* PC0引脚电压  */

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
*	函 数 名: vTaskAdcProc
*	功能说明: AD转换处理		
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2 
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
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
        /* GUI 界面绘制 */
	xTaskCreate(  vTaskGUI,             /* 任务函数  */
                  "vTaskGUI",           /* 任务名    */
                  1024,                 /* 任务栈大小，单位word，也就是4字节 */
                  NULL,                 /* 任务参数  */
                  1,                    /* 任务优先级*/
                  NULL );               /* 任务句柄  */

    /* 按键事件处理 */
    //xTaskCreate( vTaskTaskUserIF,   	/* 任务函数  */
    //             "vTaskUserIF",     	/* 任务名    */
    //             512,               	/* 任务栈大小，单位word，也就是4字节 */
    //             NULL,              	/* 任务参数  */
    //             2,                 	/* 任务优先级*/
    //             &xHandleTaskUserIF );  /* 任务句柄  */
	
	
	//xTaskCreate( vTaskLED,    		/* 任务函数  */
        //         "vTaskLED",  		/* 任务名    */
        //         512,         		/* stack大小，单位word，也就是4字节 */
        //         NULL,        		/* 任务参数  */
        //         3,           		/* 任务优先级*/
        //         &xHandleTaskLED ); /* 任务句柄  */

        /* 截图功能 */
	//xTaskCreate( vTaskMsgPro,     		/* 任务函数  */
        //         "vTaskMsgPro",   		/* 任务名    */
        //         512,             		/* 任务栈大小，单位word，也就是4字节 */
        //         NULL,           		/* 任务参数  */
        //         4,               		/* 任务优先级*/
        //         &xHandleTaskMsgPro );  /* 任务句柄  */
	
	/* 触摸和按键检测 */
	//xTaskCreate( vTaskStart,     		/* 任务函数  */
        //         "vTaskStart",   		/* 任务名    */
        //         512,            		/* 任务栈大小，单位word，也就是4字节 */
        //         NULL,           		/* 任务参数  */
        //         5,              		/* 任务优先级*/
        //         &xHandleTaskStart );   /* 任务句柄  */

    /* ADC 处理函数 */
	xTaskCreate( vTaskAdcProc,     		/* 任务函数  */
                 "vTaskAdcProc",   		/* 任务名    */
                 512,            		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 5,              		/* 任务优先级*/
                 &xHandleTaskAdcProc );   /* 任务句柄  */
    /* vTaskTest */
    xTaskCreate( vTaskTest,     		/* 任务函数  */
                 "vTaskTest",   		/* 任务名    */
                 512,            		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 5,              		/* 任务优先级*/
                 NULL );   /* 任务句柄  */
}

/*
*********************************************************************************************************
*	函 数 名: AppObjCreate
*	功能说明: 创建任务通信机制
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppObjCreate (void)
{
	/* 创建互斥信号量 */
    xMutex = xSemaphoreCreateMutex();
	
	if(xMutex == NULL)
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
	  这样做的好处是：
	  1. 防止执行的中断服务程序中有FreeRTOS的API函数。
	  2. 保证系统正常启动，不受别的中断影响。
	  3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
	  在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
	  和cpsie i是等效的。
     */
	__set_PRIMASK(1);  
	
	/* 硬件初始化 */
	bsp_Init(); 
	
	/* 1. 初始化一个定时器中断，精度高于滴答定时器中断，这样才可以获得准确的系统信息 仅供调试目的，实际项
		  目中不要使用，因为这个功能比较影响系统实时性。
	   2. 为了正确获取FreeRTOS的调试信息，可以考虑将上面的关闭中断指令__set_PRIMASK(1); 注释掉。 
	*/
	//vSetupSysInfoTest();
	
	/* 创建任务 */
	AppTaskCreate();

	/* 创建任务通信机制 */
	AppObjCreate();
	
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

	/* 
	  如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
	  heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while(1);
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
