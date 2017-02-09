#include "bsp.h"
#include "stm32f2xx.h"


//void SystemClockInit(void)
//{
//    RCC_DeInit();
//    RCC_HSICmd(ENABLE);
//    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//等待HSI使能成功


//}

/*
*********************************************************************************************************
*	函 数 名: bsp_Init
*	功能说明: 初始化硬件设备。只需要调用一次。该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量。
*			 全局变量。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Init(void)
{
	/*
	 * 由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
	 * 启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。
      *
	 * 系统时钟缺省配置为72MHz，如果需要更改，可以修改 system_stm32f10x.c 文件
	*/

	/* 使能CRC校验, 用于开启STemWin的使用 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
	/* 优先级分组设置为4，可配置0-15级抢占式优先级，0级子优先级，即不存在子优先级。*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	bsp_InitDWT();       /* 初始DWT */
	bsp_RTC_InitConfig(); /* 初始化RTC模块，配置默认时间:2016-01-01 08:00:00 */
	bsp_InitUart(); 	 /* 初始化串口 */
	//bsp_InitLed(); 		 /* 初始LED指示灯端口 */
	//bsp_InitKey();		 /* 初始化按键 */
	
	//bsp_InitI2C();       /* 配置I2C总线 */
	
	//bsp_InitSPIBus();	 /* 配置SPI总线 */
	
	//bsp_InitExtSRAM();   /* 初始DWT */
	LCD_InitHard();	     /* 初始化显示器硬件(配置GPIO和FSMC,给LCD发送初始化指令) */
	//TOUCH_InitHard();    /* 初始化触摸 */
	
	/* 挂载文件系统 */
	//result = f_mount(&fs, "0:/", 0);
	
}

