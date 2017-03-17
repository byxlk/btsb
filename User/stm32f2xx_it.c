/*
*********************************************************************************************************
*	                                  
*	模块名称 : 中断模块
*	文件名称 : stm32f10x_it.c
*	版    本 : V2.0
*	说    明 : 本文件存放所有的中断服务函数。为了便于他人了解程序用到的中断，我们不建议将中断函数移到其他
*			的文件。
*			
*			我们只需要添加需要的中断函数即可。一般中断函数名是固定的，除非您修改了启动文件：
*				Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup\arm\startup_stm32f10x_hd.s
*			
*			启动文件是汇编语言文件，定了每个中断的服务函数，这些函数使用了WEAK 关键字，表示弱定义，因此如
*			果我们在c文件中重定义了该服务函数（必须和它同名），那么启动文件的中断函数将自动无效。这也就
*			函数重定义的概念，这和C++中的函数重载的意义类似。
*				
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2013-02-01 armfly  首版
*		v1.0    2013-06-20 armfly  在硬件异常中断服务程序中增加打印错误信息的功能。
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f2xx_it.h"
#include <stdio.h>

#define ERR_INFO "\r\nEnter HardFault_Handler, System Halt.\r\n"

/*
*********************************************************************************************************
*	Cortex-M3 内核异常中断服务程序
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: NMI_Handler
*	功能说明: 不可屏蔽中断服务程序。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void NMI_Handler(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: HardFault_Handler
*	功能说明: 硬件故障中断服务程序。其他异常处理被关闭，而又发生了异常，则触发。
*			  执行异常处理时，发生了异常，则触发。复位时默认使能。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void HardFault_Handler(void)
{
#if 1
  const char *pError = ERR_INFO;
  uint8_t i;

  for (i = 0; i < sizeof(ERR_INFO); i++)
  {
     USART1->DR = pError[i];
     /* 等待发送结束 */
     while ((USART1->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif

  /* 当硬件失效异常发生时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: MemManage_Handler
*	功能说明: 内存管理异常中断服务程序。违反MPU设定的存储器访问规则时触发。 复位时默认未使能
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MemManage_Handler(void)
{
  /* 当内存管理异常发生时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: BusFault_Handler
*	功能说明: 总线访问异常中断服务程序。取指令、数据读写、堆栈操作出现异常。 复位时默认未使能
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void BusFault_Handler(void)
{
  /* 当总线异常时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: UsageFault_Handler
*	功能说明: 用法错误中断服务程序。执行未定义指令、非对齐操作、除零时触发。 复位时默认未使能
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void UsageFault_Handler(void)
{
  /* 当用法异常时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: DebugMon_Handler
*	功能说明: 调试监视器中断服务程序。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void DebugMon_Handler(void)
{
}

/*
*********************************************************************************************************
*	STM32F10x内部外设中断服务程序
*	用户在此添加用到外设中断服务函数。有效的中断服务函数名请参考启动文件(startup_stm32f10x_xx.s)
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: PPP_IRQHandler
*	功能说明: 外设中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/    
/* 
	因为中断服务程序往往和具体的应用有关，会用到用户功能模块的变量、函数。如果在本文件展开，会增加大量的
	外部变量声明或者include语句。
	
	因此，我们推荐这个地方只写一个调用语句，中断服务函数的本体放到对应的用户功能模块中。
	增加一层调用会降低代码的执行效率，不过我们宁愿损失这个效率，从而增强程序的模块化特性。
	
	增加extern关键字，直接引用用到的外部函数，避免在文件头include其他模块的头文件
extern void ppp_ISR(void);	
void PPP_IRQHandler(void)
{
	ppp_ISR();
}
*/
