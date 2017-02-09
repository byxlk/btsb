/*
*********************************************************************************************************
*
*	模块名称 : STM32内部RTC
*	文件名称 : bsp_cpu_rtc.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2015-08-08 armfly  首版
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_CPU_RTC_H
#define __BSP_CPU_RTC_H
#include "stm32f2xx.h"

typedef struct
{
	uint16_t Year;
	uint8_t Mon;
	uint8_t Day;	
	uint8_t Hour;		
	uint8_t Min;				
	uint8_t Sec;					
	uint8_t Week;	
}RTC_T;

extern RTC_T g_tRTC;


void bsp_RTC_InitConfig(void);
uint8_t IS_RTC_LeapYear(uint16_t _year);
void bsp_RTC_SetTime(uint8_t _hour, uint8_t _min, uint8_t _sec);
void bsp_RTC_SetDate(uint16_t _year, uint8_t _mon, uint8_t _day);
uint32_t bsp_RTC_GetSecond(uint16_t _year, uint8_t _mon, uint8_t _day, uint8_t _hour, uint8_t _min, uint8_t _sec);
void bsp_RTC_ReadClock(RTC_DateTypeDef *pDate, RTC_TimeTypeDef   *pTime);
uint8_t bsp_RTC_CalcWeek(uint16_t _year, uint8_t _mon, uint8_t _day);
void bsp_RTC_GetClock(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

