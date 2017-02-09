/*
*********************************************************************************************************
*
*	ģ������ : STM32�ڲ�RTC
*	�ļ����� : bsp_cpu_rtc.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2015-08-08 armfly  �װ�
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/

