/*
*********************************************************************************************************
*
*	模块名称 : STM32内部RTC模块
*	文件名称 : bsp_cpu_rtc.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2015-08-08 armfly  首版.安富莱电子原创
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

#define RTC_Debug   /* 用于选择调试模式 */

/* 选择RTC的时钟源 */
#define RTC_CLOCK_SOURCE_LSE       /* LSE */
//#define RTC_CLOCK_SOURCE_LSI     /* LSI */ 

RTC_T g_tRTC;

/* 平年的每月天数表 */
const uint8_t mon_table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*
*********************************************************************************************************
*	函 数 名: bsp_InitRTC
*	功能说明: 初始化CPU内部RTC
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitRTC(void)
{
     uint16_t u16_WaitForOscSource = 0;
     RTC_InitTypeDef   RTC_InitStructure;

     /* Enable the PWR clock */ /* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */  
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_AHB1Periph_BKPSRAM, ENABLE);

     /* Allow access to BKP Domain */ /*使能RTC和后备寄存器访问 */  
     PWR_BackupAccessCmd(ENABLE);

     /*
     我们在BKP的后备寄存器1中，存了一个特殊字符0xA5A5, 第一次上电或后备电源掉电后，该寄存器数据丢失，
     表明RTC数据丢失，需要重新配置
     */
    if (RTC_ReadBackupRegister(RTC_BKP_DR1) != 0xA5A5)
    {
        //重新配置RTC
	/* Calendar Configuration */
	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
	RTC_InitStructure.RTC_SynchPrediv =  0xFF;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
    
        /* Enable LSE */		
        RCC_LSEConfig(RCC_LSE_ON);
        //while ((u16_WaitForOscSource++) < 5000) ;

        /* Wait till LSE is ready */ /* 等待外部晶振震荡稳定输出 */  
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

        /* Select LSE as RTC Clock Source */ /*使用外部32.768KHz晶振作为RTC时钟 */ 
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        /* Enable the RTC Second */
        //RTC_ITConfig(RTC_IT_TS, ENABLE);

        /* Default date: 2016-01-01 default time: 08:00:00 */
        bsp_RTC_SetDate(2016, RTC_Month_February, 29);//默认时间
        bsp_RTC_SetTime(23, 58, 0);

        /* Configure the RTC Wakeup Clock source and Counter (Wakeup event each 1 second) */
        RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
        RTC_SetWakeUpCounter(0x7FF);

        /* Enable the Wakeup Interrupt */
        RTC_ITConfig(RTC_IT_WUT, ENABLE);
 
        /* Enable Wakeup Counter */
        RTC_WakeUpCmd(ENABLE);
		
        /* 配置完成后，向后备寄存器中写特殊字符0xA5A5 */
        RTC_WriteBackupRegister(RTC_BKP_DR1, 0xA5A5);
    }
    else
    {
        while ((u16_WaitForOscSource++) < 5000) ;
        
        /* Wait for RTC APB registers synchronisation */
        RTC_WaitForSynchro();
        RTC_ClearITPendingBit(RTC_IT_WUT);
        
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
			/* 上电复位 */
        }
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            /* 外部RST管脚复位 */
        }
        /* 清除RCC中复位标志 */
        RCC_ClearFlag();
        
        //虽然RTC模块不需要重新配置，且掉电后依靠后备电池依然运行
        //但是每次上电后，还是要使能RTCCLK???????
        //RCC_RTCCLKCmd(ENABLE);
        //等待RTC时钟与APB1时钟同步
        RTC_WaitForSynchro();
        
        //使能秒中断
        //RTC_ITConfig(RTC_IT_SEC, ENABLE);
        }
	return;
}

/*
*********************************************************************************************************
*	函 数 名: Is_Leap_Year
*	功能说明: 判断是否为闰年
*	形    参：无
*	返 回 值: 1,是.0,不是
*********************************************************************************************************
*/
uint8_t IS_RTC_LeapYear(uint16_t _year)
{                     
	if (_year % 4 == 0) /* 必须能被4整除 */
	{ 
		if (_year % 100 == 0) 
		{ 
			if (_year % 400 == 0)
			{
				return 1;	/* 如果以00结尾,还要能被400整除 */
			}
			else 
			{
				return 0;   
			}

		}
		else 
		{
			return 1;   
		}
	}
	else 
	{
		return 0; 
	}
}      

void bsp_RTC_SetTime(uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	RTC_TimeTypeDef   RTC_TimeStructure;

	/* Set the Time */
	RTC_TimeStructure.RTC_Hours   = IS_RTC_HOUR24(_hour)? _hour : 0x08;
	RTC_TimeStructure.RTC_Minutes = IS_RTC_MINUTES(_min)? _min : 0x00;
	RTC_TimeStructure.RTC_Seconds = IS_RTC_SECONDS(_sec)? _sec : 0x00;
    
	/* Set Current Time and Date */
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
}

void bsp_RTC_SetDate(uint16_t _year, uint8_t _mon, uint8_t _day)
{
	RTC_DateTypeDef   RTC_DateStructure;

	/* Set the Date */
	RTC_DateStructure.RTC_Year = IS_RTC_YEAR(_year - 2000)? (_year - 2000) : 16; 
	RTC_DateStructure.RTC_Month = IS_RTC_MONTH(_mon)? _mon : RTC_Month_January;
	RTC_DateStructure.RTC_Date = IS_RTC_DATE(_day)? _day : 0x01;  
	RTC_DateStructure.RTC_WeekDay = bsp_RTC_CalcWeek(RTC_DateStructure.RTC_Year,
                                                                                                  RTC_DateStructure.RTC_Month,
                                                                                                  RTC_DateStructure.RTC_Date); 
    
	/* Set Current Time and Date */
	RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
}

/*
*********************************************************************************************************
*	函 数 名: RTC_WriteClock
*	功能说明: 设置RTC时时间
*	形    参：无
*	返 回 值: 1表示成功 0表示错误
*********************************************************************************************************
*/
uint32_t bsp_RTC_GetSecond(uint16_t _year, uint8_t _mon, uint8_t _day, uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	uint16_t t;
	uint32_t seccount=0;


	if (_year < 2000 || _year > 2099)
	{
		return 0;	/* _year范围1970-2099，此处设置范围为2000-2099 */   
	}		
	
	for (t = 1970; t < _year; t++) 	/* 把所有年份的秒钟相加 */
	{
		if (IS_RTC_LeapYear(t))		/* 判断是否为闰年 */
		{
			seccount += 31622400;	/* 闰年的秒钟数 */
		}
		else
		{
			seccount += 31536000; 	/* 平年的秒钟数 */
		}
	}

	_mon -= 1;

	for (t = 0; t < _mon; t++)         /* 把前面月份的秒钟数相加 */
	{
		seccount += (uint32_t)mon_table[t] * 86400;	/* 月份秒钟数相加 */

		if (IS_RTC_LeapYear(_year) && t == 1)
		{
			seccount += 86400;	/* 闰年2月份增加一天的秒钟数 */
		}			
	}

	seccount += (uint32_t)(_day - 1) * 86400;	/* 把前面日期的秒钟数相加 */

	seccount += (uint32_t)_hour * 3600;		/* 小时秒钟数 */

	seccount += (uint32_t)_min * 60;	/* 分钟秒钟数 */

	seccount += _sec;	/* 最后的秒钟加上去 */
       
	return seccount;      
}

/*
*********************************************************************************************************
*	函 数 名: RTC_ReadClock
*	功能说明: 得到当前时钟。结果存放在 g_tRTC。
*	形    参：无
*	返 回 值: 1表示成功 0表示错误
*********************************************************************************************************
*/
void bsp_RTC_ReadClock(RTC_DateTypeDef *pDate, RTC_TimeTypeDef   *pTime)
{

    if(! pDate)
        RTC_GetDate(RTC_Format_BIN, pDate);

    if(! pTime)
        RTC_GetTime(RTC_Format_BIN, pTime);
}   

void bsp_RTC_GetClock(void)
{
    RTC_DateTypeDef   RTC_DateStructure; 
    RTC_TimeTypeDef   RTC_TimeStructure;

    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

    g_tRTC.Year = RTC_DateStructure.RTC_Year+2000;
    g_tRTC.Mon = RTC_DateStructure.RTC_Month;
    g_tRTC.Day = RTC_DateStructure.RTC_Date;
    g_tRTC.Week = RTC_DateStructure.RTC_WeekDay;

    g_tRTC.Hour = RTC_TimeStructure.RTC_Hours;
    g_tRTC.Min = RTC_TimeStructure.RTC_Minutes;
    g_tRTC.Sec = RTC_TimeStructure.RTC_Seconds;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_CalcWeek
*	功能说明: 根据日期计算星期几
*	形    参: _year _mon _day  年月日  (年是2字节整数，月和日是字节整数）
*	返 回 值: 周几 （1-7） 7表示周日
*********************************************************************************************************
*/
uint8_t bsp_RTC_CalcWeek(uint16_t _year, uint8_t _mon, uint8_t _day)
{
	/*
	蔡勒（Zeller）公式
		历史上的某一天是星期几？未来的某一天是星期几？关于这个问题，有很多计算公式（两个通用计算公式和
	一些分段计算公式），其中最著名的是蔡勒（Zeller）公式。
	    即w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1

		公式中的符号含义如下，
	     w：星期；
	     c：年的高2位，即世纪-1
	     y：年（两位数）；
	     m：月（m大于等于3，小于等于14，即在蔡勒公式中，某年的1、2月要看作上一年的13、14月来计算，
	  	    比如2003年1月1日要看作2002年的13月1日来计算）；
	     d：日；
	     [ ]代表取整，即只要整数部分。

	    算出来的W除以7，余数是几就是星期几。如果余数是0，则为星期日。
        如果结果是负数，负数求余数则需要特殊处理：
            负数不能按习惯的余数的概念求余数，只能按数论中的余数的定义求余。为了方便
        计算，我们可以给它加上一个7的整数倍，使它变为一个正数，然后再求余数

		以2049年10月1日（100周年国庆）为例，用蔡勒（Zeller）公式进行计算，过程如下：
		蔡勒（Zeller）公式：w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1
		=49+[49/4]+[20/4]-2×20+[26× (10+1)/10]+1-1
		=49+[12.25]+5-40+[28.6]
		=49+12+5-40+28
		=54 (除以7余5)
		即2049年10月1日（100周年国庆）是星期5。
	*/
	uint8_t y, c, m, d;
	int16_t w;

	if (_mon >= 3)
	{
		m = _mon;
		y = _year % 100;
		c = _year / 100;
		d = _day;
	}
	else	/* 某年的1、2月要看作上一年的13、14月来计算 */
	{
		m = _mon + 12;
		y = (_year - 1) % 100;
		c = (_year - 1) / 100;
		d = _day;
	}

	w = y + y / 4 +  c / 4 - 2 * c + ((uint16_t)26*(m+1))/10 + d - 1;
	if (w == 0)
	{
		w = 7;	/* 表示周日 */
	}
	else if (w < 0)	/* 如果w是负数，则计算余数方式不同 */
	{
		w = 7 - (-w) % 7;
	}
	else
	{
		w = w % 7;
	}

        if(w== 0) return RTC_Weekday_Sunday;
        else	return w;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
