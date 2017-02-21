/*
*********************************************************************************************************
*
*	ģ������ : STM32�ڲ�RTCģ��
*	�ļ����� : bsp_cpu_rtc.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2015-08-08 armfly  �װ�.����������ԭ��
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

#define RTC_Debug   /* ����ѡ�����ģʽ */

/* ѡ��RTC��ʱ��Դ */
#define RTC_CLOCK_SOURCE_LSE       /* LSE */
//#define RTC_CLOCK_SOURCE_LSI     /* LSI */ 

RTC_T g_tRTC;

/* ƽ���ÿ�������� */
const uint8_t mon_table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitRTC
*	����˵��: ��ʼ��CPU�ڲ�RTC
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitRTC(void)
{
     uint16_t u16_WaitForOscSource = 0;
     RTC_InitTypeDef   RTC_InitStructure;

     /* Enable the PWR clock */ /* PWRʱ�ӣ���Դ���ƣ���BKPʱ�ӣ�RTC�󱸼Ĵ�����ʹ�� */  
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_AHB1Periph_BKPSRAM, ENABLE);

     /* Allow access to BKP Domain */ /*ʹ��RTC�ͺ󱸼Ĵ������� */  
     PWR_BackupAccessCmd(ENABLE);

     /*
     ������BKP�ĺ󱸼Ĵ���1�У�����һ�������ַ�0xA5A5, ��һ���ϵ��󱸵�Դ����󣬸üĴ������ݶ�ʧ��
     ����RTC���ݶ�ʧ����Ҫ��������
     */
    if (RTC_ReadBackupRegister(RTC_BKP_DR1) != 0xA5A5)
    {
        //��������RTC
	/* Calendar Configuration */
	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
	RTC_InitStructure.RTC_SynchPrediv =  0xFF;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
    
        /* Enable LSE */		
        RCC_LSEConfig(RCC_LSE_ON);
        //while ((u16_WaitForOscSource++) < 5000) ;

        /* Wait till LSE is ready */ /* �ȴ��ⲿ�������ȶ���� */  
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

        /* Select LSE as RTC Clock Source */ /*ʹ���ⲿ32.768KHz������ΪRTCʱ�� */ 
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        /* Enable the RTC Second */
        //RTC_ITConfig(RTC_IT_TS, ENABLE);

        /* Default date: 2016-01-01 default time: 08:00:00 */
        bsp_RTC_SetDate(2016, RTC_Month_February, 29);//Ĭ��ʱ��
        bsp_RTC_SetTime(23, 58, 0);

        /* Configure the RTC Wakeup Clock source and Counter (Wakeup event each 1 second) */
        RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
        RTC_SetWakeUpCounter(0x7FF);

        /* Enable the Wakeup Interrupt */
        RTC_ITConfig(RTC_IT_WUT, ENABLE);
 
        /* Enable Wakeup Counter */
        RTC_WakeUpCmd(ENABLE);
		
        /* ������ɺ���󱸼Ĵ�����д�����ַ�0xA5A5 */
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
			/* �ϵ縴λ */
        }
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            /* �ⲿRST�ܽŸ�λ */
        }
        /* ���RCC�и�λ��־ */
        RCC_ClearFlag();
        
        //��ȻRTCģ�鲻��Ҫ�������ã��ҵ���������󱸵����Ȼ����
        //����ÿ���ϵ�󣬻���Ҫʹ��RTCCLK???????
        //RCC_RTCCLKCmd(ENABLE);
        //�ȴ�RTCʱ����APB1ʱ��ͬ��
        RTC_WaitForSynchro();
        
        //ʹ�����ж�
        //RTC_ITConfig(RTC_IT_SEC, ENABLE);
        }
	return;
}

/*
*********************************************************************************************************
*	�� �� ��: Is_Leap_Year
*	����˵��: �ж��Ƿ�Ϊ����
*	��    �Σ���
*	�� �� ֵ: 1,��.0,����
*********************************************************************************************************
*/
uint8_t IS_RTC_LeapYear(uint16_t _year)
{                     
	if (_year % 4 == 0) /* �����ܱ�4���� */
	{ 
		if (_year % 100 == 0) 
		{ 
			if (_year % 400 == 0)
			{
				return 1;	/* �����00��β,��Ҫ�ܱ�400���� */
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
*	�� �� ��: RTC_WriteClock
*	����˵��: ����RTCʱʱ��
*	��    �Σ���
*	�� �� ֵ: 1��ʾ�ɹ� 0��ʾ����
*********************************************************************************************************
*/
uint32_t bsp_RTC_GetSecond(uint16_t _year, uint8_t _mon, uint8_t _day, uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	uint16_t t;
	uint32_t seccount=0;


	if (_year < 2000 || _year > 2099)
	{
		return 0;	/* _year��Χ1970-2099���˴����÷�ΧΪ2000-2099 */   
	}		
	
	for (t = 1970; t < _year; t++) 	/* ��������ݵ�������� */
	{
		if (IS_RTC_LeapYear(t))		/* �ж��Ƿ�Ϊ���� */
		{
			seccount += 31622400;	/* ����������� */
		}
		else
		{
			seccount += 31536000; 	/* ƽ��������� */
		}
	}

	_mon -= 1;

	for (t = 0; t < _mon; t++)         /* ��ǰ���·ݵ���������� */
	{
		seccount += (uint32_t)mon_table[t] * 86400;	/* �·���������� */

		if (IS_RTC_LeapYear(_year) && t == 1)
		{
			seccount += 86400;	/* ����2�·�����һ��������� */
		}			
	}

	seccount += (uint32_t)(_day - 1) * 86400;	/* ��ǰ�����ڵ���������� */

	seccount += (uint32_t)_hour * 3600;		/* Сʱ������ */

	seccount += (uint32_t)_min * 60;	/* ���������� */

	seccount += _sec;	/* �������Ӽ���ȥ */
       
	return seccount;      
}

/*
*********************************************************************************************************
*	�� �� ��: RTC_ReadClock
*	����˵��: �õ���ǰʱ�ӡ��������� g_tRTC��
*	��    �Σ���
*	�� �� ֵ: 1��ʾ�ɹ� 0��ʾ����
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
*	�� �� ��: bsp_CalcWeek
*	����˵��: �������ڼ������ڼ�
*	��    ��: _year _mon _day  ������  (����2�ֽ��������º������ֽ�������
*	�� �� ֵ: �ܼ� ��1-7�� 7��ʾ����
*********************************************************************************************************
*/
uint8_t bsp_RTC_CalcWeek(uint16_t _year, uint8_t _mon, uint8_t _day)
{
	/*
	���գ�Zeller����ʽ
		��ʷ�ϵ�ĳһ�������ڼ���δ����ĳһ�������ڼ�������������⣬�кܶ���㹫ʽ������ͨ�ü��㹫ʽ��
	һЩ�ֶμ��㹫ʽ�����������������ǲ��գ�Zeller����ʽ��
	    ��w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1

		��ʽ�еķ��ź������£�
	     w�����ڣ�
	     c����ĸ�2λ��������-1
	     y���꣨��λ������
	     m���£�m���ڵ���3��С�ڵ���14�����ڲ��չ�ʽ�У�ĳ���1��2��Ҫ������һ���13��14�������㣬
	  	    ����2003��1��1��Ҫ����2002���13��1�������㣩��
	     d���գ�
	     [ ]����ȡ������ֻҪ�������֡�

	    �������W����7�������Ǽ��������ڼ������������0����Ϊ�����ա�
        �������Ǹ�������������������Ҫ���⴦��
            �������ܰ�ϰ�ߵ������ĸ�����������ֻ�ܰ������е������Ķ������ࡣΪ�˷���
        ���㣬���ǿ��Ը�������һ��7����������ʹ����Ϊһ��������Ȼ����������

		��2049��10��1�գ�100������죩Ϊ�����ò��գ�Zeller����ʽ���м��㣬�������£�
		���գ�Zeller����ʽ��w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1
		=49+[49/4]+[20/4]-2��20+[26�� (10+1)/10]+1-1
		=49+[12.25]+5-40+[28.6]
		=49+12+5-40+28
		=54 (����7��5)
		��2049��10��1�գ�100������죩������5��
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
	else	/* ĳ���1��2��Ҫ������һ���13��14�������� */
	{
		m = _mon + 12;
		y = (_year - 1) % 100;
		c = (_year - 1) / 100;
		d = _day;
	}

	w = y + y / 4 +  c / 4 - 2 * c + ((uint16_t)26*(m+1))/10 + d - 1;
	if (w == 0)
	{
		w = 7;	/* ��ʾ���� */
	}
	else if (w < 0)	/* ���w�Ǹ����������������ʽ��ͬ */
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
