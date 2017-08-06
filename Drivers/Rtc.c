/*
 * FileName:        Rtc.c
 * Author:          YuanYin  Version: QXW-Summer-V1.x  Date: 2010-4-29
 * Description:     Rtc������
 */

/*****************************************************
 *  ����Ƽ�		Q-Share We	����-��Դ-����
 *
 *  ���������ǵĲ�Ʒ���κν������������ϵ����!
 *
 *  �Ա�רӪ��Q-ShareWe.TaoBao.Com
 *  ������̳��Www.Q-ShareWe.Com
 ****************************************************/

#include "Drivers.h"
#include "Rtc.h"
#include "Debug.h"

#define RTC_START_YEAR 1912  //�ɴ�1904�����κ�һ�����꿪ʼ,����ı��ֵ�����ڳ�ֵҪ�ı�
#define DAY_SECONDS 86400	//һ���������

//�����������������
const int Leap_Month_Seconds[13]={
	0,
	DAY_SECONDS*31,
	DAY_SECONDS*(31+29),
	DAY_SECONDS*(31+29+31),
	DAY_SECONDS*(31+29+31+30),
	DAY_SECONDS*(31+29+31+30+31),
	DAY_SECONDS*(31+29+31+30+31+30),
	DAY_SECONDS*(31+29+31+30+31+30+31),
	DAY_SECONDS*(31+29+31+30+31+30+31+31),
	DAY_SECONDS*(31+29+31+30+31+30+31+31+30),
	DAY_SECONDS*(31+29+31+30+31+30+31+31+30+31),
	DAY_SECONDS*(31+29+31+30+31+30+31+31+30+31+30),
	DAY_SECONDS*(31+29+31+30+31+30+31+31+30+31+30+31),
};

//�������������������
const int Month_Seconds[13]={
	0,
	DAY_SECONDS*31,
	DAY_SECONDS*(31+28),
	DAY_SECONDS*(31+28+31),
	DAY_SECONDS*(31+28+31+30),
	DAY_SECONDS*(31+28+31+30+31),
	DAY_SECONDS*(31+28+31+30+31+30),
	DAY_SECONDS*(31+28+31+30+31+30+31),
	DAY_SECONDS*(31+28+31+30+31+30+31+31),
	DAY_SECONDS*(31+28+31+30+31+30+31+31+30),
	DAY_SECONDS*(31+28+31+30+31+30+31+31+30+31),
	DAY_SECONDS*(31+28+31+30+31+30+31+31+30+31+30),
	DAY_SECONDS*(31+28+31+30+31+30+31+31+30+31+30+31),
};

#define RTCClockSource_LSE // ʹ���ⲿʱ�ӣ� 32.768KHz

extern unsigned int RTC_Counter;


/*******************************************************************************
* Function Name  : RTC_Configuration
* Description    : Configures the rtc.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_Configuration(void)
{
    u32 rtcintcnt=0x200000;

    /* Enable the PWR clock */ /* PWRʱ�ӣ���Դ���ƣ���BKPʱ�ӣ�RTC�󱸼Ĵ�����ʹ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_AHB1Periph_BKPSRAM, ENABLE);

    /* Allow access to BKP Domain */ /*ʹ��RTC�ͺ󱸼Ĵ������� */
    PWR_BackupAccessCmd(ENABLE);

    RCC_LSEConfig(RCC_LSE_ON);              //���ⲿ���پ���

    while((RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) && (--rtcintcnt));//�ȴ�LSE׼����
    if(rtcintcnt!=0)//�ڲ�����
    {
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); //ѡ��LSEλRTCʱ��
        RCC_RTCCLKCmd(ENABLE);                  //ʹ��RTCʱ��
        RTC_WaitForSynchro();                   //�ȴ�RTC�Ĵ�����APBʱ��ͬ��
    }
    else//���ⲿ����
    {
    	Debug("!!!RTC LSE NO WORK!!!\n\r");
        rtcintcnt=0x200000;
        RCC_HSEConfig(RCC_HSE_ON);/* Enable HSE */
        while ( (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) && (--rtcintcnt) );/* Wait till HSE is ready */
        if ( rtcintcnt == 0 )
        {
            return;
        }

	    RCC_RTCCLKCmd(ENABLE);/* Enable RTC Clock */
	    RTC_WaitForSynchro();/* Wait for RTC registers synchronization */
    }
}

/*******************************************************************************
* Function Name  : RTC_Config
* Description    : �ϵ�ʱ���ñ��������Զ�����Ƿ���ҪRTC��ʼ����
*                       ����Ҫ���³�ʼ��RTC�������RTC_Configuration()�����Ӧ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_SetUp(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RTC_TIME NowTime;

	/* ʹ��rtc�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RTC_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RTCAlarm_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//��鱸�üĴ�����ֵ�Ƿ�ı��ˣ�����ı��ˣ�˵��battery�����ˣ���Ҫ��������rtc
	if(RTC_ReadBackupRegister(RTC_BKP_DR1) != 0xA5A5)
	{
		/* Backup data register value is not correct or not yet programmed (when
		 	the first time the program is executed) */
		Debug("RTC need configure!\n\r");

		/* RTC Configuration */
		RTC_Configuration();

		NowTime.year=2017;
		NowTime.mon=1;
		NowTime.day=1;
		NowTime.hour=0;
		NowTime.min=0;
		NowTime.sec=0;
		if(RTC_Adjust(&NowTime,RtcOp_SetTime)==TRUE)
		{
			Debug("RTC set sucess!\n\r");
		}
		else
		{
			Debug("RTC set error!\n\r");
		}

		//���üĴ���д��һ��ֵ�����´���������
		RTC_WriteBackupRegister(RTC_BKP_DR1, 0xA5A5);
	}
	else	//������üĴ���ֵ��һ�£�˵�������ˡ�
	{
		if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)//����Ƿ��ϵ縴λ
	    {
	      Debug("System power on reset!\n\r");
	    }
	    else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)//����Ƿ��ֶ���λ
	    {
	      Debug("System reset!\n\r");
	    }
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_AHB1Periph_BKPSRAM, ENABLE);
	    PWR_BackupAccessCmd(ENABLE);

		RTC_WaitForSynchro();//�ȴ�RTC�Ĵ�����APBʱ��ͬ��
	}
	RCC_ClearFlag();//�����λ��־

	RTC_Counter=(RTC_GetWakeUpCounter()+10)/60+1;//��RTC counter�����һ���ӵ�ֵ
	RTC_Counter*=60;
	Debug("RTC configured finished\n\r");
}

//��ȡ��ǰʱ��ĺ���
//ֻ��Ҫ����һ���ṹ��ʵ�壬����ַ�����pTime���Ϳ��Եõ���ǰ������ʱ��
//�㷨�ȽϷ�����ѧϰ����Ҫ���Ŀ�
//��arm�����������Ҫ�Ƚϳ���ʱ�䣬���Ծ����������ó������෨
void RTC_GetRealTime(RTC_TIME *pTime)
{
  u32 NowTime;
  u32 tmp,i;

  NowTime = RTC_GetWakeUpCounter();//��ȡ��ǰ��ʱ��ֵ

  //���������꣬���ǵ�����Ĵ��ڣ���4��һ������
  tmp=NowTime%(DAY_SECONDS*366+DAY_SECONDS*365*3);
  if(tmp<DAY_SECONDS*366) pTime->year=RTC_START_YEAR+NowTime/(DAY_SECONDS*366+DAY_SECONDS*365*3)*4+0;
  else if(tmp<DAY_SECONDS*366+DAY_SECONDS*365*1) pTime->year=RTC_START_YEAR+NowTime/(DAY_SECONDS*366+DAY_SECONDS*365*3)*4+1;
  else if(tmp<DAY_SECONDS*366+DAY_SECONDS*365*2) pTime->year=RTC_START_YEAR+NowTime/(DAY_SECONDS*366+DAY_SECONDS*365*3)*4+2;
  else pTime->year=RTC_START_YEAR+NowTime/(DAY_SECONDS*366+DAY_SECONDS*365*3)*4+3;

  if(tmp<DAY_SECONDS*366) //����
  {
		for(i=1;i<13;i++)
	    {
			if(tmp<Leap_Month_Seconds[i])
			{
				pTime->mon=i;
				tmp-=Leap_Month_Seconds[i-1];//�������ѯ���渴�ӵļ���
				break;
		  	}
	   }
  }
  else if(tmp<DAY_SECONDS*366+DAY_SECONDS*365*1)
  {
		tmp-=DAY_SECONDS*366;

	  	for(i=1;i<13;i++)
		{
			if(tmp<Month_Seconds[i])
			{
				pTime->mon=i;
				tmp-=Month_Seconds[i-1];
				break;
			}
		}
  }
  else if(tmp<DAY_SECONDS*366+DAY_SECONDS*365*2)
  {
  		tmp-=DAY_SECONDS*366+DAY_SECONDS*365*1;

	  	for(i=1;i<13;i++)
		{
			if(tmp<Month_Seconds[i])
			{
				pTime->mon=i;
				tmp-=Month_Seconds[i-1];
				break;
			}
		}
  }
  else
  {
		tmp-=DAY_SECONDS*366+DAY_SECONDS*365*2;

	  	for(i=1;i<13;i++)
		{
			if(tmp<Month_Seconds[i])
			{
				pTime->mon=i;
				tmp-=Month_Seconds[i-1];
				break;
			}
		}
  }

  pTime->week=NowTime/DAY_SECONDS%7;//��Ϊ1912.1.1������һ�����Բ���ƫ��ֵ
  //ע���ȡ��ֵ��ΧΪ0-6��������Ϊ�˷���������ģ��Ӷ������������ʾ

  pTime->day=tmp/DAY_SECONDS+1;
  tmp=tmp%DAY_SECONDS;

  pTime->hour = tmp/3600;
  tmp=tmp%3600;

  pTime->min = tmp/60;

  pTime->sec = tmp%60;
}

//ʱ���������
//���������ýṹ���ַ������Ҫ�趨����ֵ��ע��Ҫ����ṹ��ʵ�壡
bool RTC_Adjust(RTC_TIME *pTime,RTC_OPERATE Op)
{
	u32 tmp1,tmp2;
	u8 Leap_Flag=0;

	tmp1=pTime->year-RTC_START_YEAR;
	if((tmp1>135)||(pTime->year<RTC_START_YEAR)) return FALSE; //��ݼ��

	if(tmp1)	tmp2=(tmp1-1)/4+1;
	else tmp2=0;
	Leap_Flag=(tmp1%4)?0:1;
	tmp1=(tmp1*365+tmp2)*DAY_SECONDS;//�껻��ɵ�����

	if((pTime->mon<1)||(pTime->mon>12)) return FALSE;  //�·ݼ��

	if(Leap_Flag)
	{
		if(pTime->day>((Leap_Month_Seconds[pTime->mon]-Leap_Month_Seconds[pTime->mon-1])/DAY_SECONDS)) return FALSE; //�ռ��
		tmp2=Leap_Month_Seconds[pTime->mon-1];
	}
	else
	{
		if(pTime->day>((Month_Seconds[pTime->mon]-Month_Seconds[pTime->mon-1])/DAY_SECONDS)) return FALSE;//�ռ��
		tmp2=Month_Seconds[pTime->mon-1];
	}

	if(pTime->hour>23) return FALSE; //Сʱ���
	if(pTime->min>59) return FALSE;	 //���Ӽ��
	if(pTime->sec>59) return FALSE;	 //����

	tmp1+=(tmp2+(pTime->day-1)*DAY_SECONDS);

	tmp1+=(pTime->hour*3600 + pTime->min*60 + pTime->sec);

	switch(Op)
	{
		case RtcOp_SetTime:
			RTC_SetWakeUpCounter(tmp1);

			RTC_Counter=(RTC_GetWakeUpCounter()+10)/60+1;//��RTC counter�����һ���ӵ�ֵ
			RTC_Counter*=60;
			break;
		case RtcOp_SetAlarm:
			//RTC_SetAlarm(tmp1);
			break;
	}

	return TRUE;
}
