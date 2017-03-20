/*
*********************************************************************************************************
*
*	ģ������ : ������������ģ��
*	�ļ����� : bsp_key.c
*	��    �� : V1.0
*	˵    �� : ɨ�������������������˲����ƣ����а���FIFO�����Լ�������¼���
*				(1) ��������
*				(2) ��������
*				(3) ������
*				(4) ����ʱ�Զ�����
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*		V1.1    2013-06-29 armfly  ����1����ָ�룬����bsp_Idle() ������ȡϵͳ������ϼ���������
*								   ���� K1 K2 ��ϼ� �� K2 K3 ��ϼ�������ϵͳ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/**********************************************************************************************
**	�ó��������ڰ�����STM32-X3��STM32-V5������
**
**	�����������Ӳ�������޸�GPIO����� IsKeyDown1 - IsKeyDown8 ����
**
**	����û��İ�������С��8��������Խ�����İ���ȫ������Ϊ�͵�1������һ��������Ӱ�������
**	#define KEY_COUNT    8	  ����� bsp_key.h �ļ��ж���
***********************************************************************************************/

/////////////////////////////////////////////////////////////////
static KEY_T s_tBtn[KEY_COUNT];
static KEY_FIFO_T s_tKey;		/* ����FIFO����,�ṹ�� */
static uint8_t gTest = 0;
/*
*********************************************************************************************************
*	�� �� ��: IsKeyDownX
*	����˵��: �жϰ����Ƿ���
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ���£�0��ʾδ����
*********************************************************************************************************
*/
static uint8_t IsKeyDown0(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & 0x1) return 1;else return 0;}
static uint8_t IsKeyDown1(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 1)) return 1;else return 0;}
static uint8_t IsKeyDown2(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 2)) return 1;else return 0;}
static uint8_t IsKeyDown3(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 3)) return 1;else return 0;}
static uint8_t IsKeyDown4(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 4)) return 1;else return 0;}
static uint8_t IsKeyDown5(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 5)) return 1;else return 0;}
static uint8_t IsKeyDown6(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 6)) return 1;else return 0;}
static uint8_t IsKeyDown7(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 7)) return 1;else return 0;}
static uint8_t IsKeyDown8(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 8)) return 1;else return 0;}
static uint8_t IsKeyDown9(void) {if (RTC_ReadBackupRegister(RTC_BKP_DR0) & (0x1 << 9)) return 1;else return 0;}

static uint8_t IsKeyDown10(void) {if (IsKeyDown8() && IsKeyDown9()) return 1;else return 0;}
static uint8_t IsKeyDown11(void) {if (IsKeyDown6() && IsKeyDown7()) return 1;else return 0;}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyHard
*	����˵��: ���ð�����Ӧ��GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��1������GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_ALL_KEY, ENABLE);

	/* ��2�����������еİ���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* �������������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        /* ��Ϊ����� */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SDA;
	GPIO_Init(GPIO_PORT_SDA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_INT;
	GPIO_Init(GPIO_PORT_INT, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        /* ��Ϊ����� */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SCK;
	GPIO_Init(GPIO_PORT_SCK, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);           /* SCKĬ�����Ϊ�ߵ�ƽ*/
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyVar
*	����˵��: ��ʼ����������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
	    s_tBtn[i].KeyDownTick = xTaskGetTickCount();
        s_tBtn[i].KeyUpTick = s_tBtn[i].KeyDownTick;

	    s_tBtn[i].ShortTime = KEY_SHORT_TIME;       /* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
        
		//s_tBtn[i].KeyCodeDown = 3 * i + 1;				/* �������µļ�ֵ���� */
		//s_tBtn[i].KeyCodeUp   = 3 * i + 2;				/* ��������ļ�ֵ���� */
		//s_tBtn[i].KeyCodeLong = 3 * i + 3;				/* �������������µļ�ֵ���� */
		
		s_tBtn[i].RepeatTime = 500;						/* ����������ʱ������0��ʾ��֧������ */
		s_tBtn[i].RepeatCount = 0;						/* ���������� */
        
	}

	/* �����Ҫ��������ĳ�������Ĳ����������ڴ˵������¸�ֵ */
	/* ���磬����ϣ������1���³���1����Զ��ط���ͬ��ֵ */
	//s_tBtn[KID_JOY_U].LongTime = 100;
	//s_tBtn[KID_JOY_U].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */


	/* �жϰ������µĺ��� */
	s_tBtn[0].IsKeyDownFunc = IsKeyDown0;
	s_tBtn[1].IsKeyDownFunc = IsKeyDown1;
	s_tBtn[2].IsKeyDownFunc = IsKeyDown2;
	s_tBtn[3].IsKeyDownFunc = IsKeyDown3;
	s_tBtn[4].IsKeyDownFunc = IsKeyDown4;
	s_tBtn[5].IsKeyDownFunc = IsKeyDown5;
	s_tBtn[6].IsKeyDownFunc = IsKeyDown6;
	s_tBtn[7].IsKeyDownFunc = IsKeyDown7;
    s_tBtn[8].IsKeyDownFunc = IsKeyDown8;
	s_tBtn[9].IsKeyDownFunc = IsKeyDown9;

	/* ��ϼ� */
	s_tBtn[10].IsKeyDownFunc = IsKeyDown10;
    s_tBtn[11].IsKeyDownFunc = IsKeyDown11;
}

/*
********************************************************************************
* �� �� ��: bsp_InitKeyEXTI
* ����˵��: �����еİ������ó��ⲿ�жϴ�����ʽ������ҡ���Ҽ�����Ϊҡ���Ҽ��Ͱ�����
* ʹ�õ��ж��߶���EXTI_Line11��ͬʱʹ�õĻ���ֻ��һ����Ч
* �� ��: ��
* �� �� ֵ: ��
********************************************************************************
*/
static void bsp_InitKeyEXTI(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* ʹ��SYSCFGʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* ���� EXTI Line8 �� PI8 ���� */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    
    /* ���� EXTI LineXXX */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /* ����NVIC���ȼ�����ΪGroup2��0-3��ռʽ���ȼ���0-3����Ӧʽ���ȼ� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    /* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKey
*	����˵��: ��ʼ������. �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* ��ʼ���������� */
	bsp_InitKeyHard();		/* ��ʼ������Ӳ�� */
	bsp_InitKeyEXTI();      /* ����K1Ϊ�ⲿ�жϴ��� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PutKey
*	����˵��: ��1����ֵѹ�밴��FIFO��������������ģ��һ��������
*	��    ��:  _KeyCode : ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKey
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ��
*	��    ��:  ��
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_GetKey(void)
{
	uint8_t ret = KEY_NONE;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];
        //s_tKey.Buf[s_tKey.Read] = KEY_NONE;

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKey2
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ�������Ķ�ָ�롣
*	��    ��:  ��
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_GetKey2(void)
{
	uint8_t ret;

	if (s_tKey.Read2 == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read2];

		if (++s_tKey.Read2 >= KEY_FIFO_SIZE)
		{
			s_tKey.Read2 = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKeyState
*	����˵��: ��ȡ������״̬
*	��    ��:  _ucKeyID : ����ID����0��ʼ
*	�� �� ֵ: 1 ��ʾ���£� 0 ��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetKeyParam
*	����˵��: ���ð�������
*	��    �Σ�_ucKeyID : ����ID����0��ʼ
*			_LongTime : �����¼�ʱ��
*			 _RepeatSpeed : �����ٶ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatTime)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
	s_tBtn[_ucKeyID].RepeatTime = _RepeatTime;			/* �����������ٶȣ�0��ʾ��֧������ */
	s_tBtn[_ucKeyID].RepeatCount = 0;						/* ���������� */
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearKey
*	����˵��: ��հ���FIFO������
*	��    �Σ���
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_TouchKeyScan
*	����˵��: ɨ�����а���������������systick�ж������Եĵ���
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void bsp_TouchKeyScan(void)
{
	uint8_t i;
    KEY_T *pBtn;
    TickType_t Tick_Current = 0;
    if(gTest)
    {
        printf("Key code value: %x\r\n",RTC_ReadBackupRegister(RTC_BKP_DR0));
        gTest = 0;
    }
    Tick_Current = xTaskGetTickCount();
	for (i = 0; i < KEY_COUNT; i++)
	{
	    pBtn = &s_tBtn[i];
        if (pBtn->State) //�а�������
	    {
	        if(Tick_Current - pBtn->KeyDownTick > pBtn->LongTime) //��������
	        {
	            if (pBtn->RepeatTime > 0)
				{
					if((Tick_Current - pBtn->KeyDownTick - pBtn->LongTime)
                        / pBtn->RepeatTime == 0)
					{
						pBtn->RepeatCount++;
						/* ��������ÿ��10ms����1������ */
						bsp_PutKey((uint8_t)(3 * i + 1));
					}
				}
	        }
        }
        else //�����Ѿ��ɿ�
        {
            if(pBtn->KeyUpTick > pBtn->KeyDownTick)
            {
                pBtn->KeyDownTick = pBtn->KeyUpTick;
                if(pBtn->KeyUpTick - pBtn->KeyDownTick < 1000)
                    bsp_PutKey((uint8_t)(3 * i + 1));
                else if(pBtn->KeyUpTick - pBtn->KeyDownTick < 3000)
                    bsp_PutKey((uint8_t)(3 * i + 3));
                else
                {
                    pBtn->RepeatCount = 0;
                }
            }
            else
            {
                bsp_PutKey((uint8_t)(KEY_NONE));
            }
        }
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyCodeValueRead
*	����˵��: ��ȡ�����ļ�ֵ��д��FIFO������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_TouchKeyCodeValueProcess(void)
{
	uint8_t i = 0;
    KEY_T *pBtn; 
    TickType_t Tick_Current = 0;
	uint16_t _KeyCodeValue = 0x0;
	
	//if(GPIO_ReadInputDataBit(GPIO_PORT_SDA, GPIO_PIN_SDA))
	//	return ;
    gTest = 1;

    Tick_Current = xTaskGetTickCountFromISR();
    
	GPIO_SetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);
    GPIO_ResetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);
	GPIO_SetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);

	/* ��ȡ�����ļ�ֵ */
	for(i = 0; i < 10; i++)
	{
	    pBtn = &s_tBtn[i];

		GPIO_ResetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);
		bsp_DelayUS(50);
		GPIO_SetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);
		
		if( GPIO_ReadInputDataBit(GPIO_PORT_SDA, GPIO_PIN_SDA) )
		{
			_KeyCodeValue &= (~(0x1 << i));
            pBtn->State = 0;
            pBtn->KeyUpTick = Tick_Current;
		}
		else
		{
			_KeyCodeValue |= (0x1 << i);
            pBtn->State = 1;
            pBtn->KeyDownTick = Tick_Current;
		}        
        
		bsp_DelayUS(50);
	}

    /* ������ϰ��� */
    if(s_tBtn[8].State && s_tBtn[9].State)
    {
        s_tBtn[10].State = 1;
        s_tBtn[10].KeyDownTick = Tick_Current;
    }
    else
    {
        s_tBtn[10].State = 1;
        s_tBtn[10].KeyUpTick = Tick_Current;
    }
    
    if(s_tBtn[6].State && s_tBtn[7].State)
    {
        s_tBtn[11].State = 1;
        s_tBtn[11].KeyDownTick = Tick_Current;
    }
    else
    {
        s_tBtn[11].State = 1;
        s_tBtn[11].KeyUpTick = Tick_Current;
    }

	RTC_WriteBackupRegister(RTC_BKP_DR0, _KeyCodeValue);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetLedLight
*	����˵��: ���ð������������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetLedLight(uint8_t LightValue)
{
    bsp_SetTIMOutPWM(GPIOC, GPIO_Pin_9, TIM8, 4, 2000, LightValue);	// TIM8_CH4N
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
