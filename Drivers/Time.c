#include "Drivers.h"

void Tim3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ���ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1000; // ռ�ձȲ���
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ����ǵ�Ҫ��
    TIM_Cmd(TIM3, ENABLE);
}

//���뷶Χ0-100
void Tim3_PWM(u8 Value)
{
 	TIM_OCInitTypeDef TIM_OCStructure;	 	//����������Ƚϳ�ʼ���ṹ��

	TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;	   //PWM1ģʽ
	TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStructure.TIM_Pulse = Value*10;		//����ֵ ����CCR�Ĵ���
	TIM_OCStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�Ƚ�ƥ������ߵ�ƽ
	TIM_OC1Init(TIM3, &TIM_OCStructure);  //ʹ���ϲ�����Ч
}

void Tim2_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//���ö�ʱ��TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIME2_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void Tim4_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//���ö�ʱ��TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIME4_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void Tim5_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//���ö�ʱ��TIM5�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIME5_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
}

//uS_Base ��ʾ��λ��Ϊ1ʱ����λ��us��Ϊ100ʱ����λ��100us����Сֵ1�����ֵ900
//���ն�ʱֵ= Val x uS_Base x 1us
//�¶�ʱ�趨�Ḳ�Ǿ��趨
//AutoReload�����趨��һ�ζ�ʱ����ѭ����ʱ
//val��uS_Base��������һ��Ϊ0����ֹͣ��ǰ��ʱ��
void Tim2_Set(u16 Val,u16 uS_Base,bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//��ȡϵͳƵ��

    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);

    if((Val==0)||(uS_Base==0)) return;

    if(uS_Base>900) uS_Base=900;

    //��ʱƵ��Ϊ�� 72M/(Ԥ��Ƶ+1)/Ԥװ��
    TIM_TimeBaseStructure.TIM_Period        = (Val-1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency/1000000)*uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);

	TIM_ClearFlag(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);/* ʹ��TIM2�� ������� �ж� */

	TIM_Cmd(TIM2, ENABLE);/* ʹ��TIM2 */
}

void Tim4_Set(u16 Val, u16 uS_Base, bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//��ȡϵͳƵ��

    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM4, DISABLE);

    if((Val == 0) || (uS_Base == 0)) return;

    if(uS_Base > 900) uS_Base = 900;

    //��ʱƵ��Ϊ�� 72M/(Ԥ��Ƶ+1)/Ԥװ��
    TIM_TimeBaseStructure.TIM_Period        = (Val - 1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency / 1000000) * uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);

	TIM_ClearFlag(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);/* ʹ��TIM4�� ������� �ж� */

	TIM_Cmd(TIM4, ENABLE);/* ʹ��TIM4 */
}

void Tim5_Set(u16 Val, u16 uS_Base, bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//��ȡϵͳƵ��

    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM5, DISABLE);

    if((Val == 0) || (uS_Base == 0)) return;

    if(uS_Base > 900) uS_Base = 900;

    //��ʱƵ��Ϊ�� 72M/(Ԥ��Ƶ+1)/Ԥװ��
    TIM_TimeBaseStructure.TIM_Period        = (Val - 1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency / 1000000) * uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Single);

	TIM_ClearFlag(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);/* ʹ��TIM5�� ������� �ж� */

	TIM_Cmd(TIM5, ENABLE);/* ʹ��TIM5 */
}


uint16_t bsp_GetPinSource(uint16_t GPIOx)
{
        if(GPIOx == GPIO_Pin_0) return GPIO_PinSource0;
        else if(GPIOx == GPIO_Pin_1) return GPIO_PinSource1;
        else if(GPIOx == GPIO_Pin_2) return GPIO_PinSource2;
        else if(GPIOx == GPIO_Pin_3) return GPIO_PinSource3;
        else if(GPIOx == GPIO_Pin_4) return GPIO_PinSource4;
        else if(GPIOx == GPIO_Pin_5) return GPIO_PinSource5;
        else if(GPIOx == GPIO_Pin_6) return GPIO_PinSource6;
        else if(GPIOx == GPIO_Pin_7) return GPIO_PinSource7;
        else if(GPIOx == GPIO_Pin_8) return GPIO_PinSource8;
        else if(GPIOx == GPIO_Pin_9) return GPIO_PinSource9;
        else if(GPIOx == GPIO_Pin_10) return GPIO_PinSource10;
        else if(GPIOx == GPIO_Pin_11) return GPIO_PinSource11;
        else if(GPIOx == GPIO_Pin_12) return GPIO_PinSource12;
        else if(GPIOx == GPIO_Pin_13) return GPIO_PinSource13;
        else if(GPIOx == GPIO_Pin_14) return GPIO_PinSource14;
        else return GPIO_PinSource15;
}

uint8_t bsp_GetGpioAfTIMx(TIM_TypeDef* TIMx)
{
       if(TIMx == TIM1) return GPIO_AF_TIM1;
       else if(TIMx == TIM2) return GPIO_AF_TIM2;
       else if(TIMx == TIM3) return GPIO_AF_TIM3;
       else if(TIMx == TIM4) return GPIO_AF_TIM4;
       else if(TIMx == TIM5) return GPIO_AF_TIM5;
       else if(TIMx == TIM8) return GPIO_AF_TIM8;
       else if(TIMx == TIM9) return GPIO_AF_TIM9;
       else if(TIMx == TIM10) return GPIO_AF_TIM10;
       else if(TIMx == TIM11) return GPIO_AF_TIM11;
       else if(TIMx == TIM12) return GPIO_AF_TIM12;
       else if(TIMx == TIM13) return GPIO_AF_TIM13;
       else return GPIO_AF_TIM14;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRCCofGPIO
*	����˵��: ����GPIO �õ�RCC�Ĵ���
*	��    �Σ���
*	�� �� ֵ: GPIO����ʱ����
*********************************************************************************************************
*/
uint32_t bsp_GetRCCofGPIO(GPIO_TypeDef* GPIOx)
{
	uint32_t rcc = 0;

	if (GPIOx == GPIOA)
	{
		rcc = RCC_AHB1Periph_GPIOA;
	}
	else if (GPIOx == GPIOB)
	{
		rcc = RCC_AHB1Periph_GPIOB;
	}
	else if (GPIOx == GPIOC)
	{
		rcc = RCC_AHB1Periph_GPIOC;
	}
	else if (GPIOx == GPIOD)
	{
		rcc = RCC_AHB1Periph_GPIOD;
	}
	else if (GPIOx == GPIOE)
	{
		rcc = RCC_AHB1Periph_GPIOE;
	}
	else if (GPIOx == GPIOF)
	{
		rcc = RCC_AHB1Periph_GPIOF;
	}
	else if (GPIOx == GPIOG)
	{
		rcc = RCC_AHB1Periph_GPIOG;
	}

	return rcc;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRCCofTIM
*	����˵��: ����TIM �õ�RCC�Ĵ���
*	��    �Σ���
*	�� �� ֵ: TIM����ʱ����
*********************************************************************************************************
*/
uint32_t bsp_GetRCCofTIM(TIM_TypeDef* TIMx)
{
	uint32_t rcc = 0;

	/*
		APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14
		APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11
	*/
	if (TIMx == TIM1)
	{
		rcc = RCC_APB2Periph_TIM1;
	}
	else if (TIMx == TIM8)
	{
		rcc = RCC_APB2Periph_TIM8;
	}
	else if (TIMx == TIM9)
	{
		rcc = RCC_APB2Periph_TIM9;
	}
	else if (TIMx == TIM10)
	{
		rcc = RCC_APB2Periph_TIM10;
	}
	else if (TIMx == TIM11)
	{
		rcc = RCC_APB2Periph_TIM11;
	}
	/* ������ APB1ʱ�� */
	else if (TIMx == TIM2)
	{
		rcc = RCC_APB1Periph_TIM2;
	}
	else if (TIMx == TIM3)
	{
		rcc = RCC_APB1Periph_TIM3;
	}
	else if (TIMx == TIM4)
	{
		rcc = RCC_APB1Periph_TIM4;
	}
	else if (TIMx == TIM5)
	{
		rcc = RCC_APB1Periph_TIM5;
	}
	else if (TIMx == TIM6)
	{
		rcc = RCC_APB1Periph_TIM6;
	}
	else if (TIMx == TIM7)
	{
		rcc = RCC_APB1Periph_TIM7;
	}
	else if (TIMx == TIM12)
	{
		rcc = RCC_APB1Periph_TIM12;
	}
	else if (TIMx == TIM13)
	{
		rcc = RCC_APB1Periph_TIM13;
	}
	else if (TIMx == TIM14)
	{
		rcc = RCC_APB1Periph_TIM14;
	}

	return rcc;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ConfigTimGpio
*	����˵��: ����GPIO��TIMʱ�ӣ� GPIO���ӵ�TIM���ͨ��
*	��    ��: GPIOx
*			 GPIO_PinX
*			 TIMx
*			 _ucChannel
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX, TIM_TypeDef* TIMx, uint8_t _ucChannel)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* ʹ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

  	/* ����GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		/* ���ù��� */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

    /* Connect TIM3 pins to AF2 */
    if(TIMx != TIM6 || TIMx != TIM7)
        GPIO_PinAFConfig(GPIOx, bsp_GetPinSource(GPIO_PinX), bsp_GetGpioAfTIMx(TIMx));
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_ConfigGpioOut
*	����˵��: ����GPIOΪ�����������Ҫ����PWM�����ռ�ձ�Ϊ0��100�������
*	��    ��: GPIOx
*			  GPIO_PinX
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* ʹ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

	/* ����GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinX;		/* ������β� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* ��� */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SetTIMOutPWM_N
*	����˵��: ����TIM8_CH1N �ȷ������������PWM�źŵ�Ƶ�ʺ�ռ�ձ�.  ��Ƶ��Ϊ0������ռ��Ϊ0ʱ���رն�ʱ����GPIO���0��
*			  ��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1.
*	��    ��: _ulFreq : PWM�ź�Ƶ�ʣ���λHz  (ʵ�ʲ��ԣ�������Ƶ��Ϊ 168M / 4 = 42M��. 0 ��ʾ��ֹ���
*			  _ulDutyCycle : PWM�ź�ռ�ձȣ���λ�����֮һ����5000����ʾ50.00%��ռ�ձ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetTIMOutPWM_N(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
        TIM_TypeDef* TIMx, uint8_t _ucChannel,	 uint32_t _ulFreq, uint32_t _ulDutyCycle)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t usPeriod;
	uint16_t usPrescaler = 0;
	uint32_t uiTIMxCLK;

    /* �����߽�ֱֵ����GPIO��ģʽʹ����ƽֵ */
	if (_ulDutyCycle == 0 || _ulDutyCycle == 100)
	{
		TIM_Cmd(TIMx, DISABLE);		/* �ر�PWM��� */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* ����GPIOΪ������� */
		GPIO_WriteBit(GPIOx, GPIO_Pin, (_ulDutyCycle == 0) ? Bit_SET : Bit_RESET);	/* PWM = 0 : 1*/
		return;
	}

    /* GPIO ���� */
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, TIMx, _ucChannel);	/* ʹ��GPIO��TIMʱ�ӣ�������TIMͨ����GPIO */

    /*-----------------------------------------------------------------------
		system_stm32f4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 2      (APB2Periph)
		PCLK1 = HCLK / 4      (APB1Periph)

		��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
		��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = PCLK2 x 2 = SystemCoreClock;

		APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
		APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11

	----------------------------------------------------------------------- */
        /* ʹ��TIMʱ�� */
	if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		RCC_APB2PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	else
		RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);

    if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		uiTIMxCLK = SystemCoreClock / 1; /* APB2 ��ʱ�� = AHB����2��Ƶ */
	else
		uiTIMxCLK = SystemCoreClock / 2;	 /* APB1 ��ʱ�� = AHB����4��Ƶ */

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* ��Ƶ�� = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* ��Ƶ�� = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else	/* ����4K��Ƶ�ʣ������Ƶ */
	{
		usPrescaler = 0;					/* ��Ƶ�� = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* �Զ���װ��ֵ */
	}

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = usPeriod;//����ʱ����0������usPeriod����Ϊһ����ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler; //����Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//����ʱ�ӷ�Ƶϵ��������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ

	if(TIMx == TIM1 || TIMx == TIM8)
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCStructInit(&TIM_OCInitStructure);		/* ��ʼ���ṹ���Ա */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		/* �� bsp_SetTIMOutPWM_N() ��ͬ */
	TIM_OCInitStructure.TIM_Pulse = (_ulDutyCycle * usPeriod)  / 100; //��������ֵ�������������������ֵʱ����ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

        if(TIMx == TIM1 || TIMx == TIM8)
         {
                TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;		/* only for TIM1 and TIM8. */
                TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			/* only for TIM1 and TIM8. */
                TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		/* only for TIM1 and TIM8. */
                TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		/* only for TIM1 and TIM8. */
         }
	if (_ucChannel == 1)
	{
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == 2)
	{
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == 3)
	{
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if (_ucChannel == 4)
	{
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(TIMx, ENABLE);

        /* TIMx enable counter */
	TIM_Cmd(TIMx, ENABLE);

	/* ������仰����TIM1��TIM8�Ǳ���ģ�����TIM2-TIM6�򲻱�Ҫ */
	if ((TIMx == TIM1) || (TIMx == TIM8))
	{
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	}
}


