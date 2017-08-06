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

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 输出模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1000; // 占空比参数
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(TIM3, ENABLE); // 这个记得要开
    TIM_Cmd(TIM3, ENABLE);
}

//输入范围0-100
void Tim3_PWM(u8 Value)
{
 	TIM_OCInitTypeDef TIM_OCStructure;	 	//计数器输出比较初始化结构体

	TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;	   //PWM1模式
	TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStructure.TIM_Pulse = Value*10;		//脉冲值 赋给CCR寄存器
	TIM_OCStructure.TIM_OCPolarity = TIM_OCPolarity_High;//比较匹配输出高电平
	TIM_OC1Init(TIM3, &TIM_OCStructure);  //使以上参数有效
}

void Tim2_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//配置定时器TIM2中断
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

	//配置定时器TIM4中断
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

	//配置定时器TIM5中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIME5_IRQn_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
}

//uS_Base 表示单位，为1时，单位是us；为100时，单位是100us；最小值1，最大值900
//最终定时值= Val x uS_Base x 1us
//新定时设定会覆盖旧设定
//AutoReload用来设定是一次定时还是循环定时
//val和uS_Base其中任意一个为0，则停止当前定时。
void Tim2_Set(u16 Val,u16 uS_Base,bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//获取系统频率

    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);

    if((Val==0)||(uS_Base==0)) return;

    if(uS_Base>900) uS_Base=900;

    //定时频率为： 72M/(预分频+1)/预装载
    TIM_TimeBaseStructure.TIM_Period        = (Val-1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency/1000000)*uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);

	TIM_ClearFlag(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);/* 使能TIM2的 向上溢出 中断 */

	TIM_Cmd(TIM2, ENABLE);/* 使能TIM2 */
}

void Tim4_Set(u16 Val, u16 uS_Base, bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//获取系统频率

    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM4, DISABLE);

    if((Val == 0) || (uS_Base == 0)) return;

    if(uS_Base > 900) uS_Base = 900;

    //定时频率为： 72M/(预分频+1)/预装载
    TIM_TimeBaseStructure.TIM_Period        = (Val - 1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency / 1000000) * uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);

	TIM_ClearFlag(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);/* 使能TIM4的 向上溢出 中断 */

	TIM_Cmd(TIM4, ENABLE);/* 使能TIM4 */
}

void Tim5_Set(u16 Val, u16 uS_Base, bool AutoReload)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);//获取系统频率

    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM5, DISABLE);

    if((Val == 0) || (uS_Base == 0)) return;

    if(uS_Base > 900) uS_Base = 900;

    //定时频率为： 72M/(预分频+1)/预装载
    TIM_TimeBaseStructure.TIM_Period        = (Val - 1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((RCC_Clocks.SYSCLK_Frequency / 1000000) * uS_Base - 1);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	if(AutoReload) TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Repetitive);
	else TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Single);

	TIM_ClearFlag(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);/* 使能TIM5的 向上溢出 中断 */

	TIM_Cmd(TIM5, ENABLE);/* 使能TIM5 */
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
*	函 数 名: bsp_GetRCCofGPIO
*	功能说明: 根据GPIO 得到RCC寄存器
*	形    参：无
*	返 回 值: GPIO外设时钟名
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
*	函 数 名: bsp_GetRCCofTIM
*	功能说明: 根据TIM 得到RCC寄存器
*	形    参：无
*	返 回 值: TIM外设时钟名
*********************************************************************************************************
*/
uint32_t bsp_GetRCCofTIM(TIM_TypeDef* TIMx)
{
	uint32_t rcc = 0;

	/*
		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14
		APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11
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
	/* 下面是 APB1时钟 */
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
*	函 数 名: bsp_ConfigTimGpio
*	功能说明: 配置GPIO和TIM时钟， GPIO连接到TIM输出通道
*	形    参: GPIOx
*			 GPIO_PinX
*			 TIMx
*			 _ucChannel
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX, TIM_TypeDef* TIMx, uint8_t _ucChannel)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* 使能GPIO时钟 */
	RCC_AHB1PeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

  	/* 配置GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		/* 复用功能 */
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
*	函 数 名: bsp_ConfigGpioOut
*	功能说明: 配置GPIO为推挽输出。主要用于PWM输出，占空比为0和100的情况。
*	形    参: GPIOx
*			  GPIO_PinX
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* 使能GPIO时钟 */
	RCC_AHB1PeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

	/* 配置GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinX;		/* 带入的形参 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* 输出 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SetTIMOutPWM_N
*	功能说明: 设置TIM8_CH1N 等反相引脚输出的PWM信号的频率和占空比.  当频率为0，并且占空为0时，关闭定时器，GPIO输出0；
*			  当频率为0，占空比为100%时，GPIO输出1.
*	形    参: _ulFreq : PWM信号频率，单位Hz  (实际测试，最大输出频率为 168M / 4 = 42M）. 0 表示禁止输出
*			  _ulDutyCycle : PWM信号占空比，单位：万分之一。如5000，表示50.00%的占空比
*	返 回 值: 无
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

    /* 两个边界值直接以GPIO的模式使出电平值 */
	if (_ulDutyCycle == 0 || _ulDutyCycle == 100)
	{
		TIM_Cmd(TIMx, DISABLE);		/* 关闭PWM输出 */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* 配置GPIO为推挽输出 */
		GPIO_WriteBit(GPIOx, GPIO_Pin, (_ulDutyCycle == 0) ? Bit_SET : Bit_RESET);	/* PWM = 0 : 1*/
		return;
	}

    /* GPIO 配置 */
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, TIMx, _ucChannel);	/* 使能GPIO和TIM时钟，并连接TIM通道到GPIO */

    /*-----------------------------------------------------------------------
		system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 2      (APB2Periph)
		PCLK1 = HCLK / 4      (APB1Periph)

		因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
		因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
		APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11

	----------------------------------------------------------------------- */
        /* 使能TIM时钟 */
	if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		RCC_APB2PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	else
		RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);

    if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		uiTIMxCLK = SystemCoreClock / 1; /* APB2 定时器 = AHB进行2分频 */
	else
		uiTIMxCLK = SystemCoreClock / 2;	 /* APB1 定时器 = AHB进行4分频 */

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* 分频比 = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* 分频比 = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else	/* 大于4K的频率，无需分频 */
	{
		usPrescaler = 0;					/* 分频比 = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* 自动重装的值 */
	}

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = usPeriod;//当定时器从0计数到usPeriod，即为一个定时周期
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler; //设置预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分频系数：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式

	if(TIMx == TIM1 || TIMx == TIM8)
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCStructInit(&TIM_OCInitStructure);		/* 初始化结构体成员 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		/* 和 bsp_SetTIMOutPWM_N() 不同 */
	TIM_OCInitStructure.TIM_Pulse = (_ulDutyCycle * usPeriod)  / 100; //设置跳变值，当计数器计数到这个值时，电平发生跳变
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

	/* 下面这句话对于TIM1和TIM8是必须的，对于TIM2-TIM6则不必要 */
	if ((TIMx == TIM1) || (TIMx == TIM8))
	{
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	}
}


