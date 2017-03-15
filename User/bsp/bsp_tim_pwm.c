/*
*********************************************************************************************************
*
*	模块名称 : TIM基本定时中断和PWM驱动模块
*	文件名称 : bsp_tim_pwm.c
*	版    本 : V1.1
*	说    明 : 利用STM32F4内部TIM输出PWM信号， 并实现基本的定时中断
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-08-16 armfly  正式发布
*		V1.1	2014-06-15 armfly  完善 bsp_SetTIMOutPWM，当占空比=0和100%时，关闭定时器，GPIO配置为输出
*		V1.2	2015-05-08 armfly  解决TIM8不能输出PWM的问题。
*		V1.3	2015-07-30 armfly  增加反相引脚输出PWM函数 bsp_SetTIMOutPWM_N();
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/*
	可以输出到GPIO的TIM通道:

	PA0  TIM5_CH1
	PA0  TIM5_CH2  TIM2_CH2
	PA2  TIM5_CH3  TIM2_CH3
	PA3  TIM5_CH4  TIM2_CH4
	
	PA6  TIM3_CH1
	PA7  TIM3_CH2
	
	PB0  TIM3_CH3
	PB1  TIM3_CH4

	PE9   TIM1_CH1
	PE11  TIM1_CH2
	PE13  TIM1_CH3

	PE14  TIM1_CH4
	
	PD12	TIM4_CH1
	PD13	TIM4_CH2
	PD14	TIM4_CH3
	PD15	TIM4_CH4	
	
	PC6		TIM8_CH1
	PC7		TIM8_CH2
	PC8		TIM8_CH3
	PC9		TIM8_CH4	
	
	PA8		TIM1_CH1
	PA9		TIM1_CH2
	PA10	TIM1_CH3
	PA11	TIM1_CH4	

	PB3		TIM2_CH2
	PB4		TIM3_CH1
	PB5		TIM3_CH2
	
	PB6		TIM4_CH1
	PB7		TIM4_CH2	
	PB8		TIM4_CH3
	PB9		TIM4_CH4		

	APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7        --- 36M
	APB2 定时器有 TIM1, TIM8                    ---- 72M
*/

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
*	函 数 名: bsp_SetTIMOutPWM
*	功能说明: 设置引脚输出的PWM信号的频率和占空比.  当频率为0，并且占空为0时，关闭定时器，GPIO输出0；
*			  当频率为0，占空比为100%时，GPIO输出1.
*	形    参: _ulFreq : PWM信号频率，单位Hz  (实际测试，最大输出频率为 168M / 4 = 42M）. 0 表示禁止输出
*			  _ulDutyCycle : PWM信号占空比，单位：万分之一。如5000，表示50.00%的占空比
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, 
        TIM_TypeDef* TIMx, uint8_t _ucChannel,   uint32_t _ulFreq, uint32_t _ulDutyCycle)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    uint16_t usPeriod;
    uint16_t usPrescaler = 0;
    uint32_t uiTIMxCLK;

    /* 两个边界值直接以GPIO的模式使出电平值 */
    if (_ulDutyCycle == 0 || _ulDutyCycle == 100)
    {       
        TIM_Cmd(TIMx, DISABLE);     /* 关闭PWM输出 */
        bsp_ConfigGpioOut(GPIOx, GPIO_Pin); /* 配置GPIO为推挽输出 */        
        GPIO_WriteBit(GPIOx, GPIO_Pin, (_ulDutyCycle == 0) ? Bit_RESET : Bit_SET);  /* PWM = 0 : 1*/        
        return;
    }

    /* GPIO 配置 */
    bsp_ConfigTimGpio(GPIOx, GPIO_Pin, TIMx, _ucChannel);   /* 使能GPIO和TIM时钟，并连接TIM通道到GPIO */
    
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
        uiTIMxCLK = SystemCoreClock / 2;     /* APB1 定时器 = AHB进行4分频 */

    if (_ulFreq < 100)
    {
        usPrescaler = 10000 - 1;                    /* 分频比 = 10000 */
        usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;     /* 自动重装的值 */
    }
    else if (_ulFreq < 3000)
    {
        usPrescaler = 100 - 1;                  /* 分频比 = 100 */
        usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;       /* 自动重装的值 */
    }
    else    /* 大于4K的频率，无需分频 */
    {
        usPrescaler = 0;                    /* 分频比 = 1 */
        usPeriod = uiTIMxCLK / _ulFreq - 1; /* 自动重装的值 */
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
    TIM_OCStructInit(&TIM_OCInitStructure);     /* 初始化结构体成员 */  
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;       /* 和 bsp_SetTIMOutPWM_N() 不同 */
    TIM_OCInitStructure.TIM_Pulse = (_ulDutyCycle * usPeriod)  / 100; //设置跳变值，当计数器计数到这个值时，电平发生跳变
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

        if(TIMx == TIM1 || TIMx == TIM8)
         {
                TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;     /* only for TIM1 and TIM8. */   
                TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         /* only for TIM1 and TIM8. */       
                TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;        /* only for TIM1 and TIM8. */
                TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;      /* only for TIM1 and TIM8. */
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

/*
*********************************************************************************************************
*	函 数 名: bsp_SetTIMforInt
*	功能说明: 配置TIM和NVIC，用于简单的定时中断. 开启定时中断。 中断服务程序由应用程序实现。
*	形    参: TIMx : 定时器
*			  _ulFreq : 定时频率 （Hz）。 0 表示关闭。
*			  _PreemptionPriority : 中断优先级分组
*			  _SubPriority : 子优先级
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetTIMforInt(TIM_TypeDef* TIMx, uint32_t _ulFreq, 
                uint8_t _PreemptionPriority, uint8_t _SubPriority)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;

  	/* 使能TIM时钟 */
	if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		RCC_APB2PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);
	else
		RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);

	if (_ulFreq == 0)
	{
		TIM_Cmd(TIMx, DISABLE);		/* 关闭定时输出 */

		/* 关闭TIM定时更新中断 (Update) */
		{
			NVIC_InitTypeDef NVIC_InitStructure;	/* 中断结构体在 misc.h 中定义 */
			uint8_t irq = 0;	/* 中断号, 定义在 stm32f4xx.h */

			if (TIMx == TIM1)
				irq = TIM1_UP_TIM10_IRQn;
			else if (TIMx == TIM2)
				irq = TIM2_IRQn;
			else if (TIMx == TIM3)
				irq = TIM3_IRQn;
			else if (TIMx == TIM4)
				irq = TIM4_IRQn;
			else if (TIMx == TIM5)
				irq = TIM5_IRQn;
			else if (TIMx == TIM6)
				irq = TIM6_DAC_IRQn;
			else if (TIMx == TIM7)
				irq = TIM7_IRQn;
			else if (TIMx == TIM8)
				irq = TIM8_UP_TIM13_IRQn;

			NVIC_InitStructure.NVIC_IRQChannel = irq;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = _PreemptionPriority;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = _SubPriority;
			NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
			NVIC_Init(&NVIC_InitStructure);
		}		
		return;
	}

    /*-----------------------------------------------------------------------
		system_stm32f4xx.c 文件中 static void SetSysClockToHSE(void) 函数对时钟的配置如下：

			//HCLK = SYSCLK 
			RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
			  
			//PCLK2 = HCLK
			RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
			
			//PCLK1 = HCLK
			RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
		APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
	{
		/* APB2 定时器 */
		uiTIMxCLK = SystemCoreClock;
	}
	else	/* APB1 定时器 .  */
	{
		uiTIMxCLK = SystemCoreClock /2;	// SystemCoreClock / 2;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* 分频比 = 1000 */
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
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	if(TIMx == TIM1 || TIMx == TIM8)
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
    
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIMx, ENABLE);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIMx, ENABLE);

	/* 配置TIM定时更新中断 (Update) */
	{
		NVIC_InitTypeDef NVIC_InitStructure;	/* 中断结构体在 misc.h 中定义 */
		uint8_t irq = 0;	/* 中断号, 定义在 stm32f4xx.h */

		if (TIMx == TIM1)
			irq = TIM1_UP_TIM10_IRQn;
		else if (TIMx == TIM2)
			irq = TIM2_IRQn;
		else if (TIMx == TIM3)
			irq = TIM3_IRQn;
		else if (TIMx == TIM4)
			irq = TIM4_IRQn;
		else if (TIMx == TIM5)
			irq = TIM5_IRQn;
		else if (TIMx == TIM6)
			irq = TIM6_DAC_IRQn;
		else if (TIMx == TIM7)
			irq = TIM7_IRQn;
		else if (TIMx == TIM8)
			irq = TIM8_UP_TIM13_IRQn;

		NVIC_InitStructure.NVIC_IRQChannel = irq;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = _PreemptionPriority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = _SubPriority;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}


void bsp_SetTimInputCount(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx)
{
    /************************************************************************* 
   ** TIM3_CH2 为脉冲输入口
   ** 1. 配置GPIO_GPIOA_PIN7 输入
   ** 2. 配置TIM3 计数器在TI2 端的上升沿计数:
   ** 1). TIMx_CCMR1: CC2S =01; 配置通道2检测TI2输入的上升沿
   ** 2). TIMx_CCMR1:IC2F =000; 选择输入滤波器带宽
   ** 3). TIMx_CCER: CC2P =0;  配置上升沿极性
   ** 4). TIMx_SMCR: SMS =111;  选择定时器外部时钟模式1 
   ** 5). TIMx_SMCR: TS =110;  选择TI2作为触发输入源
   ** 6). TIMx_CR1: CEN? =1; 启动计数器
   *************************************************************************/

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    RCC_APB2PeriphClockCmd(bsp_GetRCCofGPIO(GPIOx), ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(bsp_GetRCCofTIM(TIMx), ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; /*定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
                                                 使用的采样频率之间的分频比为1*/
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit( TIMx, &TIM_TimeBaseStructure); // Time base configuration? 
 
    /********************************************************************
   ** tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);**** // CCMR1_IC2F
   ** tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8); // CCMR1_CC2S
   ** 由TIM_TIxExternalCLK1Source_TI2决定了
   ** TIM_ICSelection=TIM_ICSelection_DirectTI:**? CCMR1_CC2S = 01;
   ** TIM_ICPolarity_Rising******** = CCER_CC2P
   ** TIM_TIxExternalCLK1Source_TI2 = TIM_SMCR_TS
   ** 该函数定义了TIM_SlaveMode_External1;外部时钟模式1
   ********************************************************************/

    TIM_TIxExternalClockConfig(TIMx, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);

    TIM_SetCounter(TIMx, 0);// 清零计数器CNT

    TIM_Cmd(TIMx, ENABLE);
}

void bsp_InitTimCounter(void)
{
    bsp_SetTimInputCount(GPIOA, GPIO_Pin_8, TIM3);
    TIM_SetCounter(TIM3, 0); // 清零计数器CNT
    TIM_Cmd(TIM3,ENABLE);
}

uint16_t bsp_GetMicdBValue(void)
{
    uint16_t db_value = 0;
    uint16_t CountPulse = 0;

    TIM_Cmd(TIM3,DISABLE);
    CountPulse = TIM_GetCounter(TIM3) * 20;
    TIM_SetCounter(TIM3, 0); // 清零计数器CNT
    TIM_Cmd(TIM3,ENABLE);

    
    if((CountPulse >= 20) && (CountPulse <= 40))
        db_value = 1100 - CountPulse * 10;
    else if((CountPulse > 40) && (CountPulse <= 100))
        db_value = 533-  CountPulse / 3;
    else if((CountPulse > 100) && (CountPulse <= 500))
        db_value = 503 - 11 * CountPulse / 40;
    else if((CountPulse > 500) && (CountPulse <= 1000))
        db_value = 380 + CountPulse /50;
    else if((CountPulse > 1000) && (CountPulse <= 300))
        db_value = 450 - CountPulse / 20;
    else if((CountPulse > 3000) && (CountPulse <= 10000))
        db_value = 215 + CountPulse / 35;

    return db_value;
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) ********************************/
