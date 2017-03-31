/*
*********************************************************************************************************
*
*	模块名称 : 独立按键驱动模块
*	文件名称 : bsp_key.c
*	版    本 : V1.0
*	说    明 : 扫描独立按键，具有软件滤波机制，具有按键FIFO。可以检测如下事件：
*				(1) 按键按下
*				(2) 按键弹起
*				(3) 长按键
*				(4) 长按时自动连发
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*		V1.1    2013-06-29 armfly  增加1个读指针，用于bsp_Idle() 函数读取系统控制组合键（截屏）
*								   增加 K1 K2 组合键 和 K2 K3 组合键，用于系统控制
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/**********************************************************************************************
**	该程序适用于安富莱STM32-X3、STM32-V5开发板
**
**	如果用于其它硬件，请修改GPIO定义和 IsKeyDown1 - IsKeyDown8 函数
**
**	如果用户的按键个数小于8个，你可以将多余的按键全部定义为和第1个按键一样，并不影响程序功能
**	#define KEY_COUNT    8	  这个在 bsp_key.h 文件中定义
***********************************************************************************************/

/////////////////////////////////////////////////////////////////
static KEY_T s_tBtn[KEY_COUNT];
static KEY_FIFO_T s_tKey;		/* 按键FIFO变量,结构体 */
static uint16_t gLastRingKeyValue = 0;
static uint16_t gLastPressKeyValue = 0;
//static uint16_t gMuxKeyValue = 0;
static uint8_t gTest = 0;
/*
*********************************************************************************************************
*	函 数 名: IsKeyDownX
*	功能说明: 判断按键是否按下
*	形    参: 无
*	返 回 值: 返回值1 表示按下，0表示未按下
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
*	函 数 名: bsp_InitKeyHard
*	功能说明: 配置按键对应的GPIO
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 第1步：打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_ALL_KEY, ENABLE);

	/* 第2步：配置所有的按键GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* 设为输入口 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 无需上下拉电阻 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        /* 设为输入口 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SDA;
	GPIO_Init(GPIO_PORT_SDA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_INT;
	GPIO_Init(GPIO_PORT_INT, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        /* 设为输出口 */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SCK;
	GPIO_Init(GPIO_PORT_SCK, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIO_PORT_SCK, GPIO_PIN_SCK);           /* SCK默认输出为高电平*/
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyVar
*	功能说明: 初始化按键变量
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* 对按键FIFO读写指针清零 */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* 给每个按键结构体成员变量赋一组缺省值 */
	for (i = 0; i < KEY_COUNT; i++)
	{
	    s_tBtn[i].KeyDownTick = xTaskGetTickCount();
        s_tBtn[i].KeyUpTick = s_tBtn[i].KeyDownTick;

	    s_tBtn[i].ShortTime = KEY_SHORT_TIME;       /* 长按时间 0 表示不检测长按键事件 */
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* 长按时间 0 表示不检测长按键事件 */
		s_tBtn[i].State = 0;							/* 按键缺省状态，0为未按下 */
        
		//s_tBtn[i].KeyCodeDown = 3 * i + 1;				/* 按键按下的键值代码 */
		//s_tBtn[i].KeyCodeUp   = 3 * i + 2;				/* 按键弹起的键值代码 */
		//s_tBtn[i].KeyCodeLong = 3 * i + 3;				/* 按键被持续按下的键值代码 */
		
		s_tBtn[i].RepeatTime = 500;						/* 按键连发的时间间隔，0表示不支持连发 */
		s_tBtn[i].RepeatCount = 0;						/* 连发计数器 */
        
	}

	/* 如果需要单独更改某个按键的参数，可以在此单独重新赋值 */
	/* 比如，我们希望按键1按下超过1秒后，自动重发相同键值 */
	//s_tBtn[KID_JOY_U].LongTime = 100;
	//s_tBtn[KID_JOY_U].RepeatSpeed = 5;	/* 每隔50ms自动发送键值 */


	/* 判断按键按下的函数 */
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

	/* 组合键 */
	s_tBtn[10].IsKeyDownFunc = IsKeyDown10;
    s_tBtn[11].IsKeyDownFunc = IsKeyDown11;
}

/*
********************************************************************************
* 函 数 名: bsp_InitKeyEXTI
* 功能说明: 将所有的按键配置成外部中断触发方式，除了摇杆右键，因为摇杆右键和按键三
* 使用的中断线都是EXTI_Line11，同时使用的话，只会一个有效
* 形 参: 无
* 返 回 值: 无
********************************************************************************
*/
static void bsp_InitKeyEXTI(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* 使能SYSCFG时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* 连接 EXTI Line8 到 PI8 引脚 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    
    /* 配置 EXTI LineXXX */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /* 设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    /* 中断优先级配置 最低优先级 这里一定要分开的设置中断，不能够合并到一个里面设置 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitKey
*	功能说明: 初始化按键. 该函数被 bsp_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* 初始化按键变量 */
	bsp_InitKeyHard();		/* 初始化按键硬件 */
	bsp_InitKeyEXTI();      /* 配置K1为外部中断触发 */
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PutKey
*	功能说明: 将1个键值压入按键FIFO缓冲区。可用于模拟一个按键。
*	形    参:  _KeyCode : 按键代码
*	返 回 值: 无
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
*	函 数 名: bsp_GetKey
*	功能说明: 从按键FIFO缓冲区读取一个键值。
*	形    参:  无
*	返 回 值: 按键代码
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

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKey2
*	功能说明: 从按键FIFO缓冲区读取一个键值。独立的读指针。
*	形    参:  无
*	返 回 值: 按键代码
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
*	函 数 名: bsp_GetKeyState
*	功能说明: 读取按键的状态
*	形    参:  _ucKeyID : 按键ID，从0开始
*	返 回 值: 1 表示按下， 0 表示未按下
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetKeyParam
*	功能说明: 设置按键参数
*	形    参：_ucKeyID : 按键ID，从0开始
*			_LongTime : 长按事件时间
*			 _RepeatSpeed : 连发速度
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatTime)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* 长按时间 0 表示不检测长按键事件 */
	s_tBtn[_ucKeyID].RepeatTime = _RepeatTime;			/* 按键连发的速度，0表示不支持连发 */
	s_tBtn[_ucKeyID].RepeatCount = 0;						/* 连发计数器 */
}


/*
*********************************************************************************************************
*	函 数 名: bsp_ClearKey
*	功能说明: 清空按键FIFO缓冲区
*	形    参：无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_TouchKeyScan
*	功能说明: 扫描所有按键。非阻塞，被systick中断周期性的调用
*	形    参:  无
*	返 回 值: 无
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
        if (pBtn->State) //有按键按下
	    {
	        if(Tick_Current - pBtn->KeyDownTick > pBtn->LongTime) //连续按键
	        {
	            if (pBtn->RepeatTime > 0)
				{
					if((Tick_Current - pBtn->KeyDownTick - pBtn->LongTime)
                        / pBtn->RepeatTime == 0)
					{
						pBtn->RepeatCount++;
						/* 常按键后，每隔10ms发送1个按键 */
						bsp_PutKey((uint8_t)(3 * i + 1));
					}
				}
	        }
        }
        else //按键已经松开
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
*	函 数 名: bsp_TouchKeyScan
*	功能说明: 扫描所有按键。非阻塞，被systick中断周期性的调用
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/

void bsp_TouchKeyScan_new(void)
{
	uint8_t i;
    KEY_T *pBtn;
    TickType_t Tick_Current = 0;

    Tick_Current = xTaskGetTickCount();
	for (i = 0; i < KEY_COUNT; i++)
	{
	    pBtn = &s_tBtn[i];
        if (pBtn->State) //有按键按下
	    {
	        if(Tick_Current - pBtn->KeyDownTick > pBtn->LongTime) //连续按键
	        {
	            if (pBtn->RepeatTime > 0)
				{
					if((Tick_Current - pBtn->KeyDownTick - pBtn->LongTime)
                        / pBtn->RepeatTime == 0)
					{
						pBtn->RepeatCount++;
						/* 常按键后，每隔10ms发送1个按键 */
						bsp_PutKey((uint8_t)(3 * i + 1));
					}
				}
	        }
        }
        else //按键已经松开
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
*	函 数 名: bsp_KeyCodeValueRead
*	功能说明: 读取按键的键值并写入FIFO缓冲区
*	形    参:  无
*	返 回 值: 无
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

	/* 读取按键的键值 */
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


    if(_KeyCodeValue == 0)//单个 松开案件后才处理
    {
        gLastRingKeyValue = 0;
        if(gLastPressKeyValue != 0)
        {
            switch(gLastPressKeyValue)
            {
                case KEY6_VAL:
                    if(s_tBtn[6].KeyUpTick - s_tBtn[6].KeyDownTick < 3000) bsp_PutKey(KEY_6_DOWN);
                    else bsp_PutKey(KEY_6_DOWN_LONG);
                    break;
                case KEY7_VAL:
                    if(s_tBtn[7].KeyUpTick - s_tBtn[7].KeyDownTick < 3000) bsp_PutKey(KEY_7_DOWN);
                    else bsp_PutKey(KEY_7_DOWN_LONG);
                    break;
                case KEY8_VAL:
                    if(s_tBtn[8].KeyUpTick - s_tBtn[8].KeyDownTick < 3000) bsp_PutKey(KEY_8_DOWN);
                    else bsp_PutKey(KEY_8_DOWN_LONG);
                    break;
                case KEY9_VAL:
                    if(s_tBtn[9].KeyUpTick - s_tBtn[9].KeyDownTick < 3000) bsp_PutKey(KEY_9_DOWN);
                    else bsp_PutKey(KEY_9_DOWN_LONG);
                    break;
            }

            gLastPressKeyValue = 0;
        }
    }
    else if(_KeyCodeValue > KEY_NONE && _KeyCodeValue < KEY6_VAL)//0-5
    {
        if((gLastRingKeyValue == KEY0_VAL && _KeyCodeValue == KEY1_VAL)
           || (gLastRingKeyValue == KEY1_VAL && _KeyCodeValue == KEY2_VAL)
           || (gLastRingKeyValue == KEY2_VAL && _KeyCodeValue == KEY3_VAL)
           || (gLastRingKeyValue == KEY3_VAL && _KeyCodeValue == KEY4_VAL)
           || (gLastRingKeyValue == KEY4_VAL && _KeyCodeValue == KEY5_VAL)
           || (gLastRingKeyValue == KEY5_VAL && _KeyCodeValue == KEY0_VAL) )
        {
            bsp_PutKey(KEY_VOL_DOWN);
        }
        else if((gLastRingKeyValue == KEY0_VAL && _KeyCodeValue == KEY5_VAL)
           || (gLastRingKeyValue == KEY1_VAL && _KeyCodeValue == KEY0_VAL)
           || (gLastRingKeyValue == KEY2_VAL && _KeyCodeValue == KEY1_VAL)
           || (gLastRingKeyValue == KEY3_VAL && _KeyCodeValue == KEY2_VAL)
           || (gLastRingKeyValue == KEY4_VAL && _KeyCodeValue == KEY3_VAL)
           || (gLastRingKeyValue == KEY5_VAL && _KeyCodeValue == KEY4_VAL) )
        {
            bsp_PutKey(KEY_VOL_UP);
        }
        gLastRingKeyValue = _KeyCodeValue; 
    }
    else// 6789
    {
        if(gLastPressKeyValue == 0)
            gLastPressKeyValue = _KeyCodeValue;
        else
        {
            if((gLastPressKeyValue == KEY6_VAL && _KeyCodeValue == KEY7_VAL)
                || gLastPressKeyValue == KEY7_VAL && _KeyCodeValue == KEY6_VAL)
            {
                if(((s_tBtn[7].KeyUpTick > s_tBtn[7].KeyDownTick) 
                    && ((s_tBtn[7].KeyUpTick - s_tBtn[7].KeyDownTick) < 3000))
                    ||((s_tBtn[6].KeyUpTick > s_tBtn[6].KeyDownTick) 
                    && ((s_tBtn[6].KeyUpTick - s_tBtn[6].KeyDownTick) < 3000)))
                {
                    bsp_PutKey(KEY_11_DOWN);
                }    
                else
                    bsp_PutKey(KEY_11_DOWN_LONG);
            }
            else if((gLastPressKeyValue == KEY8_VAL && _KeyCodeValue == KEY9_VAL)
                || (gLastPressKeyValue == KEY9_VAL && _KeyCodeValue == KEY8_VAL))
            {
                if(((s_tBtn[9].KeyUpTick > s_tBtn[9].KeyDownTick) 
                    && ((s_tBtn[9].KeyUpTick - s_tBtn[9].KeyDownTick) < 3000))
                    ||((s_tBtn[8].KeyUpTick > s_tBtn[8].KeyDownTick) 
                    && ((s_tBtn[8].KeyUpTick - s_tBtn[8].KeyDownTick) < 3000)))
                {
                    bsp_PutKey(KEY_10_DOWN);
                }
                else
                    bsp_PutKey(KEY_10_DOWN_LONG);
            }
            gLastPressKeyValue = 0;
        }
    }
    
	RTC_WriteBackupRegister(RTC_BKP_DR0, _KeyCodeValue);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetLedLight
*	功能说明: 设置按键背光灯亮度
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetLedLight(uint8_t LightValue)
{
    bsp_SetTIMOutPWM(GPIOC, GPIO_Pin_9, TIM8, 4, 2000, LightValue);	// TIM8_CH4N
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
