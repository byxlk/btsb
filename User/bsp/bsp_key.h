/*
*********************************************************************************************************
*
*	模块名称 : 按键驱动模块
*	文件名称 : bsp_key.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#define LIGHT_CLOSE 0
#define LIGHT_WEAK 30
#define LIGHT_HIGH  70

#define KEY_COUNT    12	   					/* 按键个数, 10个独立建 + 2个组合键 */

/* 按键口对应的RCC时钟 */
#define RCC_ALL_KEY 	(RCC_AHB1Periph_GPIOC)

#define GPIO_PORT_INT    GPIOC
#define GPIO_PIN_INT	    GPIO_Pin_1

#define GPIO_PORT_SDA    GPIOC
#define GPIO_PIN_SDA	    GPIO_Pin_2

#define GPIO_PORT_SCK    GPIOC
#define GPIO_PIN_SCK	    GPIO_Pin_3


/* 根据应用程序的功能重命名按键宏 */
#define KEY_DOWN_VOL0		KEY_0_DOWN
#define KEY_UP_VOL0		KEY_0_UP
#define KEY_LONG_VOL0		KEY_0_LONG

#define KEY_DOWN_VOL1		KEY_1_DOWN
#define KEY_UP_VOL1		KEY_1_UP
#define KEY_LONG_VOL1		KEY_1_LONG

#define KEY_DOWN_VOL2		KEY_2_DOWN
#define KEY_UP_VOL2		KEY_2_UP
#define KEY_LONG_VOL2		KEY_2_LONG

#define KEY_DOWN_VOL3		KEY_3_DOWN
#define KEY_UP_VOL3		KEY_3_UP
#define KEY_LONG_VOL3		KEY_3_LONG

#define KEY_DOWN_VOL4		KEY_4_DOWN		/* 上 */
#define KEY_UP_VOL4		KEY_4_UP
#define KEY_LONG_VOL4		KEY_4_LONG

#define KEY_DOWN_VOL5		KEY_5_DOWN		/* 下 */
#define KEY_UP_VOL5		KEY_5_UP
#define KEY_LONG_VOL5		KEY_5_LONG

#define KEY_DOWN_PLAY_PAUSE		KEY_6_DOWN		/* 左 */
#define KEY_UP_PLAY_PAUSE		KEY_6_UP
#define KEY_LONG_PLAY_PAUSE		KEY_6_LONG

#define KEY_DOWN_MENU		KEY_7_DOWN		/* 右 */
#define KEY_UP_MENU		KEY_7_UP
#define KEY_LONG_MENU		KEY_7_LONG

#define KEY_DOWN_UP		KEY_8_DOWN		/* ok */
#define KEY_UP_UP		KEY_8_UP
#define KEY_LONG_UP		KEY_8_LONG

#define KEY_DOWN_DOWN	KEY_9_DOWN		
#define KEY_UP_DOWN	    KEY_9_UP
#define KEY_LONG_DOWN	KEY_9_LONG

#define KEY_DOWN_MUX	KEY_10_DOWN		/* K2 K3 组合键 */
#define KEY_UP_MUX  	KEY_10_UP
#define KEY_LONG_MUX	KEY_10_LONG

#define KEY_DOWN_DEBUG	KEY_11_DOWN		/* K2 K3 组合键 */
#define KEY_UP_DEBUG  	KEY_11_UP
#define KEY_LONG_DEBUG	KEY_11_LONG


/* 按键ID, 主要用于bsp_KeyState()函数的入口参数 */
typedef enum
{
	KID_VOL0 = 0,
	KID_VOL1,
	KID_VOL2,
	KID_VOL3,
	KID_VOL4,
	KID_VOL5,
	KID_PLAY_PAUSE,
	KID_KEY_MENU,
	KID_KEY_UP,
	KID_KEY_DOWN,
	KID_KEY_MUX,
	KID_KEY_DEBUG
}KEY_ID_E;

/*
	按键滤波时间50ms, 单位10ms。
	只有连续检测到50ms状态不变才认为有效，包括弹起和按下两种事件
	即使按键电路不做硬件滤波，该滤波机制也可以保证可靠地检测到按键事件
*/
#define KEY_SHORT_TIME   1000           /* 1 秒内认为是短按  */
#define KEY_LONG_TIME     3000			/* 3000ms， 持续3秒，认为长按事件 */

/*
	每个按键对应1个全局的结构体变量。
*/
typedef struct
{
	/* 下面是一个函数指针，指向判断按键手否按下的函数 */
	uint8_t (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */

	TickType_t  KeyDownTick;			/* 按键按下时的系统tick值 */
    TickType_t  KeyUpTick;     /* 按键松开下时的系统tick值 */
	uint16_t ShortTime;		/* 按键按下持续时间, 0表示不检测短按 */
	uint16_t LongTime;		/* 按键按下持续时间, 0表示不检测长按 */
	uint8_t  State;			/* 按键当前状态（按下还是弹起） */
	uint16_t  RepeatTime;	/* 连续按键发送周期 */
	uint8_t  RepeatCount;	/* 连续按键计数器 */
}KEY_T;

/*
	定义键值代码, 必须按如下次序定时每个键的按下、弹起和长按事件

	推荐使用enum, 不用#define，原因：
	(1) 便于新增键值,方便调整顺序，使代码看起来舒服点
	(2) 编译器可帮我们避免键值重复。
*/
typedef enum
{
	KEY_NONE = 0,			/* 0 表示按键事件 */

	KEY_0_DOWN,				/* 1键按下 */
	KEY_0_UP,				/* 1键弹起 */
	KEY_0_LONG,				/* 1键长按 */

	KEY_1_DOWN,				/* 1键按下 */
	KEY_1_UP,				/* 1键弹起 */
	KEY_1_LONG,				/* 1键长按 */

	KEY_2_DOWN,				/* 2键按下 */
	KEY_2_UP,				/* 2键弹起 */
	KEY_2_LONG,				/* 2键长按 */

	KEY_3_DOWN,				/* 3键按下 */
	KEY_3_UP,				/* 3键弹起 */
	KEY_3_LONG,				/* 3键长按 */

	KEY_4_DOWN,				/* 4键按下 */
	KEY_4_UP,				/* 4键弹起 */
	KEY_4_LONG,				/* 4键长按 */

	KEY_5_DOWN,				/* 5键按下 */
	KEY_5_UP,				/* 5键弹起 */
	KEY_5_LONG,				/* 5键长按 */

	KEY_6_DOWN,				/* 6键按下 */
	KEY_6_UP,				/* 6键弹起 */
	KEY_6_LONG,				/* 6键长按 */

	KEY_7_DOWN,				/* 7键按下 */
	KEY_7_UP,				/* 7键弹起 */
	KEY_7_LONG,				/* 7键长按 */

	KEY_8_DOWN,				/* 8键按下 */
	KEY_8_UP,				/* 8键弹起 */
	KEY_8_LONG,				/* 8键长按 */

	KEY_9_DOWN,				/* 9键按下 */
	KEY_9_UP,				/* 9键弹起 */
	KEY_9_LONG,				/* 9键长按 */

	/* 组合键 */
	KEY_10_DOWN,			/* 10键按下 */
	KEY_10_UP,				/* 10键弹起 */
	KEY_10_LONG,			/* 10键长按 */

    KEY_11_DOWN,			/* 10键按下 */
	KEY_11_UP,				/* 10键弹起 */
	KEY_11_LONG,			/* 10键长按 */
}KEY_ENUM;

/* 按键FIFO用到变量 */
#define KEY_FIFO_SIZE	10
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* 键值缓冲区 */
	uint8_t Read;					/* 缓冲区读指针1 */
	uint8_t Write;					/* 缓冲区写指针 */
	uint8_t Read2;					/* 缓冲区读指针2 */
}KEY_FIFO_T;

/* 供外部调用的函数声明 */
void bsp_InitKey(void);
void bsp_KeyScan(void);
void bsp_TouchKeyScan(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKey2(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed);
void bsp_ClearKey(void);
void bsp_TouchKeyCodeValueProcess(void);
void bsp_SetLedLight(uint8_t LightValue);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
