/*
*********************************************************************************************************
*
*	ģ������ : ��������ģ��
*	�ļ����� : bsp_key.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#define LIGHT_CLOSE 0
#define LIGHT_WEAK 30
#define LIGHT_HIGH  70

#define KEY_COUNT    12	   					/* ��������, 10�������� + 2����ϼ� */

/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_KEY 	(RCC_AHB1Periph_GPIOC)

#define GPIO_PORT_INT    GPIOC
#define GPIO_PIN_INT	    GPIO_Pin_1

#define GPIO_PORT_SDA    GPIOC
#define GPIO_PIN_SDA	    GPIO_Pin_2

#define GPIO_PORT_SCK    GPIOC
#define GPIO_PIN_SCK	    GPIO_Pin_3


/* ����Ӧ�ó���Ĺ��������������� */
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

#define KEY_DOWN_VOL4		KEY_4_DOWN		/* �� */
#define KEY_UP_VOL4		KEY_4_UP
#define KEY_LONG_VOL4		KEY_4_LONG

#define KEY_DOWN_VOL5		KEY_5_DOWN		/* �� */
#define KEY_UP_VOL5		KEY_5_UP
#define KEY_LONG_VOL5		KEY_5_LONG

#define KEY_DOWN_PLAY_PAUSE		KEY_6_DOWN		/* �� */
#define KEY_UP_PLAY_PAUSE		KEY_6_UP
#define KEY_LONG_PLAY_PAUSE		KEY_6_LONG

#define KEY_DOWN_MENU		KEY_7_DOWN		/* �� */
#define KEY_UP_MENU		KEY_7_UP
#define KEY_LONG_MENU		KEY_7_LONG

#define KEY_DOWN_UP		KEY_8_DOWN		/* ok */
#define KEY_UP_UP		KEY_8_UP
#define KEY_LONG_UP		KEY_8_LONG

#define KEY_DOWN_DOWN	KEY_9_DOWN		
#define KEY_UP_DOWN	    KEY_9_UP
#define KEY_LONG_DOWN	KEY_9_LONG

#define KEY_DOWN_MUX	KEY_10_DOWN		/* K2 K3 ��ϼ� */
#define KEY_UP_MUX  	KEY_10_UP
#define KEY_LONG_MUX	KEY_10_LONG

#define KEY_DOWN_DEBUG	KEY_11_DOWN		/* K2 K3 ��ϼ� */
#define KEY_UP_DEBUG  	KEY_11_UP
#define KEY_LONG_DEBUG	KEY_11_LONG


/* ����ID, ��Ҫ����bsp_KeyState()��������ڲ��� */
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
	�����˲�ʱ��50ms, ��λ10ms��
	ֻ��������⵽50ms״̬�������Ϊ��Ч����������Ͱ��������¼�
	��ʹ������·����Ӳ���˲������˲�����Ҳ���Ա�֤�ɿ��ؼ�⵽�����¼�
*/
#define KEY_SHORT_TIME   1000           /* 1 ������Ϊ�Ƕ̰�  */
#define KEY_LONG_TIME     3000			/* 3000ms�� ����3�룬��Ϊ�����¼� */

/*
	ÿ��������Ӧ1��ȫ�ֵĽṹ�������
*/
typedef struct
{
	/* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
	uint8_t (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */

	TickType_t  KeyDownTick;			/* ��������ʱ��ϵͳtickֵ */
    TickType_t  KeyUpTick;     /* �����ɿ���ʱ��ϵͳtickֵ */
	uint16_t ShortTime;		/* �������³���ʱ��, 0��ʾ�����̰� */
	uint16_t LongTime;		/* �������³���ʱ��, 0��ʾ����ⳤ�� */
	uint8_t  State;			/* ������ǰ״̬�����»��ǵ��� */
	uint16_t  RepeatTime;	/* ���������������� */
	uint8_t  RepeatCount;	/* �������������� */
}KEY_T;

/*
	�����ֵ����, ���밴���´���ʱÿ�����İ��¡�����ͳ����¼�

	�Ƽ�ʹ��enum, ����#define��ԭ��
	(1) ����������ֵ,�������˳��ʹ���뿴���������
	(2) �������ɰ����Ǳ����ֵ�ظ���
*/
typedef enum
{
	KEY_NONE = 0,			/* 0 ��ʾ�����¼� */

	KEY_0_DOWN,				/* 1������ */
	KEY_0_UP,				/* 1������ */
	KEY_0_LONG,				/* 1������ */

	KEY_1_DOWN,				/* 1������ */
	KEY_1_UP,				/* 1������ */
	KEY_1_LONG,				/* 1������ */

	KEY_2_DOWN,				/* 2������ */
	KEY_2_UP,				/* 2������ */
	KEY_2_LONG,				/* 2������ */

	KEY_3_DOWN,				/* 3������ */
	KEY_3_UP,				/* 3������ */
	KEY_3_LONG,				/* 3������ */

	KEY_4_DOWN,				/* 4������ */
	KEY_4_UP,				/* 4������ */
	KEY_4_LONG,				/* 4������ */

	KEY_5_DOWN,				/* 5������ */
	KEY_5_UP,				/* 5������ */
	KEY_5_LONG,				/* 5������ */

	KEY_6_DOWN,				/* 6������ */
	KEY_6_UP,				/* 6������ */
	KEY_6_LONG,				/* 6������ */

	KEY_7_DOWN,				/* 7������ */
	KEY_7_UP,				/* 7������ */
	KEY_7_LONG,				/* 7������ */

	KEY_8_DOWN,				/* 8������ */
	KEY_8_UP,				/* 8������ */
	KEY_8_LONG,				/* 8������ */

	KEY_9_DOWN,				/* 9������ */
	KEY_9_UP,				/* 9������ */
	KEY_9_LONG,				/* 9������ */

	/* ��ϼ� */
	KEY_10_DOWN,			/* 10������ */
	KEY_10_UP,				/* 10������ */
	KEY_10_LONG,			/* 10������ */

    KEY_11_DOWN,			/* 10������ */
	KEY_11_UP,				/* 10������ */
	KEY_11_LONG,			/* 10������ */
}KEY_ENUM;

/* ����FIFO�õ����� */
#define KEY_FIFO_SIZE	10
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* ��ֵ������ */
	uint8_t Read;					/* ��������ָ��1 */
	uint8_t Write;					/* ������дָ�� */
	uint8_t Read2;					/* ��������ָ��2 */
}KEY_FIFO_T;

/* ���ⲿ���õĺ������� */
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
