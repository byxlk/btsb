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
#define KEY_DOWN_VOL_DOWN		KEY_VOL_DOWN
#define KEY_DOWN_VOL_UP		KEY_VOL_UP

#define KEY_DOWN_PLAY_PAUSE		KEY_6_DOWN		/* �� */
#define KEY_DOWN_PLAY_PAUSE_LONG KEY_6_DOWN_LONG
#define KEY_DOWN_MENU		KEY_7_DOWN		/* �� */
#define KEY_DOWN_MENU_LONG		KEY_7_DOWN_LONG

#define KEY_DOWN_UP		KEY_8_DOWN		/* ok */
#define KEY_DOWN_UP_LONG		KEY_8_DOWN_LONG

#define KEY_DOWN_DOWN	KEY_9_DOWN		
#define KEY_DOWN_DOWN_LONG	KEY_9_DOWN_LONG	


#define KEY_DOWN_MUX	KEY_10_DOWN		/* K2 K3 ��ϼ� */
#define KEY_DOWN_MUX_LONG	KEY_10_DOWN_LONG	

#define KEY_DOWN_DEBUG	KEY_11_DOWN		/* K2 K3 ��ϼ� */
#define KEY_DOWN_DEBUG_LONG	KEY_11_DOWN_LONG


//���尴����ֵ
#define KEY0_VAL 0X1
#define KEY1_VAL (0X1 << 1)
#define KEY2_VAL (0X1 << 2)
#define KEY3_VAL (0X1 << 3)
#define KEY4_VAL (0X1 << 4)
#define KEY5_VAL (0X1 << 5)
#define KEY6_VAL (0X1 << 6)
#define KEY7_VAL (0X1 << 7)
#define KEY8_VAL (0X1 << 8)
#define KEY9_VAL (0X1 << 9)

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
	KID_PLAY_PAUSE_LONG,
	KID_KEY_MENU,
	KID_KEY_MENU_LONG,
	KID_KEY_UP,
	KID_KEY_UP_LONG,
	KID_KEY_DOWN,
	KID_KEY_DOWN_LONG,
	KID_KEY_MUX,
	KID_KEY_MUX_LONG,
	KID_KEY_DEBUG,
	KID_KEY_DEBUG_LONG
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

	KEY_VOL_DOWN,				/* 1������ */
	KEY_VOL_UP,				/* 5������ */
	KEY_6_DOWN,				/* 6������ */
	KEY_6_DOWN_LONG,
	KEY_7_DOWN,				/* 7������ */
	KEY_7_DOWN_LONG,
	KEY_8_DOWN,				/* 8������ */
	KEY_8_DOWN_LONG,
	KEY_9_DOWN,				/* 9������ */
	KEY_9_DOWN_LONG,

	/* ��ϼ� */
	KEY_10_DOWN,			/* 10������ */
	KEY_10_DOWN_LONG,
    KEY_11_DOWN,			/* 10������ */
    KEY_11_DOWN_LONG

}KEY_ENUM;

/* ����FIFO�õ����� */
#define KEY_FIFO_SIZE	16
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
