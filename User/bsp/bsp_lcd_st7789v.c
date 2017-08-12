/*
*********************************************************************************************************
*
*	ģ������ : TFTҺ����ʾ������ģ��
*	�ļ����� : LCD_ST7789V.c
*	��    �� : V1.0
*	˵    �� : ST7789V ��ʾ���ֱ���Ϊ 480 * 320,  3.5����ͨ����4��3
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2014-07-26 armfly  �װ�
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
//#include "fonts.h"

#define ST7789V_BASE       ((uint32_t)(0x6C000000 | 0x00000000))

#define ST7789V_REG		*(__IO uint16_t *)(ST7789V_BASE)
#define ST7789V_RAM		*(__IO uint16_t *)(ST7789V_BASE + (1 << (0 + 1)))	/* FSMC 16λ����ģʽ�£�FSMC_A0���߶�Ӧ�����ַA1 */

static __IO uint8_t s_RGBChgEn = 0;		/* RGBת��ʹ��, 4001��д�Դ������RGB��ʽ��д��Ĳ�ͬ */

static void Init_7789(void);
static void ST7789V_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth);
static void ST7789V_QuitWinMode(void);
static void ST7789V_SetCursor(uint16_t _usX, uint16_t _usY);
static void ST7789V_ReadData(uint8_t *readBuf, uint8_t _uCount);
static void ST7789V_WriteCmd(uint8_t _ucCmd);
static void ST7789V_WriteParam(uint8_t _ucParam);

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_InitHard
*	����˵��: ��ʼ��LCD
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint32_t ST7789V_InitHard(void)
{
	uint32_t id;

	id = ST7789V_ReadID();

	if (id == IC_ST7789V_ID)
	{
		Init_7789();	/* ��ʼ��5420��4001��Ӳ�� */

		s_RGBChgEn = 0;

		//ST7789V_PutPixel(1,1, 0x12);
		//g_ChipID = ST7789V_GetPixel(1,1);

		//ST7789V_PutPixel(1,1, 0x34);
		//g_ChipID = ST7789V_GetPixel(1,1);

		//ST7789V_PutPixel(1,1, 0x56);
		//g_ChipID = ST7789V_GetPixel(1,1);

		g_ChipID = IC_7789;

                return IC_7789;
	}

        return IC_UNKN;
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_SetDirection
*	����˵��: ������ʾ����
*	��    ��:  _ucDir : ��ʾ������� 0 ��������, 1=����180�ȷ�ת, 2=����, 3=����180�ȷ�ת
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_SetDirection(uint8_t _ucDir)
{
	/*
		Memory Access Control (36h)
		This command defines read/write scanning direction of the frame memory.

		These 3 bits control the direction from the MPU to memory write/read.

		Bit  Symbol  Name  Description
		D7   MY  Row Address Order
		D6   MX  Column Address Order
		D5   MV  Row/Column Exchange
		D4   ML  Vertical Refresh Order  LCD vertical refresh direction control. ��

		D3   BGR RGB-BGR Order   Color selector switch control
		     (0 = RGB color filter panel, 1 = BGR color filter panel )
		D2   MH  Horizontal Refresh ORDER  LCD horizontal refreshing direction control.
		D1   X   Reserved  Reserved
		D0   X   Reserved  Reserved
	*/
	ST7789V_WriteCmd(0x36);
	/* 0 ��ʾ����(��������)��1��ʾ����(��������), 2��ʾ����(���������)  3��ʾ���� (�������ұ�) */
	if (_ucDir == 0)
	{
		ST7789V_WriteParam(0xA0);	/* ����(���������) */
		g_LcdHeight = 240;
		g_LcdWidth = 320;
	}
	else if (_ucDir == 1)
	{
		ST7789V_WriteParam(0x60);	/* ���� (�������ұ�) */
		g_LcdHeight = 240;
		g_LcdWidth = 320;
	}
	else if (_ucDir == 2)
	{
		ST7789V_WriteParam(0xC0);	/* ����(��������) */
		g_LcdHeight = 320;
		g_LcdWidth = 240;
	}
	else if (_ucDir == 3)
	{
		ST7789V_WriteParam(0x00);	/* ����(��������) */
		g_LcdHeight = 320;
		g_LcdWidth = 240;
	}
}

static void ST7789V_SetGammaCtrl(void)
{
	ST7789V_WriteCmd(0xe0);  //PVGAMCTRL (E0h): Positive Voltage Gamma Control
	ST7789V_WriteParam(0xf0);
	ST7789V_WriteParam(0x00);
	ST7789V_WriteParam(0x0a);
	ST7789V_WriteParam(0x10);
	ST7789V_WriteParam(0x12);
	ST7789V_WriteParam(0x1b);
	ST7789V_WriteParam(0x39);
	ST7789V_WriteParam(0x44);
	ST7789V_WriteParam(0x47);
	ST7789V_WriteParam(0x28);
	ST7789V_WriteParam(0x12);
	ST7789V_WriteParam(0x10);
	ST7789V_WriteParam(0x16);
	ST7789V_WriteParam(0x1b);

	ST7789V_WriteCmd(0xe1);  //NVGAMCTRL (E1h): Negative Voltage Gamma Control
	ST7789V_WriteParam(0xf0);
	ST7789V_WriteParam(0x00);
	ST7789V_WriteParam(0x0a);
	ST7789V_WriteParam(0x10);
	ST7789V_WriteParam(0x11);
	ST7789V_WriteParam(0x1a);
	ST7789V_WriteParam(0x3b);
	ST7789V_WriteParam(0x34);
	ST7789V_WriteParam(0x4e);
	ST7789V_WriteParam(0x3a);
	ST7789V_WriteParam(0x17);
	ST7789V_WriteParam(0x16);
	ST7789V_WriteParam(0x21);
	ST7789V_WriteParam(0x22);

}

/*
*********************************************************************************************************
*	�� �� ��: Init_7789
*	����˵��: ��ʼ��ST7789V������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void Init_7789(void)
{
	/* ��ʼ��LCD��дLCD�Ĵ����������� */

	//************* Start Initial Sequence **********//
        //-------------------------------- SOFTWARE RESET --------------------------------------------------------//
        
        //---------------------------------------------------------------------------------------------------//
        ST7789V_WriteCmd(0x11);     //Sleep out
        bsp_DelayMS(120); //Delay 120ms
        //--------------------------------Display and color format setting-------------------
        ST7789V_WriteCmd(0x36); //Memory data access contro (MADCTL)
        ST7789V_WriteParam(0x00);
        ST7789V_WriteCmd(0x3a);  //Interface pixel format (COLMOD)
        ST7789V_WriteParam(0x55);//16bit/pix 565
        //--------------------------------ST7789S Frame rate setting----------------------------------//
        //Set reg B2 as default value
        ST7789V_WriteCmd(0xb2);  //PORCTRL (B2h): Porch Setting
        ST7789V_WriteParam(0x0c);//BPA[6:0]: Back porch setting in normal mode. The minimum setting is 0x01
        ST7789V_WriteParam(0x0c);//FPA[6:0]: Front porch setting in normal mode. The minimum setting is 0x01
        ST7789V_WriteParam(0x00);//Disable separate porch control
        ST7789V_WriteParam(0x33);
        ST7789V_WriteParam(0x33);
 
        ST7789V_WriteCmd(0xb7);  //GCTRL (B7h): Gate Control
        ST7789V_WriteParam(0x35);
        //---------------------------------ST7789S Power setting--------------------------------------//
        ST7789V_WriteCmd(0xbb); //VCOMS (BBh): VCOMS Setting
        ST7789V_WriteParam(0x2b);//offset 1.175V

        ST7789V_WriteCmd(0xc3); //VRHS (C3h): VRH Set
        ST7789V_WriteParam(0x17); // 4.7+/-( vcom+vcom offset+vdv)

        ST7789V_WriteCmd(0xc4); //VDVS (C4h): VDV Set
        ST7789V_WriteParam(0x20);// 0V - default value

        //FRCTRL2 (C6h): Frame Rate Control in Normal Mode
        //Frame rate=10MHz/(320+FPA[6:0]+BPA[6:0])*(250+RTNA[4:0]*16)
        //Frame rate=10MHz/(320+12         +12)         *(250+5*16) = 88HZ
        ST7789V_WriteCmd(0xc6);//NLA2 NLA1 NLA0 (0x00: dot/0x07: column) RTNA4  RTNA3 RTNA2 RTNA1 RTNA0    Frame
        ST7789V_WriteParam(0x05);

        ST7789V_WriteCmd(0xd0); //PWCTRL1 (D0h): Power Control 1
        ST7789V_WriteParam(0xa4);
        ST7789V_WriteParam(0xa2);// dfault value is 81H
        //--------------------------------ST7789S gamma setting---------------------------------------//
        ST7789V_SetGammaCtrl(); //Set Gamma value control

        /* Table1 setting values */
        ST7789V_WriteCmd(0x13); //NORON (13h): Normal Display Mode On
        ST7789V_WriteCmd(0x20); // INVOFF (20h): Display Inversion Off
        ST7789V_DispOn(); //Display on

#if 1
	/* ������ʾ���� */
	ST7789V_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);
#endif
}

void ST7789V_SoftReset(void)
{
        ST7789V_WriteCmd(0X01);
        ST7789V_WriteParam(0X01);
        bsp_DelayMS(10);
}
/*
*********************************************************************************************************
*	�� �� ��: ST7789V_ReadData()
*	����˵��: ��LCD������оƬ��ȡ����
*	��    ��: ��
*	�� �� ֵ: ����
*********************************************************************************************************
*/
static void ST7789V_ReadData(uint8_t *readBuf, uint8_t _uCount)
{
        //CS(CS)=PC9 RS(D/CX)=PC8 WR(WRX)=PC7 RD(RDX)=PC6 RST=PA8

        uint8_t i =0x00;   
        GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ�� GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/*  GPIO ����Ϊ����������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* LCD Data Bus */
	/* LCD Data Bus */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0    /* Data0 */
	                                            | GPIO_Pin_1    /* Data1 */
	                                            | GPIO_Pin_2    /* Data2 */
	                                            | GPIO_Pin_3    /* Data3 */
	                                            | GPIO_Pin_4    /* Data4 */
	                                            | GPIO_Pin_5    /* Data5 */
	                                            | GPIO_Pin_6    /* Data6 */
	                                            | GPIO_Pin_7;   /* Data7 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
        // INIT status
        //GPIO_SetBits(GPIOA, GPIO_Pin_8); //RST = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_8);  //DCX = 1, //DCX = 0;Write Comand
        //GPIO_SetBits(GPIOC, GPIO_Pin_6);  // RD = 1
        //GPIO_SetBits(GPIOC, GPIO_Pin_7);  //WR = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_9);  // CS = 1;
        GPIO_ResetBits(GPIOB, GPIO_Pin_8); // CS = 0

        for(i = 0; i < _uCount; i++)
        {
                GPIO_ResetBits(GPIOB, GPIO_Pin_10); // RD = 0
                GPIO_SetBits(GPIOB, GPIO_Pin_10);  //RD = 1;
                readBuf[i] = (uint8_t)(GPIO_ReadInputData(GPIOB) & 0xFF);
        }
        GPIO_SetBits(GPIOB, GPIO_Pin_8);  // CS = 1;

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* LCD Data Bus */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0    /* Data0 */
	                                            | GPIO_Pin_1    /* Data1 */
	                                            | GPIO_Pin_2    /* Data2 */
	                                            | GPIO_Pin_3    /* Data3 */
	                                            | GPIO_Pin_4    /* Data4 */
	                                            | GPIO_Pin_5    /* Data5 */
	                                            | GPIO_Pin_6    /* Data6 */
	                                            | GPIO_Pin_7;   /* Data7 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: ST7789V_WriteCmd
*	����˵��: ��LCD������оƬ��������
*	��    ��: _ucCmd : �������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ST7789V_WriteCmd(uint8_t _ucCmd)
{
        //CS(CS)=PC9 RS(D/CX)=PC8 WR(WRX)=PC7 RD(RDX)=PC6 RST=PA8

        // INIT status
        //GPIO_SetBits(GPIOA, GPIO_Pin_8); //RST = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_8);  //DCX = 1, //DCX = 0;Write Comand
        //GPIO_SetBits(GPIOC, GPIO_Pin_6);  // RD = 1
        //GPIO_SetBits(GPIOC, GPIO_Pin_7);  //WR = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_9);  // CS = 1;
#if 0
        GPIO_ResetBits(GPIOB, GPIO_Pin_8); // CS = 0
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);  //DCX = 0, //DCX = 0;Write Comand
        GPIO_ResetBits(GPIOB, GPIO_Pin_11); // WR = 0
        
        GPIO_Write(GPIOB, (0xff << 8) | _ucCmd);
        
        GPIO_SetBits(GPIOB, GPIO_Pin_11);  //WR = 1;
        GPIO_SetBits(GPIOB, GPIO_Pin_8);  // CS = 1;
        GPIO_SetBits(GPIOB, GPIO_Pin_9);  //DCX = 1, //DCX = 0;Write Comand
 #else
        GPIO_Write(GPIOB, (0xf4 << 8) | _ucCmd);
        GPIO_Write(GPIOB, (0xff << 8) | _ucCmd);
 #endif
}


/*
*********************************************************************************************************
*	�� �� ��: ST7789V_WriteParam
*	����˵��: ��LCD������оƬ���Ͳ���(data)
*	��    ��: _ucParam : ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ST7789V_WriteParam(uint8_t _ucParam)
{
        //CS(CS)=PC9 RS(D/CX)=PC8 WR(WRX)=PC7 RD(RDX)=PC6 RST=PA8

        // INIT status
        //GPIO_SetBits(GPIOA, GPIO_Pin_8); //RST = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_8);  //DCX = 1, //DCX = 0;Write Comand
        //GPIO_SetBits(GPIOC, GPIO_Pin_6);  // RD = 1
        //GPIO_SetBits(GPIOC, GPIO_Pin_7);  //WR = 1;
        //GPIO_SetBits(GPIOC, GPIO_Pin_9);  // CS = 1;
#if 0
        GPIO_ResetBits(GPIOB, GPIO_Pin_8); // CS = 0
        GPIO_ResetBits(GPIOB, GPIO_Pin_11); // WR = 0
        
        GPIO_Write(GPIOB, (0xff << 8) | _ucParam);
        
        GPIO_SetBits(GPIOB, GPIO_Pin_11);  //WR = 1;
        GPIO_SetBits(GPIOB, GPIO_Pin_8);  // CS = 1;
 #else
        GPIO_Write(GPIOB, (0xf6 << 8) | _ucParam);
        GPIO_Write(GPIOB, (0xff << 8) | _ucParam);
 #endif
}
/*
*********************************************************************************************************
*	�� �� ��: ST7789V_SetDispWin
*	����˵��: ������ʾ���ڣ����봰����ʾģʽ��TFT����оƬ֧�ִ�����ʾģʽ�����Ǻ�һ���12864����LCD���������
*	��    ��:
*		_usX : ˮƽ����
*		_usY : ��ֱ����
*		_usHeight: ���ڸ߶�
*		_usWidth : ���ڿ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ST7789V_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth)
{
	ST7789V_WriteCmd(0X2A); 		/* ����X���� */
	ST7789V_WriteParam((uint8_t)((_usX & 0xFF00) >> 8));	/* ��ʼ�� */
	ST7789V_WriteParam((uint8_t)(_usX & 0x00FF));
	ST7789V_WriteParam((uint8_t)(((_usX + _usWidth - 1) & 0xFF00) >> 8));	/* ������ */
	ST7789V_WriteParam((uint8_t)(_usX + _usWidth - 1));

	ST7789V_WriteCmd(0X2B); 				  /* ����Y����*/
	ST7789V_WriteParam((uint8_t)((_usY & 0xFF00) >> 8));   /* ��ʼ�� */
	ST7789V_WriteParam((uint8_t)(_usY & 0x00FF));
	ST7789V_WriteParam((uint8_t)(((_usY + _usHeight - 1) & 0xFF00) >>8));		/* ������ */
	ST7789V_WriteParam((uint8_t)(_usY + _usHeight - 1));
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_SetCursor
*	����˵��: ���ù��λ��
*	��    ��:  _usX : X����; _usY: Y����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ST7789V_SetCursor(uint16_t _usX, uint16_t _usY)
{
	ST7789V_WriteCmd(0X2A); 		/* ����X���� */
	ST7789V_WriteParam(_usX >> 8);	/* �ȸ�8λ��Ȼ���8λ */
	ST7789V_WriteParam(_usX);		/* ������ʼ��ͽ�����*/
	ST7789V_WriteParam(_usX >> 8);	/* �ȸ�8λ��Ȼ���8λ */
	ST7789V_WriteParam(_usX);		/* ������ʼ��ͽ�����*/

    ST7789V_WriteCmd(0X2B); 		/* ����Y����*/
	ST7789V_WriteParam(_usY >> 8);
	ST7789V_WriteParam(_usY);
	ST7789V_WriteParam(_usY >> 8);
	ST7789V_WriteParam(_usY);
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_QuitWinMode
*	����˵��: �˳�������ʾģʽ����Ϊȫ����ʾģʽ
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ST7789V_QuitWinMode(void)
{
	ST7789V_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_ReadID
*	����˵��: ��ȡLCD����оƬID�� 4��bit
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint32_t ST7789V_ReadID(void)
{
	uint8_t buf[4] = {0x0};

        ST7789V_WriteCmd(0x04); //RDDID (Read Display ID)
        ST7789V_ReadData(buf, 4);

	return (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DispOn
*	����˵��: ����ʾ
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DispOn(void)
{
    ST7789V_WriteCmd(0x29); //Display on
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DispOff
*	����˵��: �ر���ʾ
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DispOff(void)
{
    ST7789V_WriteCmd(0x28); //Display on
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_ClrScr
*	����˵��: �����������ɫֵ����
*	��    ��:  _usColor : ����ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_ClrScr(uint16_t _usColor)
{
	uint32_t i;
	uint32_t n;

	ST7789V_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);

	ST7789V_WriteCmd(0x2C); 			/* ׼����д�Դ� */

#if 1		/* �Ż�����ִ���ٶ� */
	n = (g_LcdHeight * g_LcdWidth) >> 3;  // DIV 8;
	for (i = 0; i < n; i++)
	{
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);

		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
	}
#else
	n = g_LcdHeight * g_LcdWidth;
	for (i = 0; i < n; i++)
	{
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
	}
#endif

}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_PutPixel
*	����˵��: ��1������
*	��    ��:
*			_usX,_usY : ��������
*			_usColor  ��������ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor)
{
	ST7789V_SetCursor(_usX, _usY);	/* ���ù��λ�� */

	/* д�Դ� */
	ST7789V_WriteCmd(0x2C);
	ST7789V_WriteParam(_usColor >> 8);
	ST7789V_WriteParam(_usColor);
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_GetPixel
*	����˵��: ��ȡ1������
*	��    ��:
*			_usX,_usY : ��������
*			_usColor  ��������ɫ
*	�� �� ֵ: RGB��ɫֵ ��565��
*********************************************************************************************************
*/
uint16_t ST7789V_GetPixel(uint16_t _usX, uint16_t _usY)
{
	uint8_t buf[3] = {0x0};

	ST7789V_SetCursor(_usX, _usY);	/* ���ù��λ�� */

	ST7789V_WriteCmd(0x2E);
	ST7789V_ReadData(buf, 3); 	/* ��1���ƶ������� */

    return ((buf[1] << 8) | buf[2]);
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawLine
*	����˵��: ���� Bresenham �㷨����2��仭һ��ֱ�ߡ�
*	��    ��:
*			_usX1, _usY1 ����ʼ������
*			_usX2, _usY2 ����ֹ��Y����
*			_usColor     ����ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor)
{
	int32_t dx , dy ;
	int32_t tx , ty ;
	int32_t inc1 , inc2 ;
	int32_t d , iTag ;
	int32_t x , y ;

	/* ���� Bresenham �㷨����2��仭һ��ֱ�� */

	ST7789V_PutPixel(_usX1 , _usY1 , _usColor);

	/* ��������غϣ���������Ķ�����*/
	if ( _usX1 == _usX2 && _usY1 == _usY2 )
	{
		return;
	}

	iTag = 0 ;
	/* dx = abs ( _usX2 - _usX1 ); */
	if (_usX2 >= _usX1)
	{
		dx = _usX2 - _usX1;
	}
	else
	{
		dx = _usX1 - _usX2;
	}

	/* dy = abs ( _usY2 - _usY1 ); */
	if (_usY2 >= _usY1)
	{
		dy = _usY2 - _usY1;
	}
	else
	{
		dy = _usY1 - _usY2;
	}

	if ( dx < dy )   /*���dyΪ�Ƴ������򽻻��ݺ����ꡣ*/
	{
		uint16_t temp;

		iTag = 1 ;
		temp = _usX1; _usX1 = _usY1; _usY1 = temp;
		temp = _usX2; _usX2 = _usY2; _usY2 = temp;
		temp = dx; dx = dy; dy = temp;
	}
	tx = _usX2 > _usX1 ? 1 : -1 ;    /* ȷ������1���Ǽ�1 */
	ty = _usY2 > _usY1 ? 1 : -1 ;
	x = _usX1 ;
	y = _usY1 ;
	inc1 = 2 * dy ;
	inc2 = 2 * ( dy - dx );
	d = inc1 - dx ;
	while ( x != _usX2 )     /* ѭ������ */
	{
		if ( d < 0 )
		{
			d += inc1 ;
		}
		else
		{
			y += ty ;
			d += inc2 ;
		}
		if ( iTag )
		{
			ST7789V_PutPixel ( y , x , _usColor) ;
		}
		else
		{
			ST7789V_PutPixel ( x , y , _usColor) ;
		}
		x += tx ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawHLine
*	����˵��: ����һ��ˮƽ�� ����Ҫ����UCGUI�Ľӿں�����
*	��    ��:  _usX1    ����ʼ��X����
*			  _usY1    ��ˮƽ�ߵ�Y����
*			  _usX2    ��������X����
*			  _usColor : ��ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor)
{
	uint16_t i;

	ST7789V_SetDispWin(_usX1, _usY1, 1, _usX2 - _usX1 + 1);

	ST7789V_WriteCmd(0x2C);

	/* д�Դ� */
	for (i = 0; i <_usX2-_usX1 + 1; i++)
	{
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: LCD9341_DrawVLine
*	����˵��: ����ֱƽ�� ��UCGUI�Ľӿں���
*	��    ��: X,Y���������ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawVLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usY2 , uint16_t _usColor)
{
	uint16_t i;
	
	for (i = _usY1; i <=_usY2; i++)
	{	
		ST7789V_PutPixel(_usX1, i, _usColor);	
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawHColorLine
*	����˵��: ����һ����ɫˮƽ�� ����Ҫ����UCGUI�Ľӿں�����
*	��    ��:  _usX1    ����ʼ��X����
*			  _usY1    ��ˮƽ�ߵ�Y����
*			  _usWidth ��ֱ�ߵĿ��
*			  _pColor : ��ɫ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawHColorLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor)
{
	uint16_t i, colorValue ;
	
	ST7789V_SetDispWin(_usX1, _usY1, 1, _usWidth);

	ST7789V_WriteCmd(0x2C);

	/* д�Դ� */
	for (i = 0; i <_usWidth; i++)
	{
	        colorValue = *_pColor++;
		ST7789V_WriteParam(colorValue>> 8);
		ST7789V_WriteParam(colorValue);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawHTransLine
*	����˵��: ����һ����ɫ͸����ˮƽ�� ����Ҫ����UCGUI�Ľӿں������� ��ɫֵΪ0��ʾ͸��ɫ
*	��    ��:  _usX1    ����ʼ��X����
*			  _usY1    ��ˮƽ�ߵ�Y����
*			  _usWidth ��ֱ�ߵĿ��
*			  _pColor : ��ɫ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawHTransLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor)
{
	uint16_t i, j;
	uint16_t Index;

	ST7789V_SetCursor(_usX1, _usY1);

	/* д�Դ� */
	ST7789V_WriteCmd(0x2C);
	for (i = 0,j = 0; i < _usWidth; i++, j++)
	{
		Index = *_pColor++;
	    if (Index)
        {
			 ST7789V_WriteParam(Index >> 8);
                        ST7789V_WriteParam(Index);
		}
		else
		{
			ST7789V_SetCursor(_usX1 + j, _usY1);
			ST7789V_WriteCmd(0x2C);
			ST7789V_WriteParam(Index >> 8);
                        ST7789V_WriteParam(Index);
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawRect
*	����˵��: ����ˮƽ���õľ��Ρ�
*	��    ��:
*			_usX,_usY���������Ͻǵ�����
*			_usHeight �����εĸ߶�
*			_usWidth  �����εĿ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	/*
	 ---------------->---
	|(_usX��_usY)        |
	V                    V  _usHeight
	|                    |
	 ---------------->---
		  _usWidth
	*/

	ST7789V_DrawLine(_usX, _usY, _usX + _usWidth - 1, _usY, _usColor);	/* �� */
	ST7789V_DrawLine(_usX, _usY + _usHeight - 1, _usX + _usWidth - 1, _usY + _usHeight - 1, _usColor);	/* �� */

	ST7789V_DrawLine(_usX, _usY, _usX, _usY + _usHeight - 1, _usColor);	/* �� */
	ST7789V_DrawLine(_usX + _usWidth - 1, _usY, _usX + _usWidth - 1, _usY + _usHeight, _usColor);	/* �� */
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_FillRect
*	����˵��: �����Ρ�
*	��    ��:
*			_usX,_usY���������Ͻǵ�����
*			_usHeight �����εĸ߶�
*			_usWidth  �����εĿ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_FillRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	uint32_t i;

	/*
	 ---------------->---
	|(_usX��_usY)        |
	V                    V  _usHeight
	|                    |
	 ---------------->---
		  _usWidth
	*/

	ST7789V_SetDispWin(_usX, _usY, _usHeight, _usWidth);

	ST7789V_WriteCmd(0x2C);
	for (i = 0; i < _usHeight * _usWidth; i++)
	{
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
	}
}


/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawCircle
*	����˵��: ����һ��Բ���ʿ�Ϊ1������
*	��    ��:
*			_usX,_usY  ��Բ�ĵ�����
*			_usRadius  ��Բ�İ뾶
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor)
{
	int32_t  D;			/* Decision Variable */
	uint32_t  CurX;		/* ��ǰ X ֵ */
	uint32_t  CurY;		/* ��ǰ Y ֵ */

	D = 3 - (_usRadius << 1);
	CurX = 0;
	CurY = _usRadius;

	while (CurX <= CurY)
	{
		ST7789V_PutPixel(_usX + CurX, _usY + CurY, _usColor);
		ST7789V_PutPixel(_usX + CurX, _usY - CurY, _usColor);
		ST7789V_PutPixel(_usX - CurX, _usY + CurY, _usColor);
		ST7789V_PutPixel(_usX - CurX, _usY - CurY, _usColor);
		ST7789V_PutPixel(_usX + CurY, _usY + CurX, _usColor);
		ST7789V_PutPixel(_usX + CurY, _usY - CurX, _usColor);
		ST7789V_PutPixel(_usX - CurY, _usY + CurX, _usColor);
		ST7789V_PutPixel(_usX - CurY, _usY - CurX, _usColor);

		if (D < 0)
		{
			D += (CurX << 2) + 6;
		}
		else
		{
			D += ((CurX - CurY) << 2) + 10;
			CurY--;
		}
		CurX++;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_DrawBMP
*	����˵��: ��LCD����ʾһ��BMPλͼ��λͼ����ɨ����򣺴����ң����ϵ���
*	��    ��:
*			_usX, _usY : ͼƬ������
*			_usHeight  ��ͼƬ�߶�
*			_usWidth   ��ͼƬ���
*			_ptr       ��ͼƬ����ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ST7789V_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr)
{
	uint32_t index = 0;
	const uint16_t *p;

	/* ����ͼƬ��λ�úʹ�С�� ��������ʾ���� */
	ST7789V_SetDispWin(_usX, _usY, _usHeight, _usWidth);

	p = _ptr;
	for (index = 0; index < _usHeight * _usWidth; index++)
	{
		ST7789V_PutPixel(_usX, _usY, *p++);
	}

	/* �˳����ڻ�ͼģʽ */
	ST7789V_QuitWinMode();
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
