/********************************************************************************
 * FileName:
 * Author:         YuanYin  Version: QXW-Summer-V1.x  Date: 2010-4-29
 * Description:
 * Version:
 * Function List:
 *                 1.
 * History:
 *     <author>   <time>    <version >   <desc>
 *      YuanYin       4-29         1.x
*******************************************************************************/
#include "Drivers.h"
#include "Lcd.h"
#if LCD_BGLIGHT_PWM_MODE
#include "Time.h"
#endif
#include "Debug.h"

#if 1//QXW_LCM_ID == 220 || QXW_LCM_ID == 221

/* ����3����������Ҫ����ʹ����ͬʱ֧�ֲ�ͬ���� */
uint16_t g_LcdHeight = 240;			/* ��ʾ���ֱ���-�߶� */
uint16_t g_LcdWidth = 320;			/* ��ʾ���ֱ���-��� */
uint8_t s_ucBright;					/* �������Ȳ��� */
uint8_t g_LcdDirection;				/* ��ʾ����.0��1��2��3 */


#define LCD_ILI9320_On() GPIO_SetBits(GPIOC,GPIO_Pin_7)
#define LCD_ILI9320_Off() GPIO_ResetBits(GPIOC,GPIO_Pin_7)

#define Bank1_LCD_R    ((uint32_t)0x60000000)    //disp Reg ADDR
#define Bank1_LCD_D    ((uint32_t)0x60020000)	//disp Data ADDR

static u8 gLcdScale=100;

//9320���ܼĴ�����ַ
#define WINDOW_XADDR_START	0x0050 // ˮƽ�Ŀ�ʼ��ַ��
#define WINDOW_XADDR_END		0x0051 // ˮƽ�Ľ�����ַ��
#define WINDOW_YADDR_START	0x0052 // ��ֱ�Ŀ�ʼ��ַ��
#define WINDOW_YADDR_END		0x0053 // ��ֱ�Ľ�����ַ��
#define GRAM_XADDR		    		0x0020 // GRAM ˮƽ�ĵ�ַ��
#define GRAM_YADDR		    		0x0021 // GRAM ��ֱ�ĵ�ַ��
#define GRAMWR 			    			0x0022 // GRAM

/**********************************************
��������LCD_Delay
���ܣ�����LCD������ʱ
��ڲ�������ʱ��
����ֵ����
***********************************************/
static void LCD_DelayMs(u32 Ms)
{
  u32 i;
	for(; Ms; Ms--)
		for(i=1000;i;i--);
}

/*************************************************
��������LCD_WR_Reg
���ܣ���lcd�Ĵ�����д����
��ڲ������Ĵ�����ַ������
����ֵ���Ĵ���ֵ
*************************************************/
static void LCD_WriteCmd(uint8_t _ucCmd)
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

/*************************************************
��������LCD_RD_Reg
���ܣ���lcd�Ĵ�������ֵ
��ڲ������Ĵ�����ַ
����ֵ���Ĵ���ֵ
*************************************************/
//static u16 LCD_ReadReg(u16 index)
//{
//	*(__IO uint16_t *) (Bank1_LCD_R)= index;
//	return (*(__IO uint16_t *) (Bank1_LCD_D));
//}

/*************************************************
��������LCD_WR_Data
���ܣ���lcdд����
��ڲ���������ֵ
����ֵ����
*************************************************/
static void LCD_WriteParam(uint8_t _ucParam)
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

/*************************************************
��������LCD_RD_Data
���ܣ���lcd����
��ڲ�������
����ֵ������
*************************************************/
static void LCD_ReadData(uint8_t *readBuf, uint8_t _uCount)
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

/**********************************************
��������FSMC_LCD_Init
���ܣ�����FSMC����
��ڲ�������
����ֵ����
***********************************************/
static void LCD_CtrlLinesConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ�� GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	/*  GPIO ����Ϊ����������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* LCD Data Bus */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0    /* Data0 */
                                | GPIO_Pin_1    /* Data1 */
                                | GPIO_Pin_2    /* Data2 */
                                | GPIO_Pin_3    /* Data3 */
                                | GPIO_Pin_4    /* Data4 */
                                | GPIO_Pin_5    /* Data5 */
                                | GPIO_Pin_6    /* Data6 */
                                | GPIO_Pin_7    /* Data7 */
                                | GPIO_Pin_8    /* CS */
                                | GPIO_Pin_9    /* RS */
                                | GPIO_Pin_10  /* RD */
                                | GPIO_Pin_11;  /* WR */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    //Ĭ������ߵ�ƽ
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);

    /* ���ñ���GPIOΪ�������ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_SetDirection
*	����˵��: ������ʾ����
*	��    ��:  _ucDir : ��ʾ������� 0 ��������, 1=����180�ȷ�ת, 2=����, 3=����180�ȷ�ת
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void LCD_SetDirection(uint8_t _ucDir)
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
    g_LcdDirection =  _ucDir;		/* ������ȫ�ֱ��� */
    LCD_WriteCmd(0x36);
    /* 0 ��ʾ����(��������)��1��ʾ����(��������), 2��ʾ����(���������)  3��ʾ���� (�������ұ�) */
    if (_ucDir == 0)
    {
        LCD_WriteParam(0xA0);   /* ����(���������) */
        g_LcdHeight = 240;
        g_LcdWidth = 320;
    }
    else if (_ucDir == 1)
    {
        LCD_WriteParam(0x60);   /* ���� (�������ұ�) */
        g_LcdHeight = 240;
        g_LcdWidth = 320;
    }
    else if (_ucDir == 2)
    {
        LCD_WriteParam(0xC0);   /* ����(��������) */
        g_LcdHeight = 320;
        g_LcdWidth = 240;
    }
    else if (_ucDir == 3)
    {
        LCD_WriteParam(0x00);   /* ����(��������) */
        g_LcdHeight = 320;
        g_LcdWidth = 240;
    }

}

static void LCD_SetGammaCtrl(void)
{
	LCD_WriteCmd(0xe0);  //PVGAMCTRL (E0h): Positive Voltage Gamma Control
	LCD_WriteParam(0xf0);
	LCD_WriteParam(0x00);
	LCD_WriteParam(0x0a);
	LCD_WriteParam(0x10);
	LCD_WriteParam(0x12);
	LCD_WriteParam(0x1b);
	LCD_WriteParam(0x39);
	LCD_WriteParam(0x44);
	LCD_WriteParam(0x47);
	LCD_WriteParam(0x28);
	LCD_WriteParam(0x12);
	LCD_WriteParam(0x10);
	LCD_WriteParam(0x16);
	LCD_WriteParam(0x1b);

	LCD_WriteParam(0xe1);  //NVGAMCTRL (E1h): Negative Voltage Gamma Control
	LCD_WriteParam(0xf0);
	LCD_WriteParam(0x00);
	LCD_WriteParam(0x0a);
	LCD_WriteParam(0x10);
	LCD_WriteParam(0x11);
	LCD_WriteParam(0x1a);
	LCD_WriteParam(0x3b);
	LCD_WriteParam(0x34);
	LCD_WriteParam(0x4e);
	LCD_WriteParam(0x3a);
	LCD_WriteParam(0x17);
	LCD_WriteParam(0x16);
	LCD_WriteParam(0x21);
	LCD_WriteParam(0x22);

}

/*************************************************
��������LCD_Power_On
���ܣ�LCD��������
��ڲ�������
����ֵ����
*************************************************/
static void LCD_PowerOn(void)
{
	LCD_WriteCmd(0x29); //Display on
}

/*************************************************
��������LCD_Power_Off
���ܣ�LCD�ر�����
��ڲ�������
����ֵ����
*************************************************/
static void LCD_PowerOff(void)
{
	LCD_WriteCmd(0x28); //Display on
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
static void LCD_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth)
{
	LCD_WriteCmd(0X2A); 		/* ����X���� */
	LCD_WriteParam((uint8_t)((_usX & 0xFF00) >> 8));	/* ��ʼ�� */
	LCD_WriteParam((uint8_t)(_usX & 0x00FF));
	LCD_WriteParam((uint8_t)(((_usX + _usWidth - 1) & 0xFF00) >> 8));	/* ������ */
	LCD_WriteParam((uint8_t)(_usX + _usWidth - 1));

	LCD_WriteCmd(0X2B); 				  /* ����Y����*/
	LCD_WriteParam((uint8_t)((_usY & 0xFF00) >> 8));   /* ��ʼ�� */
	LCD_WriteParam((uint8_t)(_usY & 0x00FF));
	LCD_WriteParam((uint8_t)(((_usY + _usHeight - 1) & 0xFF00) >>8));		/* ������ */
	LCD_WriteParam((uint8_t)(_usY + _usHeight - 1));
}

static void LCD_HardInit()
{
    /* ��ʼ��LCD��дLCD�Ĵ����������� */

    //************* Start Initial Sequence **********//
    //-------------------------------- SOFTWARE RESET --------------------------------------------------------//

    //---------------------------------------------------------------------------------------------------//
    LCD_WriteCmd(0x11);     //Sleep out
    LCD_DelayMs(120); //Delay 120ms
    //--------------------------------Display and color format setting-------------------
    LCD_WriteCmd(0x36); //Memory data access contro (MADCTL)
    LCD_WriteParam(0x00);
    LCD_WriteCmd(0x3a);  //Interface pixel format (COLMOD)
    LCD_WriteParam(0x55);//16bit/pix 565
    //--------------------------------ST7789S Frame rate setting----------------------------------//
    //Set reg B2 as default value
    LCD_WriteCmd(0xb2);  //PORCTRL (B2h): Porch Setting
    LCD_WriteParam(0x0c);//BPA[6:0]: Back porch setting in normal mode. The minimum setting is 0x01
    LCD_WriteParam(0x0c);//FPA[6:0]: Front porch setting in normal mode. The minimum setting is 0x01
    LCD_WriteParam(0x00);//Disable separate porch control
    LCD_WriteParam(0x33);
    LCD_WriteParam(0x33);

    LCD_WriteCmd(0xb7);  //GCTRL (B7h): Gate Control
    LCD_WriteParam(0x35);
    //---------------------------------ST7789S Power setting--------------------------------------//
    LCD_WriteCmd(0xbb); //VCOMS (BBh): VCOMS Setting
    LCD_WriteParam(0x2b);//offset 1.175V

    LCD_WriteCmd(0xc3); //VRHS (C3h): VRH Set
    LCD_WriteParam(0x17); // 4.7+/-( vcom+vcom offset+vdv)

    LCD_WriteCmd(0xc4); //VDVS (C4h): VDV Set
    LCD_WriteParam(0x20);// 0V - default value

    //FRCTRL2 (C6h): Frame Rate Control in Normal Mode
    //Frame rate=10MHz/(320+FPA[6:0]+BPA[6:0])*(250+RTNA[4:0]*16)
    //Frame rate=10MHz/(320+12         +12)         *(250+5*16) = 88HZ
    LCD_WriteCmd(0xc6);//NLA2 NLA1 NLA0 (0x00: dot/0x07: column) RTNA4  RTNA3 RTNA2 RTNA1 RTNA0    Frame
    LCD_WriteParam(0x05);

    LCD_WriteCmd(0xd0); //PWCTRL1 (D0h): Power Control 1
    LCD_WriteParam(0xa4);
    LCD_WriteParam(0xa2);// dfault value is 81H
    //--------------------------------ST7789S gamma setting---------------------------------------//
    LCD_SetGammaCtrl(); //Set Gamma value control

    /* Table1 setting values */
    LCD_WriteCmd(0x13); //NORON (13h): Normal Display Mode On
    LCD_WriteCmd(0x20); // INVOFF (20h): Display Inversion Off
    LCD_PowerOn(); //Display on

#if 1
    /* ������ʾ���� */
    LCD_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);
#endif

}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_ReadID
*	����˵��: ��ȡLCD����оƬID�� 4��bit
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint32_t LCD_ReadID(void)
{
	uint8_t buf[4] = {0x0};

    LCD_WriteCmd(0x04); //RDDID (Read Display ID)
    LCD_ReadData(buf, 4);

	return (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

/*
*********************************************************************************************************
*	�� �� ��: ST7789V_ClrScr
*	����˵��: �����������ɫֵ����
*	��    ��:  _usColor : ����ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void LCD_ClrScr(uint16_t _usColor)
{
	uint32_t i;
	uint32_t n;

	LCD_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);

	LCD_WriteCmd(0x2C); 			/* ׼����д�Դ� */

#if 1		/* �Ż�����ִ���ٶ� */
	n = (g_LcdHeight * g_LcdWidth) >> 3;  // DIV 8;
	for (i = 0; i < n; i++)
	{
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);

		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
	}
#else
	n = g_LcdHeight * g_LcdWidth;
	for (i = 0; i < n; i++)
	{
		LCD_WriteParam(_usColor >> 8);
		LCD_WriteParam(_usColor);
	}
#endif

}

/*************************************************
��������LCD_WR_Data_Start
���ܣ�LCD��ʼ����������ǰ����
��ڲ�������
����ֵ����
*************************************************/
void LCD_BlukWriteDataStart(void)
{
    LCD_WriteCmd(0x3C);
}

/*************************************************
��������LCD_ReadDataStart
���ܣ�LCD��ʼ����������ǰ����
��ڲ�������
����ֵ����
*************************************************/
void LCD_BulkReadDataStart(void)
{
    LCD_WriteCmd(0x3E);
}

/*************************************************
��������LCD_BulkWriteData
���ܣ���lcd����д����
��ڲ���������ֵ
����ֵ����
*************************************************/
void LCD_BulkWriteData(u16 val)
{
	LCD_WriteParam(val & 0xff);
    LCD_WriteParam((val & 0xff00) >> 8);
}

/*************************************************
��������LCD_BulkReadData
���ܣ�������lcd����
��ڲ�������
����ֵ������
*************************************************/
u16 LCD_BulkReadData(void)
{
    uint8_t buf[2] = {0x0};
    LCD_ReadData(buf, 2);
    return ((buf[0] << 8) | buf[1]);
}

/*************************************************
��������LCD_Set_XY
���ܣ�����lcd��ʾ��ʼ��
��ڲ�����xy����
����ֵ����
*************************************************/
void LCD_SetXY(u16 _usX,u16 _usY)
{
    LCD_WriteCmd(0X2A); 		/* ����X���� */
	LCD_WriteParam(_usX >> 8);	/* �ȸ�8λ��Ȼ���8λ */
	LCD_WriteParam(_usX);		/* ������ʼ��ͽ�����*/
	LCD_WriteParam(_usX >> 8);	/* �ȸ�8λ��Ȼ���8λ */
	LCD_WriteParam(_usX);		/* ������ʼ��ͽ�����*/

    LCD_WriteCmd(0X2B); 		/* ����Y����*/
	LCD_WriteParam(_usY >> 8);
	LCD_WriteParam(_usY);
	LCD_WriteParam(_usY >> 8);
	LCD_WriteParam(_usY);
}

/*************************************************
��������LCD_Set_Region
���ܣ�����lcd��ʾ�����ڴ�����д�������Զ�����
��ڲ�����xy�����յ�,Y_IncMode��ʾ������y������x
����ֵ����
*************************************************/
void LCD_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end,bool yIncFrist)
{
    LCD_WriteCmd(0X2A);         /* ����X���� */
    LCD_WriteParam(x_start >> 8);  /* �ȸ�8λ��Ȼ���8λ */
    LCD_WriteParam(x_start);       /* ������ʼ��ͽ�����*/
    LCD_WriteParam(x_end >> 8);  /* �ȸ�8λ��Ȼ���8λ */
    LCD_WriteParam(x_end);       /* ������ʼ��ͽ�����*/

    LCD_WriteCmd(0X2B);         /* ����Y����*/
    LCD_WriteParam(y_start >> 8);
    LCD_WriteParam(y_start);
    LCD_WriteParam(y_end >> 8);
    LCD_WriteParam(y_end);

  	//{
	//	u16 ModeReg=LCD_ReadReg(0x0003);

	//	if(yIncFrist)
	//		ModeReg|=(0x8);
	//	else
	//		ModeReg&=(~0x8);
	//	LCD_WriteReg(0x0003, ModeReg);
	//}
}

/*************************************************
��������LCD_Set_XY_Addr_Direction
���ܣ�����lcd����д�����ķ���
��ڲ�����0:��0���ߣ�1:�ɸߵ�0
����ֵ����
*************************************************/
void LCD_SetAddrIncMode(LCD_INC_MODE xyDirection)
{
	//register u16 ModeReg=LCD_ReadReg(0x0003);

	//ModeReg&=(~0x30);
	//ModeReg|=((xyDirection)<<4);
	//LCD_WriteReg(0x0003, ModeReg);
}

/*************************************************
��������LCD_BGR_Mode
���ܣ�����lcd RGB˳��
��ڲ�����0:RGB   1:BGR
����ֵ����
*************************************************/
void LCD_BgrMode(bool UseBGR)
{
	//register u16 ModeReg=LCD_ReadReg(0x0003);

	//if(UseBGR)
	//	ModeReg&=(~0x1000);
	//else
	//	ModeReg|=(0x1000);
	//LCD_WriteReg(0x0003, ModeReg);
}

/*************************************************
��������LCD_Addr_Inc
���ܣ���ַ����
��ڲ�������
����ֵ����
*************************************************/
void LCD_AddrInc(void)
{
	//register u16 Color16;
	//LCD_ReadData();
	//Color16=LCD_ReadData();
	//LCD_WriteData((((Color16>>11)&0x001f)|(Color16&0x07e0)|((Color16<<11)&0xf800)));//��16λRGB(565)ɫ�ʻ����16λBGR(565)ɫ��
}

/*************************************************
��������LCD_DrawPoint
���ܣ���һ����
��ڲ�������
����ֵ����
*************************************************/
void LCD_DrawPoint(u16 _usX, u16 _usY, u16 _usColor)
{
    LCD_SetXY(_usX, _usY);	/* ���ù��λ�� */

	/* д�Դ� */
	LCD_WriteCmd(0x2C);
	LCD_WriteParam(_usColor >> 8);
	LCD_WriteParam(_usColor);
}

/*************************************************
��������LCD_DrawPoint
���ܣ���һ����
��ڲ�������
����ֵ����
*************************************************/
u16 LCD_ReadPoint(u16 _usX, u16 _usY)
{
    uint8_t buf[3] = {0x0};

	LCD_SetXY(_usX, _usY);	/* ���ù��λ�� */

	LCD_WriteCmd(0x2E);
	LCD_ReadData(buf, 3); 	/* ��1���ƶ������� */

    return ((buf[1] << 8) | buf[2]);
}

/*************************************************
��������LCD_Light_Set
���ܣ�LCD���ñ�������
��ڲ�����Scale:0-100��0ΪϨ��100����
����ֵ����
*************************************************/
void LCD_Light_Set(u8 Scale)
{
	Debug("LCD Light Set:%d\n\r",Scale);
#if LCD_BGLIGHT_PWM_MODE
    /* STM32-V4�����壬PB1/TIM3_CH4/TIM8_CH3N ���Ʊ���PWM �� ��Ϊ TIM3���ں�����롣�����TIM8_CH3N������PWM */
	//bsp_SetTIMOutPWM(GPIOB, GPIO_Pin_1, TIM3, 4, 100, (_bright * 10000) /255);	// TIM3_CH4
	bsp_SetTIMOutPWM_N(GPIOA, GPIO_Pin_2, TIM2, 3, 2000, Scale);	// TIM2_CH3N
#else
	if(Scale) GPIO_SetBits(GPIOA, GPIO_Pin_2);
	else GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#endif
	gLcdScale=Scale;
}

/*************************************************
��������LCD_Light_State
����:��ѯ��ǰ��������
��ڲ�������
����ֵ��0-100��0ΪϨ��100����
*************************************************/
u8 LCD_Light_State(void)
{
	return gLcdScale;
}

/*************************************************
��������LCD_Init
���ܣ���ʼ������lcd
��ڲ�������
����ֵ����
*************************************************/
void LCD_Init(void)
{
	LCD_CtrlLinesConfig();
	LCD_DelayMs(100);
	LCD_HardReset();

    if(0x858552 == LCD_ReadID()) {
        LCD_HardInit();
        LCD_SetDirection(2);
        LCD_ClrScr(CL_BLACK);   /* ��������ʾȫ�� */
        LCD_Light_Set(50);

#ifdef LCD_DRIVER_TEST
        LCD_ClrScr(CL_RED);
        LCD_Fill_Rect(0, 0, 100, 50, CL_BLUE);
        LCD_DrawCircle(120,  160,  30, CL_YELLOW);
        LCD_DrawLine(0, 0, 240, 320, CL_GREEN);
#endif
    }

	Debug("LCD Driver IC:%x\n\r",LCD_ReadID());
}

void LCD_DeInit(void)
{
	LCD_Light_Set(0);
	LCD_PowerOff();
}

/**********************************************
��������LCD_Reset
���ܣ�LCD��λ
��ڲ�������ʱ��
����ֵ����
***********************************************/
void LCD_HardReset(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
    LCD_DelayMs(50);
    GPIO_SetBits(GPIOC, GPIO_Pin_12);
	LCD_DelayMs(50);
}

#endif

