/*
*********************************************************************************************************
*
*	模块名称 : TFT液晶显示器驱动模块
*	文件名称 : LCD_ST7789V.c
*	版    本 : V1.0
*	说    明 : ST7789V 显示器分辨率为 480 * 320,  3.5寸普通比例4：3
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2014-07-26 armfly  首版
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
//#include "fonts.h"

#define ST7789V_BASE       ((uint32_t)(0x6C000000 | 0x00000000))

#define ST7789V_REG		*(__IO uint16_t *)(ST7789V_BASE)
#define ST7789V_RAM		*(__IO uint16_t *)(ST7789V_BASE + (1 << (0 + 1)))	/* FSMC 16位总线模式下，FSMC_A0口线对应物理地址A1 */

static __IO uint8_t s_RGBChgEn = 0;		/* RGB转换使能, 4001屏写显存后读会的RGB格式和写入的不同 */

static void Init_7789(void);
static void ST7789V_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth);
static void ST7789V_QuitWinMode(void);
static void ST7789V_SetCursor(uint16_t _usX, uint16_t _usY);
static void ST7789V_ReadData(uint8_t *readBuf, uint8_t _uCount);
static void ST7789V_WriteCmd(uint8_t _ucCmd);
static void ST7789V_WriteParam(uint8_t _ucParam);

/*
*********************************************************************************************************
*	函 数 名: ST7789V_InitHard
*	功能说明: 初始化LCD
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
uint32_t ST7789V_InitHard(void)
{
	uint32_t id;

	id = ST7789V_ReadID();

	if (id == IC_ST7789V_ID)
	{
		Init_7789();	/* 初始化5420和4001屏硬件 */

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
*	函 数 名: ST7789V_SetDirection
*	功能说明: 设置显示方向。
*	形    参:  _ucDir : 显示方向代码 0 横屏正常, 1=横屏180度翻转, 2=竖屏, 3=竖屏180度翻转
*	返 回 值: 无
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
		D4   ML  Vertical Refresh Order  LCD vertical refresh direction control. 、

		D3   BGR RGB-BGR Order   Color selector switch control
		     (0 = RGB color filter panel, 1 = BGR color filter panel )
		D2   MH  Horizontal Refresh ORDER  LCD horizontal refreshing direction control.
		D1   X   Reserved  Reserved
		D0   X   Reserved  Reserved
	*/
	ST7789V_WriteCmd(0x36);
	/* 0 表示竖屏(排线在下)，1表示竖屏(排线在上), 2表示横屏(排线在左边)  3表示横屏 (排线在右边) */
	if (_ucDir == 0)
	{
		ST7789V_WriteParam(0xA0);	/* 横屏(排线在左边) */
		g_LcdHeight = 240;
		g_LcdWidth = 320;
	}
	else if (_ucDir == 1)
	{
		ST7789V_WriteParam(0x60);	/* 横屏 (排线在右边) */
		g_LcdHeight = 240;
		g_LcdWidth = 320;
	}
	else if (_ucDir == 2)
	{
		ST7789V_WriteParam(0xC0);	/* 竖屏(排线在上) */
		g_LcdHeight = 320;
		g_LcdWidth = 240;
	}
	else if (_ucDir == 3)
	{
		ST7789V_WriteParam(0x00);	/* 竖屏(排线在下) */
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
*	函 数 名: Init_7789
*	功能说明: 初始化ST7789V驱动器
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void Init_7789(void)
{
	/* 初始化LCD，写LCD寄存器进行配置 */

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
	/* 设置显示窗口 */
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
*	函 数 名: ST7789V_ReadData()
*	功能说明: 从LCD控制器芯片读取数据
*	形    参: 无
*	返 回 值: 数据
*********************************************************************************************************
*/
static void ST7789V_ReadData(uint8_t *readBuf, uint8_t _uCount)
{
        //CS(CS)=PC9 RS(D/CX)=PC8 WR(WRX)=PC7 RD(RDX)=PC6 RST=PA8

        uint8_t i =0x00;   
        GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能 GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/*  GPIO 配置为复用推挽输出 */
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
*	函 数 名: ST7789V_WriteCmd
*	功能说明: 向LCD控制器芯片发送命令
*	形    参: _ucCmd : 命令代码
*	返 回 值: 无
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
*	函 数 名: ST7789V_WriteParam
*	功能说明: 向LCD控制器芯片发送参数(data)
*	形    参: _ucParam : 参数数据
*	返 回 值: 无
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
*	函 数 名: ST7789V_SetDispWin
*	功能说明: 设置显示窗口，进入窗口显示模式。TFT驱动芯片支持窗口显示模式，这是和一般的12864点阵LCD的最大区别。
*	形    参:
*		_usX : 水平坐标
*		_usY : 垂直坐标
*		_usHeight: 窗口高度
*		_usWidth : 窗口宽度
*	返 回 值: 无
*********************************************************************************************************
*/
static void ST7789V_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth)
{
	ST7789V_WriteCmd(0X2A); 		/* 设置X坐标 */
	ST7789V_WriteParam((uint8_t)((_usX & 0xFF00) >> 8));	/* 起始点 */
	ST7789V_WriteParam((uint8_t)(_usX & 0x00FF));
	ST7789V_WriteParam((uint8_t)(((_usX + _usWidth - 1) & 0xFF00) >> 8));	/* 结束点 */
	ST7789V_WriteParam((uint8_t)(_usX + _usWidth - 1));

	ST7789V_WriteCmd(0X2B); 				  /* 设置Y坐标*/
	ST7789V_WriteParam((uint8_t)((_usY & 0xFF00) >> 8));   /* 起始点 */
	ST7789V_WriteParam((uint8_t)(_usY & 0x00FF));
	ST7789V_WriteParam((uint8_t)(((_usY + _usHeight - 1) & 0xFF00) >>8));		/* 结束点 */
	ST7789V_WriteParam((uint8_t)(_usY + _usHeight - 1));
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_SetCursor
*	功能说明: 设置光标位置
*	形    参:  _usX : X坐标; _usY: Y坐标
*	返 回 值: 无
*********************************************************************************************************
*/
static void ST7789V_SetCursor(uint16_t _usX, uint16_t _usY)
{
	ST7789V_WriteCmd(0X2A); 		/* 设置X坐标 */
	ST7789V_WriteParam(_usX >> 8);	/* 先高8位，然后低8位 */
	ST7789V_WriteParam(_usX);		/* 设置起始点和结束点*/
	ST7789V_WriteParam(_usX >> 8);	/* 先高8位，然后低8位 */
	ST7789V_WriteParam(_usX);		/* 设置起始点和结束点*/

    ST7789V_WriteCmd(0X2B); 		/* 设置Y坐标*/
	ST7789V_WriteParam(_usY >> 8);
	ST7789V_WriteParam(_usY);
	ST7789V_WriteParam(_usY >> 8);
	ST7789V_WriteParam(_usY);
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_QuitWinMode
*	功能说明: 退出窗口显示模式，变为全屏显示模式
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ST7789V_QuitWinMode(void)
{
	ST7789V_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_ReadID
*	功能说明: 读取LCD驱动芯片ID， 4个bit
*	形    参:  无
*	返 回 值: 无
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
*	函 数 名: ST7789V_DispOn
*	功能说明: 打开显示
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DispOn(void)
{
    ST7789V_WriteCmd(0x29); //Display on
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_DispOff
*	功能说明: 关闭显示
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DispOff(void)
{
    ST7789V_WriteCmd(0x28); //Display on
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_ClrScr
*	功能说明: 根据输入的颜色值清屏
*	形    参:  _usColor : 背景色
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_ClrScr(uint16_t _usColor)
{
	uint32_t i;
	uint32_t n;

	ST7789V_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);

	ST7789V_WriteCmd(0x2C); 			/* 准备读写显存 */

#if 1		/* 优化代码执行速度 */
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
*	函 数 名: ST7789V_PutPixel
*	功能说明: 画1个像素
*	形    参:
*			_usX,_usY : 像素坐标
*			_usColor  ：像素颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor)
{
	ST7789V_SetCursor(_usX, _usY);	/* 设置光标位置 */

	/* 写显存 */
	ST7789V_WriteCmd(0x2C);
	ST7789V_WriteParam(_usColor >> 8);
	ST7789V_WriteParam(_usColor);
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_GetPixel
*	功能说明: 读取1个像素
*	形    参:
*			_usX,_usY : 像素坐标
*			_usColor  ：像素颜色
*	返 回 值: RGB颜色值 （565）
*********************************************************************************************************
*/
uint16_t ST7789V_GetPixel(uint16_t _usX, uint16_t _usY)
{
	uint8_t buf[3] = {0x0};

	ST7789V_SetCursor(_usX, _usY);	/* 设置光标位置 */

	ST7789V_WriteCmd(0x2E);
	ST7789V_ReadData(buf, 3); 	/* 第1个哑读，丢弃 */

    return ((buf[1] << 8) | buf[2]);
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_DrawLine
*	功能说明: 采用 Bresenham 算法，在2点间画一条直线。
*	形    参:
*			_usX1, _usY1 ：起始点坐标
*			_usX2, _usY2 ：终止点Y坐标
*			_usColor     ：颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor)
{
	int32_t dx , dy ;
	int32_t tx , ty ;
	int32_t inc1 , inc2 ;
	int32_t d , iTag ;
	int32_t x , y ;

	/* 采用 Bresenham 算法，在2点间画一条直线 */

	ST7789V_PutPixel(_usX1 , _usY1 , _usColor);

	/* 如果两点重合，结束后面的动作。*/
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

	if ( dx < dy )   /*如果dy为计长方向，则交换纵横坐标。*/
	{
		uint16_t temp;

		iTag = 1 ;
		temp = _usX1; _usX1 = _usY1; _usY1 = temp;
		temp = _usX2; _usX2 = _usY2; _usY2 = temp;
		temp = dx; dx = dy; dy = temp;
	}
	tx = _usX2 > _usX1 ? 1 : -1 ;    /* 确定是增1还是减1 */
	ty = _usY2 > _usY1 ? 1 : -1 ;
	x = _usX1 ;
	y = _usY1 ;
	inc1 = 2 * dy ;
	inc2 = 2 * ( dy - dx );
	d = inc1 - dx ;
	while ( x != _usX2 )     /* 循环画点 */
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
*	函 数 名: ST7789V_DrawHLine
*	功能说明: 绘制一条水平线 （主要用于UCGUI的接口函数）
*	形    参:  _usX1    ：起始点X坐标
*			  _usY1    ：水平线的Y坐标
*			  _usX2    ：结束点X坐标
*			  _usColor : 颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor)
{
	uint16_t i;

	ST7789V_SetDispWin(_usX1, _usY1, 1, _usX2 - _usX1 + 1);

	ST7789V_WriteCmd(0x2C);

	/* 写显存 */
	for (i = 0; i <_usX2-_usX1 + 1; i++)
	{
		ST7789V_WriteParam(_usColor >> 8);
		ST7789V_WriteParam(_usColor);
	}
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_DrawVLine
*	功能说明: 画垂直平线 用UCGUI的接口函数
*	形    参: X,Y的坐标和颜色
*	返 回 值: 无
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
*	函 数 名: ST7789V_DrawHColorLine
*	功能说明: 绘制一条彩色水平线 （主要用于UCGUI的接口函数）
*	形    参:  _usX1    ：起始点X坐标
*			  _usY1    ：水平线的Y坐标
*			  _usWidth ：直线的宽度
*			  _pColor : 颜色缓冲区
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawHColorLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor)
{
	uint16_t i, colorValue ;
	
	ST7789V_SetDispWin(_usX1, _usY1, 1, _usWidth);

	ST7789V_WriteCmd(0x2C);

	/* 写显存 */
	for (i = 0; i <_usWidth; i++)
	{
	        colorValue = *_pColor++;
		ST7789V_WriteParam(colorValue>> 8);
		ST7789V_WriteParam(colorValue);
	}
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_DrawHTransLine
*	功能说明: 绘制一条彩色透明的水平线 （主要用于UCGUI的接口函数）， 颜色值为0表示透明色
*	形    参:  _usX1    ：起始点X坐标
*			  _usY1    ：水平线的Y坐标
*			  _usWidth ：直线的宽度
*			  _pColor : 颜色缓冲区
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawHTransLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor)
{
	uint16_t i, j;
	uint16_t Index;

	ST7789V_SetCursor(_usX1, _usY1);

	/* 写显存 */
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
*	函 数 名: ST7789V_DrawRect
*	功能说明: 绘制水平放置的矩形。
*	形    参:
*			_usX,_usY：矩形左上角的坐标
*			_usHeight ：矩形的高度
*			_usWidth  ：矩形的宽度
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	/*
	 ---------------->---
	|(_usX，_usY)        |
	V                    V  _usHeight
	|                    |
	 ---------------->---
		  _usWidth
	*/

	ST7789V_DrawLine(_usX, _usY, _usX + _usWidth - 1, _usY, _usColor);	/* 顶 */
	ST7789V_DrawLine(_usX, _usY + _usHeight - 1, _usX + _usWidth - 1, _usY + _usHeight - 1, _usColor);	/* 底 */

	ST7789V_DrawLine(_usX, _usY, _usX, _usY + _usHeight - 1, _usColor);	/* 左 */
	ST7789V_DrawLine(_usX + _usWidth - 1, _usY, _usX + _usWidth - 1, _usY + _usHeight, _usColor);	/* 右 */
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_FillRect
*	功能说明: 填充矩形。
*	形    参:
*			_usX,_usY：矩形左上角的坐标
*			_usHeight ：矩形的高度
*			_usWidth  ：矩形的宽度
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_FillRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	uint32_t i;

	/*
	 ---------------->---
	|(_usX，_usY)        |
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
*	函 数 名: ST7789V_DrawCircle
*	功能说明: 绘制一个圆，笔宽为1个像素
*	形    参:
*			_usX,_usY  ：圆心的坐标
*			_usRadius  ：圆的半径
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor)
{
	int32_t  D;			/* Decision Variable */
	uint32_t  CurX;		/* 当前 X 值 */
	uint32_t  CurY;		/* 当前 Y 值 */

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
*	函 数 名: ST7789V_DrawBMP
*	功能说明: 在LCD上显示一个BMP位图，位图点阵扫描次序：从左到右，从上到下
*	形    参:
*			_usX, _usY : 图片的坐标
*			_usHeight  ：图片高度
*			_usWidth   ：图片宽度
*			_ptr       ：图片点阵指针
*	返 回 值: 无
*********************************************************************************************************
*/
void ST7789V_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr)
{
	uint32_t index = 0;
	const uint16_t *p;

	/* 设置图片的位置和大小， 即设置显示窗口 */
	ST7789V_SetDispWin(_usX, _usY, _usHeight, _usWidth);

	p = _ptr;
	for (index = 0; index < _usHeight * _usWidth; index++)
	{
		ST7789V_PutPixel(_usX, _usY, *p++);
	}

	/* 退出窗口绘图模式 */
	ST7789V_QuitWinMode();
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
