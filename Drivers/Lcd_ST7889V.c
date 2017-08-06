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

/* 下面3个变量，主要用于使程序同时支持不同的屏 */
uint16_t g_LcdHeight = 240;			/* 显示屏分辨率-高度 */
uint16_t g_LcdWidth = 320;			/* 显示屏分辨率-宽度 */
uint8_t s_ucBright;					/* 背光亮度参数 */
uint8_t g_LcdDirection;				/* 显示方向.0，1，2，3 */


#define LCD_ILI9320_On() GPIO_SetBits(GPIOC,GPIO_Pin_7)
#define LCD_ILI9320_Off() GPIO_ResetBits(GPIOC,GPIO_Pin_7)

#define Bank1_LCD_R    ((uint32_t)0x60000000)    //disp Reg ADDR
#define Bank1_LCD_D    ((uint32_t)0x60020000)	//disp Data ADDR

static u8 gLcdScale=100;

//9320功能寄存器地址
#define WINDOW_XADDR_START	0x0050 // 水平的开始地址组
#define WINDOW_XADDR_END		0x0051 // 水平的结束地址组
#define WINDOW_YADDR_START	0x0052 // 垂直的开始地址组
#define WINDOW_YADDR_END		0x0053 // 垂直的结束地址组
#define GRAM_XADDR		    		0x0020 // GRAM 水平的地址组
#define GRAM_YADDR		    		0x0021 // GRAM 垂直的地址组
#define GRAMWR 			    			0x0022 // GRAM

/**********************************************
函数名：LCD_Delay
功能：用于LCD配置延时
入口参数：延时数
返回值：无
***********************************************/
static void LCD_DelayMs(u32 Ms)
{
  u32 i;
	for(; Ms; Ms--)
		for(i=1000;i;i--);
}

/*************************************************
函数名：LCD_WR_Reg
功能：对lcd寄存器，写命令
入口参数：寄存器地址和命令
返回值：寄存器值
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
函数名：LCD_RD_Reg
功能：对lcd寄存器，读值
入口参数：寄存器地址
返回值：寄存器值
*************************************************/
//static u16 LCD_ReadReg(u16 index)
//{
//	*(__IO uint16_t *) (Bank1_LCD_R)= index;
//	return (*(__IO uint16_t *) (Bank1_LCD_D));
//}

/*************************************************
函数名：LCD_WR_Data
功能：对lcd写数据
入口参数：数据值
返回值：无
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
函数名：LCD_RD_Data
功能：读lcd数据
入口参数：无
返回值：数据
*************************************************/
static void LCD_ReadData(uint8_t *readBuf, uint8_t _uCount)
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

/**********************************************
函数名：FSMC_LCD_Init
功能：用于FSMC配置
入口参数：无
返回值：无
***********************************************/
static void LCD_CtrlLinesConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能 GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	/*  GPIO 配置为复用推挽输出 */
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
    //默认输出高电平
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);

    /* 配置背光GPIO为推挽输出模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_SetDirection
*	功能说明: 设置显示方向。
*	形    参:  _ucDir : 显示方向代码 0 横屏正常, 1=横屏180度翻转, 2=竖屏, 3=竖屏180度翻转
*	返 回 值: 无
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
        D4   ML  Vertical Refresh Order  LCD vertical refresh direction control. 、

        D3   BGR RGB-BGR Order   Color selector switch control
             (0 = RGB color filter panel, 1 = BGR color filter panel )
        D2   MH  Horizontal Refresh ORDER  LCD horizontal refreshing direction control.
        D1   X   Reserved  Reserved
        D0   X   Reserved  Reserved
    */
    g_LcdDirection =  _ucDir;		/* 保存在全局变量 */
    LCD_WriteCmd(0x36);
    /* 0 表示竖屏(排线在下)，1表示竖屏(排线在上), 2表示横屏(排线在左边)  3表示横屏 (排线在右边) */
    if (_ucDir == 0)
    {
        LCD_WriteParam(0xA0);   /* 横屏(排线在左边) */
        g_LcdHeight = 240;
        g_LcdWidth = 320;
    }
    else if (_ucDir == 1)
    {
        LCD_WriteParam(0x60);   /* 横屏 (排线在右边) */
        g_LcdHeight = 240;
        g_LcdWidth = 320;
    }
    else if (_ucDir == 2)
    {
        LCD_WriteParam(0xC0);   /* 竖屏(排线在上) */
        g_LcdHeight = 320;
        g_LcdWidth = 240;
    }
    else if (_ucDir == 3)
    {
        LCD_WriteParam(0x00);   /* 竖屏(排线在下) */
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
函数名：LCD_Power_On
功能：LCD驱动序列
入口参数：无
返回值：无
*************************************************/
static void LCD_PowerOn(void)
{
	LCD_WriteCmd(0x29); //Display on
}

/*************************************************
函数名：LCD_Power_Off
功能：LCD关闭序列
入口参数：无
返回值：无
*************************************************/
static void LCD_PowerOff(void)
{
	LCD_WriteCmd(0x28); //Display on
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
static void LCD_SetDispWin(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth)
{
	LCD_WriteCmd(0X2A); 		/* 设置X坐标 */
	LCD_WriteParam((uint8_t)((_usX & 0xFF00) >> 8));	/* 起始点 */
	LCD_WriteParam((uint8_t)(_usX & 0x00FF));
	LCD_WriteParam((uint8_t)(((_usX + _usWidth - 1) & 0xFF00) >> 8));	/* 结束点 */
	LCD_WriteParam((uint8_t)(_usX + _usWidth - 1));

	LCD_WriteCmd(0X2B); 				  /* 设置Y坐标*/
	LCD_WriteParam((uint8_t)((_usY & 0xFF00) >> 8));   /* 起始点 */
	LCD_WriteParam((uint8_t)(_usY & 0x00FF));
	LCD_WriteParam((uint8_t)(((_usY + _usHeight - 1) & 0xFF00) >>8));		/* 结束点 */
	LCD_WriteParam((uint8_t)(_usY + _usHeight - 1));
}

static void LCD_HardInit()
{
    /* 初始化LCD，写LCD寄存器进行配置 */

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
    /* 设置显示窗口 */
    LCD_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);
#endif

}

/*
*********************************************************************************************************
*	函 数 名: ST7789V_ReadID
*	功能说明: 读取LCD驱动芯片ID， 4个bit
*	形    参:  无
*	返 回 值: 无
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
*	函 数 名: ST7789V_ClrScr
*	功能说明: 根据输入的颜色值清屏
*	形    参:  _usColor : 背景色
*	返 回 值: 无
*********************************************************************************************************
*/
static void LCD_ClrScr(uint16_t _usColor)
{
	uint32_t i;
	uint32_t n;

	LCD_SetDispWin(0, 0, g_LcdHeight, g_LcdWidth);

	LCD_WriteCmd(0x2C); 			/* 准备读写显存 */

#if 1		/* 优化代码执行速度 */
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
函数名：LCD_WR_Data_Start
功能：LCD开始批量传数据前调用
入口参数：无
返回值：无
*************************************************/
void LCD_BlukWriteDataStart(void)
{
    LCD_WriteCmd(0x3C);
}

/*************************************************
函数名：LCD_ReadDataStart
功能：LCD开始批量传数据前调用
入口参数：无
返回值：无
*************************************************/
void LCD_BulkReadDataStart(void)
{
    LCD_WriteCmd(0x3E);
}

/*************************************************
函数名：LCD_BulkWriteData
功能：对lcd批量写数据
入口参数：数据值
返回值：无
*************************************************/
void LCD_BulkWriteData(u16 val)
{
	LCD_WriteParam(val & 0xff);
    LCD_WriteParam((val & 0xff00) >> 8);
}

/*************************************************
函数名：LCD_BulkReadData
功能：批量读lcd数据
入口参数：无
返回值：数据
*************************************************/
u16 LCD_BulkReadData(void)
{
    uint8_t buf[2] = {0x0};
    LCD_ReadData(buf, 2);
    return ((buf[0] << 8) | buf[1]);
}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void LCD_SetXY(u16 _usX,u16 _usY)
{
    LCD_WriteCmd(0X2A); 		/* 设置X坐标 */
	LCD_WriteParam(_usX >> 8);	/* 先高8位，然后低8位 */
	LCD_WriteParam(_usX);		/* 设置起始点和结束点*/
	LCD_WriteParam(_usX >> 8);	/* 先高8位，然后低8位 */
	LCD_WriteParam(_usX);		/* 设置起始点和结束点*/

    LCD_WriteCmd(0X2B); 		/* 设置Y坐标*/
	LCD_WriteParam(_usY >> 8);
	LCD_WriteParam(_usY);
	LCD_WriteParam(_usY >> 8);
	LCD_WriteParam(_usY);
}

/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点,Y_IncMode表示先自增y再自增x
返回值：无
*************************************************/
void LCD_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end,bool yIncFrist)
{
    LCD_WriteCmd(0X2A);         /* 设置X坐标 */
    LCD_WriteParam(x_start >> 8);  /* 先高8位，然后低8位 */
    LCD_WriteParam(x_start);       /* 设置起始点和结束点*/
    LCD_WriteParam(x_end >> 8);  /* 先高8位，然后低8位 */
    LCD_WriteParam(x_end);       /* 设置起始点和结束点*/

    LCD_WriteCmd(0X2B);         /* 设置Y坐标*/
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
函数名：LCD_Set_XY_Addr_Direction
功能：设置lcd读或写自增的方向
入口参数：0:由0到高，1:由高到0
返回值：无
*************************************************/
void LCD_SetAddrIncMode(LCD_INC_MODE xyDirection)
{
	//register u16 ModeReg=LCD_ReadReg(0x0003);

	//ModeReg&=(~0x30);
	//ModeReg|=((xyDirection)<<4);
	//LCD_WriteReg(0x0003, ModeReg);
}

/*************************************************
函数名：LCD_BGR_Mode
功能：设置lcd RGB顺序
入口参数：0:RGB   1:BGR
返回值：无
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
函数名：LCD_Addr_Inc
功能：地址自增
入口参数：无
返回值：无
*************************************************/
void LCD_AddrInc(void)
{
	//register u16 Color16;
	//LCD_ReadData();
	//Color16=LCD_ReadData();
	//LCD_WriteData((((Color16>>11)&0x001f)|(Color16&0x07e0)|((Color16<<11)&0xf800)));//将16位RGB(565)色彩换算成16位BGR(565)色彩
}

/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void LCD_DrawPoint(u16 _usX, u16 _usY, u16 _usColor)
{
    LCD_SetXY(_usX, _usY);	/* 设置光标位置 */

	/* 写显存 */
	LCD_WriteCmd(0x2C);
	LCD_WriteParam(_usColor >> 8);
	LCD_WriteParam(_usColor);
}

/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
u16 LCD_ReadPoint(u16 _usX, u16 _usY)
{
    uint8_t buf[3] = {0x0};

	LCD_SetXY(_usX, _usY);	/* 设置光标位置 */

	LCD_WriteCmd(0x2E);
	LCD_ReadData(buf, 3); 	/* 第1个哑读，丢弃 */

    return ((buf[1] << 8) | buf[2]);
}

/*************************************************
函数名：LCD_Light_Set
功能：LCD设置背光亮度
入口参数：Scale:0-100，0为熄灭，100最亮
返回值：无
*************************************************/
void LCD_Light_Set(u8 Scale)
{
	Debug("LCD Light Set:%d\n\r",Scale);
#if LCD_BGLIGHT_PWM_MODE
    /* STM32-V4开发板，PB1/TIM3_CH4/TIM8_CH3N 控制背光PWM ； 因为 TIM3用于红外解码。因此用TIM8_CH3N做背光PWM */
	//bsp_SetTIMOutPWM(GPIOB, GPIO_Pin_1, TIM3, 4, 100, (_bright * 10000) /255);	// TIM3_CH4
	bsp_SetTIMOutPWM_N(GPIOA, GPIO_Pin_2, TIM2, 3, 2000, Scale);	// TIM2_CH3N
#else
	if(Scale) GPIO_SetBits(GPIOA, GPIO_Pin_2);
	else GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#endif
	gLcdScale=Scale;
}

/*************************************************
函数名：LCD_Light_State
功能:查询当前背光亮度
入口参数：无
返回值：0-100，0为熄灭，100最亮
*************************************************/
u8 LCD_Light_State(void)
{
	return gLcdScale;
}

/*************************************************
函数名：LCD_Init
功能：初始化启动lcd
入口参数：无
返回值：无
*************************************************/
void LCD_Init(void)
{
	LCD_CtrlLinesConfig();
	LCD_DelayMs(100);
	LCD_HardReset();

    if(0x858552 == LCD_ReadID()) {
        LCD_HardInit();
        LCD_SetDirection(2);
        LCD_ClrScr(CL_BLACK);   /* 清屏，显示全黑 */
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
函数名：LCD_Reset
功能：LCD复位
入口参数：延时数
返回值：无
***********************************************/
void LCD_HardReset(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
    LCD_DelayMs(50);
    GPIO_SetBits(GPIOC, GPIO_Pin_12);
	LCD_DelayMs(50);
}

#endif

