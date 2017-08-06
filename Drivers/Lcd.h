#ifndef __LCD_H__
#define __LCD_H__

#include "Drivers.h"
#include "stm32f2xx.h"

#define LCD_BGLIGHT_PWM_MODE 1 //启用pwm控制背光

#if QXW_LCM_ID == 212 || QXW_LCM_ID == 211 || QXW_LCM_ID == 210

typedef enum{
	xInc_yInc=0,
	xInc_yDec,
	xDec_yInc,
	xDec_yDec
}LCD_INC_MODE;//LCD 坐标自增模式，需根据不同LCD调节四种情况的定义位置

#elif QXW_LCM_ID == 220 || QXW_LCM_ID == 221

typedef enum{
	xDec_yDec=0,
	xInc_yDec,
	xDec_yInc,
	xInc_yInc
}LCD_INC_MODE;

#endif

/*
	LCD 颜色代码，CL_是Color的简写
	16Bit由高位至低位， RRRR RGGG GGGB BBBB

	下面的RGB 宏将24位的RGB值转换为16位格式。
	启动windows的画笔程序，点击编辑颜色，选择自定义颜色，可以获得的RGB值。

	推荐使用迷你取色器软件获得你看到的界面颜色。
*/
#define RGB(R,G,B)	(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3))	/* 将8位R,G,B转化为 16位RGB565格式 */
#define RGB565_R(x)  ((x >> 8) & 0xF8)
#define RGB565_G(x)  ((x >> 3) & 0xFC)
#define RGB565_B(x)  ((x << 3) & 0xF8)
enum
{
	CL_WHITE        = RGB(255,255,255),	/* 白色 */
	CL_BLACK        = RGB(  0,  0,  0),	/* 黑色 */
	CL_RED          = RGB(255,	0,  0),	/* 红色 */
	CL_GREEN        = RGB(  0,255,  0),	/* 绿色 */
	CL_BLUE         = RGB(  0,	0,255),	/* 蓝色 */
	CL_YELLOW       = RGB(255,255,  0),	/* 黄色 */

	CL_GREY			= RGB( 98, 98, 98), 	/* 深灰色 */
	CL_GREY1		= RGB( 150, 150, 150), 	/* 浅灰色 */
	CL_GREY2		= RGB( 180, 180, 180), 	/* 浅灰色 */
	CL_GREY3		= RGB( 200, 200, 200), 	/* 最浅灰色 */
	CL_GREY4		= RGB( 230, 230, 230), 	/* 最浅灰色 */

	CL_BUTTON_GREY	= RGB( 220, 220, 220), /* WINDOWS 按钮表面灰色 */

	CL_MAGENTA      = 0xF81F,	/* 红紫色，洋红色 */
	CL_CYAN         = 0x7FFF,	/* 蓝绿色，青色 */

	CL_BLUE1        = RGB(  0,  0, 240),		/* 深蓝色 */
	CL_BLUE2        = RGB(  0,  0, 128),		/* 深蓝色 */
	CL_BLUE3        = RGB(  68, 68, 255),		/* 浅蓝色1 */
	CL_BLUE4        = RGB(  0, 64, 128),		/* 浅蓝色1 */

	/* UI 界面 Windows控件常用色 */
	CL_BTN_FACE		= RGB(236, 233, 216),	/* 按钮表面颜色(灰) */

	CL_BTN_FONT		= CL_BLACK,				/* 按钮字体颜色（黑） */

	CL_BOX_BORDER1	= RGB(172, 168,153),	/* 分组框主线颜色 */
	CL_BOX_BORDER2	= RGB(255, 255,255),	/* 分组框阴影线颜色 */


	CL_MASK			= 0x9999	/* 颜色掩码，用于文字背景透明 */
};


void LCD_BulkReadDataStart(void);//批量读取数据前需要做的事情
u16 LCD_BulkReadData(void);//批量读取液晶屏色彩数据
void LCD_BlukWriteDataStart(void);//批量写液晶屏之前要做的事情
void LCD_BulkWriteData(u16 val);//批量读取液晶屏色彩数据

void LCD_SetXY(u16 x,u16 y);//设置坐标
void LCD_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end,bool yIncFrist);//设置绘画范围，yIncFrist表示先增加Y再加X
void LCD_SetAddrIncMode(LCD_INC_MODE xyDirection);//设置xy自增方向
void LCD_BgrMode(bool UseBGR);//设置是用BGR或者RGB
void LCD_AddrInc(void);//地址自增1

void LCD_DrawPoint(u16 x,u16 y,u16 Data);//画指定点
u16 LCD_ReadPoint(u16 x,u16 y);//读指定点

void LCD_Light_Set(u8 Scale);//设置背光亮度
u8 LCD_Light_State(void);//读取背光亮度

void LCD_HardReset(void);//硬件复位
void LCD_Init(void);//初始化
void LCD_DeInit(void);//注销

#endif

