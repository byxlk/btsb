#ifndef __LCD_H__
#define __LCD_H__

#include "Drivers.h"
#include "stm32f2xx.h"

#define LCD_BGLIGHT_PWM_MODE 1 //����pwm���Ʊ���

#if QXW_LCM_ID == 212 || QXW_LCM_ID == 211 || QXW_LCM_ID == 210

typedef enum{
	xInc_yInc=0,
	xInc_yDec,
	xDec_yInc,
	xDec_yDec
}LCD_INC_MODE;//LCD ��������ģʽ������ݲ�ͬLCD������������Ķ���λ��

#elif QXW_LCM_ID == 220 || QXW_LCM_ID == 221

typedef enum{
	xDec_yDec=0,
	xInc_yDec,
	xDec_yInc,
	xInc_yInc
}LCD_INC_MODE;

#endif

/*
	LCD ��ɫ���룬CL_��Color�ļ�д
	16Bit�ɸ�λ����λ�� RRRR RGGG GGGB BBBB

	�����RGB �꽫24λ��RGBֵת��Ϊ16λ��ʽ��
	����windows�Ļ��ʳ��򣬵���༭��ɫ��ѡ���Զ�����ɫ�����Ի�õ�RGBֵ��

	�Ƽ�ʹ������ȡɫ���������㿴���Ľ�����ɫ��
*/
#define RGB(R,G,B)	(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3))	/* ��8λR,G,Bת��Ϊ 16λRGB565��ʽ */
#define RGB565_R(x)  ((x >> 8) & 0xF8)
#define RGB565_G(x)  ((x >> 3) & 0xFC)
#define RGB565_B(x)  ((x << 3) & 0xF8)
enum
{
	CL_WHITE        = RGB(255,255,255),	/* ��ɫ */
	CL_BLACK        = RGB(  0,  0,  0),	/* ��ɫ */
	CL_RED          = RGB(255,	0,  0),	/* ��ɫ */
	CL_GREEN        = RGB(  0,255,  0),	/* ��ɫ */
	CL_BLUE         = RGB(  0,	0,255),	/* ��ɫ */
	CL_YELLOW       = RGB(255,255,  0),	/* ��ɫ */

	CL_GREY			= RGB( 98, 98, 98), 	/* ���ɫ */
	CL_GREY1		= RGB( 150, 150, 150), 	/* ǳ��ɫ */
	CL_GREY2		= RGB( 180, 180, 180), 	/* ǳ��ɫ */
	CL_GREY3		= RGB( 200, 200, 200), 	/* ��ǳ��ɫ */
	CL_GREY4		= RGB( 230, 230, 230), 	/* ��ǳ��ɫ */

	CL_BUTTON_GREY	= RGB( 220, 220, 220), /* WINDOWS ��ť�����ɫ */

	CL_MAGENTA      = 0xF81F,	/* ����ɫ�����ɫ */
	CL_CYAN         = 0x7FFF,	/* ����ɫ����ɫ */

	CL_BLUE1        = RGB(  0,  0, 240),		/* ����ɫ */
	CL_BLUE2        = RGB(  0,  0, 128),		/* ����ɫ */
	CL_BLUE3        = RGB(  68, 68, 255),		/* ǳ��ɫ1 */
	CL_BLUE4        = RGB(  0, 64, 128),		/* ǳ��ɫ1 */

	/* UI ���� Windows�ؼ�����ɫ */
	CL_BTN_FACE		= RGB(236, 233, 216),	/* ��ť������ɫ(��) */

	CL_BTN_FONT		= CL_BLACK,				/* ��ť������ɫ���ڣ� */

	CL_BOX_BORDER1	= RGB(172, 168,153),	/* �����������ɫ */
	CL_BOX_BORDER2	= RGB(255, 255,255),	/* �������Ӱ����ɫ */


	CL_MASK			= 0x9999	/* ��ɫ���룬�������ֱ���͸�� */
};


void LCD_BulkReadDataStart(void);//������ȡ����ǰ��Ҫ��������
u16 LCD_BulkReadData(void);//������ȡҺ����ɫ������
void LCD_BlukWriteDataStart(void);//����дҺ����֮ǰҪ��������
void LCD_BulkWriteData(u16 val);//������ȡҺ����ɫ������

void LCD_SetXY(u16 x,u16 y);//��������
void LCD_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end,bool yIncFrist);//���û滭��Χ��yIncFrist��ʾ������Y�ټ�X
void LCD_SetAddrIncMode(LCD_INC_MODE xyDirection);//����xy��������
void LCD_BgrMode(bool UseBGR);//��������BGR����RGB
void LCD_AddrInc(void);//��ַ����1

void LCD_DrawPoint(u16 x,u16 y,u16 Data);//��ָ����
u16 LCD_ReadPoint(u16 x,u16 y);//��ָ����

void LCD_Light_Set(u8 Scale);//���ñ�������
u8 LCD_Light_State(void);//��ȡ��������

void LCD_HardReset(void);//Ӳ����λ
void LCD_Init(void);//��ʼ��
void LCD_DeInit(void);//ע��

#endif

