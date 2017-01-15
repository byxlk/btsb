/*
*********************************************************************************************************
*
*	模块名称 : TFT液晶显示器驱动模块
*	文件名称 : LCD_ST7789V.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/


#ifndef _BSP_LCD_ST7789V_H
#define _BSP_LCD_ST7789V_H

#define IC_ST7789V_ID 0x858552

/* 可供外部模块调用的函数 */
uint32_t ST7789V_ReadID(void);
void ST7789V_InitHard(void);
void ST7789V_DispOn(void);
void ST7789V_DispOff(void);
void ST7789V_ClrScr(uint16_t _usColor);
void ST7789V_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor);
uint16_t ST7789V_GetPixel(uint16_t _usX, uint16_t _usY);
void ST7789V_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor);
void ST7789V_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor);
void ST7789V_DrawHColorLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ST7789V_DrawHTransLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ST7789V_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ST7789V_FillRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ST7789V_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor);
void ST7789V_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr);

void ST7789V_SetBackLight(uint8_t _bright);
void ST7789V_SetDirection(uint8_t _ucDir);

void ST7789V_DrawVLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usY2 , uint16_t _usColor);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
