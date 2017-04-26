/*
*********************************************************************************************************
*
*	模块名称 : ATM机主界面
*	文件名称 : MainTask.c
*	版    本 : V1.0
*	说    明 : 实验内容如下
*              1. 所有界面支持按键操作
*                 （1）K2按键用于控件焦点的切换，切换时，所选择的控件会出现浅色边框
*                 （2）K3按键用于进入下一个界面
*                 （3）摇杆的OK键用于选择相应控件后，触发控件所对应的的操作。
*              2. 采用触摸方式操作界面时，如果进入到没有按钮控件时，可以点击屏幕中间部分进入下一页。
*
*	修改记录 :
*		版本号   日期         作者          说明
*		V1.0    2016-11-26   Eric2013  	    首版
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "MainTask.h"
#include "includes.h"



/*
*********************************************************************************************************
*                                         extern
*********************************************************************************************************
*/
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_AmericanExpress;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_ECCard;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_GeldKarte;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_Maestro;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_MasterCard;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_VisaCard;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armfly;
extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armflySmall;

extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;

/*
*********************************************************************************************************
*                                       宏定义
*********************************************************************************************************
*/
#define BUTTON_CLICK_SPEED        600
#define BUTTON_CLICK_DELAY        750
#define DEFAULT_WIDGET_EFFECT     (&WIDGET_Effect_3D2L)

#define MAIN_BKCOLOR              0xD0D0D0
#define MAIN_TEXTCOLOR            0x000000
#define MAIN_FONT                 (&GUI_FontYahei)
#define MAIN_BORDER               0
#define MAIN_TITLE_HEIGHT         0
#define MAIN_LOGO_BITMAP          (&bmLogo_armflySmall)
#define MAIN_LOGO_OFFSET_X        0
#define MAIN_LOGO_OFFSET_Y        0

#define FRAME_BKCOLOR             0xB0B0B0
#define FRAME_TEXTCOLOR           0x000000
#define FRAME_FONT                (&GUI_FontYahei)
#define FRAME_EFFECT              (&WIDGET_Effect_3D1L)
#define FRAME_BORDER              FRAME_EFFECT->EffectSize
#define FRAME_WIDTH               (LCD_GetXSize() - (FRAME_BORDER * 2) - (MAIN_BORDER * 2))
#define FRAME_HEIGHT              (LCD_GetYSize() - (FRAME_BORDER * 2) - (MAIN_BORDER + MAIN_TITLE_HEIGHT))

#define FRAME_BUTTON_BKCOLOR0     0xB8B8B8              /* Unpressed */
#define FRAME_BUTTON_BKCOLOR1     0xE0E0E0              /* Pressed   */
#define FRAME_BUTTON_BKCOLOR2     0xC0C0C0              /* Disabled  */
#define FRAME_BUTTON_COLOR0       0x000000              /* Unpressed */
#define FRAME_BUTTON_COLOR1       0x000000              /* Pressed   */
#define FRAME_BUTTON_FONT         (&GUI_FontYahei)
#define FRAME_BUTTON_EFFECT       (&WIDGET_Effect_3D2L)

#define LOGO_FRAME_OFFSET_Y       5
#define LOGO_FRAME_SIZE_X         116
#define LOGO_FRAME_SIZE_Y         92
#define LOGO_FRAME_DIST_X         4
#define LOGO_FRAME_BKCOLOR        0xFFFFFF
#define LOGO_FRAME_EFFECT         (&WIDGET_Effect_3D2L)

#define NUMPAD_BKCOLOR            GUI_LIGHTGRAY
#define NUMPAD_EFFECT             (&WIDGET_Effect_3D2L)
#define NUMPAD_BORDER             9
#define NUMPAD_PIN                1685

#define NUMPAD_BUTTON_BKCOLOR0    0xB8B8B8              /* Unpressed */
#define NUMPAD_BUTTON_BKCOLOR1    0xE0E0E0              /* Pressed   */
#define NUMPAD_BUTTON_BKCOLOR2    0xC0C0C0              /* Disabled  */
#define NUMPAD_BUTTON_COLOR0      0x700000              /* Unpressed */
#define NUMPAD_BUTTON_COLOR1      0x700000              /* Pressed   */
#define NUMPAD_BUTTON_RED0        0x0060FF              /* Unpressed */
#define NUMPAD_BUTTON_RED1        0x0030E8              /* Pressed   */
#define NUMPAD_BUTTON_BLUE0       0x00FF00              /* Unpressed */
#define NUMPAD_BUTTON_BLUE1       0x00FF7F              /* Pressed   */
#define NUMPAD_BUTTON_GREEN0      0x00B040              /* Unpressed */
#define NUMPAD_BUTTON_GREEN1      0x00D000              /* Pressed   */
#define NUMPAD_BUTTON_FONT        (&GUI_FontYahei)
#define NUMPAD_BUTTON_EFFECT      (&WIDGET_Effect_3D1L)
#define NUMPAD_BUTTON_WIDTH       40
#define NUMPAD_BUTTON_HEIGHT      32
#define NUMPAD_BUTTON_DIST_X      7
#define NUMPAD_BUTTON_DIST_Y      7

/*
*********************************************************************************************************
*                                       宏定义-常数
*********************************************************************************************************
*/
#define NUMPAD_ID_0         (GUI_ID_USER +  0)
#define NUMPAD_ID_1         (GUI_ID_USER +  1)
#define NUMPAD_ID_2         (GUI_ID_USER +  2)
#define NUMPAD_ID_3         (GUI_ID_USER +  3)
#define NUMPAD_ID_4         (GUI_ID_USER +  4)
#define NUMPAD_ID_5         (GUI_ID_USER +  5)
#define NUMPAD_ID_6         (GUI_ID_USER +  6)
#define NUMPAD_ID_7         (GUI_ID_USER +  7)
#define NUMPAD_ID_8         (GUI_ID_USER +  8)
#define NUMPAD_ID_9         (GUI_ID_USER +  9)
#define NUMPAD_ID_X         (GUI_ID_USER + 10)
#define NUMPAD_ID_Y         (GUI_ID_USER + 11)
#define NUMPAD_ID_A         (GUI_ID_USER + 12)
#define NUMPAD_ID_B         (GUI_ID_USER + 13)
#define NUMPAD_ID_C         (GUI_ID_USER + 14)
#define NUMPAD_ID_D         (GUI_ID_USER + 15)

#define MSG_CARD_INSERTED   (WM_USER + 0)
#define MSG_CARD_REMOVED    (WM_USER + 1)
#define MSG_MONEY_REMOVED   (WM_USER + 2)
#define MSG_PIN_CHANGED     (WM_USER + 3)
#define MSG_PIN_OK          (WM_USER + 4)
#define MSG_PIN_CANCEL      (WM_USER + 5)
#define MSG_PIN_ERROR       (WM_USER + 6)

#define LANG_GER 0
#define LANG_ENG 1

#define TEXT_ID_ABBRUCH             1
#define TEXT_ID_KORREKTUR           2
#define TEXT_ID_BESTAETIGEN         3
#define TEXT_ID_PIN_EINGEBEN        4
#define TEXT_ID_KARTE_EINFUEHREN    5
#define TEXT_ID_GELD_ABHEBEN        6
#define TEXT_ID_KONTOSTAND_ANSEHEN  7
#define TEXT_ID_AUSWAHL_TREFFEN     8
#define TEXT_ID_ANDERER_BETRAG      9
#define TEXT_ID_BITTE_GEDULD        10
#define TEXT_ID_KARTE_ENTNEHMEN     11
#define TEXT_ID_GELD_ENTNEHMEN      12
#define TEXT_ID_VIELEN_DANK         13
#define TEXT_ID_GELDAUTOMAT         14
#define TEXT_ID_KONTOSTAND          15
#define TEXT_ID_AUSWAHL_BETRAG      16
#define TEXT_ID_FALSCHE_PIN         17
#define TEXT_ID_BETRAG_EINGEBEN     18

/*
*********************************************************************************************************
*                                       结构体
*********************************************************************************************************
*/
typedef struct {
  int x;
  int y;
  int Pressed;
  int Duration;
} PID_EVENT;

/*
*********************************************************************************************************
*                                       静态数据
*********************************************************************************************************
*/
static const char * _aLang[][2] = {
  { "取消",
    "Cancel"
  },

  { "更正",
    "Correction"
  },

  { "确认",
    "Confirm"
  },

  { "请输入密码：",
    "Please enter your PIN:"
  },

  { "请插入银行卡",
    "Please insert your card"
  },

  { "请取款",
    "Draw money"
  },

  { "查看余额",
    "View account balance"
  },

  { "请选择",
    "Please select..."
  },
  { "其它金额",
    "Different amount"
  },

  { "处理中，请不要离开",
    "Please wait until your transaction\nis being processed."
  },

  { "请取走您的银行卡",
    "Please you take your card"
  },

  { "请取款",
    "Transaction completed.\nPlease take your money"
  },

  { "谢谢使用",
    "Thank you for\nusing"
  },

  { "自动取款机",
    "ATM"
  },

  { "您当前的余额是：-1234.56 RMB",
    "Your balance amount is: -1234.56 RMB"
  },

  { "请选择金额",
    "Please select amount"
  },

  { "密码错误！至少是4位",
    "Wrong PIN. At least 4 digits"
  },

  { "请输入金额",
    "Please enter amount"
  },
};

static WM_HWIN    _hLastFrame;
static WM_HWIN    _hNumPad;

static unsigned   _PIN_Value;
static unsigned   _PIN_Digits;

static unsigned   _Language;
static WM_HWIN    _hTitle;

static void _cbDrawHomePage(WM_MESSAGE * pMsg);

/*
*********************************************************************************************************
*	函 数 名: _GetLang
*	功能说明: 获取要显示字符串的地址
*	形    参: Index  要显示字符串的序号
*	返 回 值: p      返回要显示字符串地址
*********************************************************************************************************
*/
static const char * _GetLang(unsigned Index)
{
	const char * p;

	p = NULL;
	Index--;

	if ((Index < GUI_COUNTOF(_aLang)) && (_Language < GUI_COUNTOF(_aLang[0])))
	{
		p = _aLang[Index][_Language];
	}

	return p;
}

/*
*********************************************************************************************************
*	函 数 名: _DrawDownRectEx
*	功能说明: 显示凹陷的矩形框
*	形    参：pEffect  控件显示效果
*             pRect    GUI_RECT类型变量地址
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DrawDownRectEx(const WIDGET_EFFECT* pEffect, const GUI_RECT* pRect)
{
	WM_LOCK();
	pEffect->pfDrawDownRect(pRect);
	WM_UNLOCK();
}

/*
*********************************************************************************************************
*	函 数 名: _DrawDownRect
*	功能说明: 显示凹陷的矩形框
*	形    参：pEffect  控件显示效果
*             x0       起始x轴坐标
*             y0       起始y轴坐标
*             x1       结束x轴坐标
*             y1       结束y轴坐标
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DrawDownRect(const WIDGET_EFFECT* pEffect, int x0, int y0, int x1, int y1)
{
	GUI_RECT r;

	r.x0 = x0;
	r.y0 = y0;
	r.x1 = x1;
	r.y1 = y1;
	_DrawDownRectEx(pEffect, &r);
}

/*
*********************************************************************************************************
*	函 数 名: _DrawUpRectEx
*	功能说明: 显示凸起的矩形框
*	形    参：pEffect  控件显示效果
*             pRect    GUI_RECT类型变量地址
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DrawUpRectEx(const WIDGET_EFFECT* pEffect, const GUI_RECT* pRect)
{
	WM_LOCK();
	pEffect->pfDrawUpRect(pRect);
	WM_UNLOCK();
}

/*
*********************************************************************************************************
*	函 数 名: _PaintFrame
*	功能说明: 框架窗口的重绘函数
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void _PaintFrame(void)
{
	GUI_RECT r;
	WM_GetClientRect(&r);
	GUI_SetBkColor(FRAME_BKCOLOR);
	GUI_SetColor(FRAME_TEXTCOLOR);
	GUI_SetFont(FRAME_FONT);
	GUI_SetTextMode(GUI_TM_TRANS);
	GUI_ClearRectEx(&r);
}

/*
*********************************************************************************************************
*	函 数 名: _CreateFrame
*	功能说明: 创建框架窗口
*	形    参：cb  回调函数地址
*	返 回 值: 无
*********************************************************************************************************
*/
static WM_HWIN _CreateFrame(WM_CALLBACK* cb)
{
#if 0
	//int x = 0;
	//int y = 0;
	//x = FRAME_BORDER + MAIN_BORDER;
	//y = FRAME_BORDER + MAIN_TITLE_HEIGHT;

	//_hLastFrame = WM_CreateWindowAsChild(x, y,
                                           FRAME_WIDTH,
                                           FRAME_HEIGHT,
                                           WM_HBKWIN,
                                           WM_CF_SHOW, cb, 0);
#else
	_hLastFrame = WM_CreateWindowAsChild(0, 0,
	                                         FRAME_WIDTH,
	                                         FRAME_HEIGHT,
	                                         WM_HBKWIN,
	                                         WM_CF_SHOW, cb, 0);
#endif
	return _hLastFrame;
}

/*
*********************************************************************************************************
*	函 数 名: _DeleteFrame
*	功能说明: 删除创建的框架窗口
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DeleteFrame(void)
{
	WM_DeleteWindow(_hLastFrame);
	_hLastFrame = 0;
}

/*
*********************************************************************************************************
*	函 数 名: _CreateButton
*	功能说明: 创建按钮
*	形    参：hParent  父窗口
*             pText    按键上显示的文本
*             Id       按钮Id
*             x        x轴坐标
*             y        y轴坐标
*             w        按钮宽
*             h        按钮高
*             TextId   文本的ID
*	返 回 值: 无
*********************************************************************************************************
*/
static WM_HWIN _CreateButton(WM_HWIN hParent, const char* pText, int Id, int x, int y, int w, int h, unsigned TextId)
{
	WM_HWIN hButton;
	hButton = BUTTON_CreateEx(x, y, w, h, hParent, WM_CF_SHOW, 0, Id);

	/* 获取当前创建按钮要显示的文本 */
	if (TextId)
	{
		pText = _GetLang(TextId);
	}
	BUTTON_SetText      (hButton,    pText);
	BUTTON_SetFont      (hButton,    FRAME_BUTTON_FONT);
	BUTTON_SetBkColor   (hButton, 0, FRAME_BUTTON_BKCOLOR0);
	BUTTON_SetBkColor   (hButton, 1, FRAME_BUTTON_BKCOLOR1);
	BUTTON_SetBkColor   (hButton, 2, FRAME_BUTTON_BKCOLOR2);
	BUTTON_SetTextColor (hButton, 0, FRAME_BUTTON_COLOR0);
	BUTTON_SetTextColor (hButton, 1, FRAME_BUTTON_COLOR1);
	BUTTON_SetTextColor (hButton, 2, FRAME_BUTTON_COLOR0);
	WIDGET_SetEffect    (hButton,    FRAME_BUTTON_EFFECT);

	/* 设置接收输入焦点的能力 */
	BUTTON_SetFocussable(hButton,    1);

	return hButton;
}

/*
*********************************************************************************************************
*	函 数 名: _DrawLogoBox
*	功能说明: 小键盘的回调函数
*	形    参：Index     要显示的位图序号
*             pBitmap   位图地址
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DrawLogoBox(int Index, const GUI_BITMAP GUI_UNI_PTR* pBitmap)
{
	int x, y, w, h;

	x  = (FRAME_WIDTH - (5 * LOGO_FRAME_SIZE_X) - (4 * LOGO_FRAME_DIST_X)) >> 1;
	y  = FRAME_HEIGHT - LOGO_FRAME_OFFSET_Y - LOGO_FRAME_SIZE_Y;
	w  = LOGO_FRAME_SIZE_X;
	h  = LOGO_FRAME_SIZE_Y;
	x += Index * (w + LOGO_FRAME_DIST_X);
	GUI_SetBkColor(LOGO_FRAME_BKCOLOR);
	GUI_ClearRect(x, y, x + w - 1, y + h - 1);
	_DrawDownRect(LOGO_FRAME_EFFECT, x, y, x + w - 1, y + h - 1);
	x += (w - pBitmap->XSize) >> 1;
	y += (h - pBitmap->YSize) >> 1;
	GUI_DrawBitmap(pBitmap, x, y);
	GUI_SetBkColor(FRAME_BKCOLOR);
	GUI_SetColor(FRAME_TEXTCOLOR);
	GUI_SetFont(FRAME_FONT);
}

/*
*********************************************************************************************************
*	函 数 名: _cbNumPad
*	功能说明: 小键盘的回调函数
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbNumPad(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{

		case WM_CREATE:
			/* 创建后设置聚焦 */
			WM_SetFocus(hWin);
			_PIN_Value  = 0;
			_PIN_Digits = 0;
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;
            }
            break;

		case WM_PAINT:
			{
				GUI_RECT r;
				WM_GetClientRect(&r);
				GUI_SetBkColor(NUMPAD_BKCOLOR);
				GUI_Clear();
				_DrawUpRectEx(NUMPAD_EFFECT, &r);
			}
			break;

		case WM_NOTIFY_PARENT:
			if (pMsg->Data.v == WM_NOTIFICATION_RELEASED)
			{
				int Id = WM_GetId(pMsg->hWinSrc);
				switch (Id)
				{
					case NUMPAD_ID_0:
					case NUMPAD_ID_1:
					case NUMPAD_ID_2:
					case NUMPAD_ID_3:
					case NUMPAD_ID_4:
					case NUMPAD_ID_5:
					case NUMPAD_ID_6:
					case NUMPAD_ID_7:
					case NUMPAD_ID_8:
					case NUMPAD_ID_9:
						/* 通过这种方式将每次输入的数值添加到之前数值的末尾 */
						_PIN_Value = (_PIN_Value * 10) + (Id - NUMPAD_ID_0);
						/* _PIN_Digits是设置密码位数的 */
						if (_PIN_Digits < 4)
						{
							_PIN_Digits++;
							WM_SendMessageNoPara(WM_GetParent(hWin), MSG_PIN_CHANGED);
						}
						break;

					case NUMPAD_ID_A:   /* Cancel   */
						WM_SendMessageNoPara(WM_GetParent(hWin), MSG_PIN_CANCEL);
						break;

					case NUMPAD_ID_B:   /* Correct  */
						/* 通过这种方式用于实现更正 */
						_PIN_Value /= 10;
						if (_PIN_Digits)
						{
							_PIN_Digits--;
						}
						WM_SendMessageNoPara(WM_GetParent(hWin), MSG_PIN_CHANGED);
						break;

					case NUMPAD_ID_D:   /* Confirm  */
						/* 输入四个后才认为正确 */
						if (_PIN_Digits == 4)
						{
							WM_SendMessageNoPara(WM_GetParent(hWin), MSG_PIN_OK);
						}
						else
						{
							WM_SendMessageNoPara(WM_GetParent(hWin), MSG_PIN_ERROR);
						}
						break;
				}
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _CreateNumPadButton
*	功能说明: 创建小键盘所需要的按钮
*	形    参：hParent  父窗口
*             pText    按键上显示的文本
*             Id       按钮Id
*             x        x轴坐标
*             y        y轴坐标
*             w        按钮宽
*             h        按钮高
*             Color    按钮颜色
*             PressedColor  按钮被按下后的颜色
*             Enable   使能或者禁止状态
*             TextId   文本的ID
*	返 回 值: 无
*********************************************************************************************************
*/
static WM_HWIN _CreateNumPadButton(WM_HWIN hParent, const char* pText, int Id,
                                   int x, int y, int w, int h,
                                   GUI_COLOR Color, GUI_COLOR PressedColor, int Enable, unsigned TextId)
{
	WM_HWIN hButton;


	hButton = BUTTON_CreateEx(x, y, w, h, hParent, WM_CF_SHOW, 0, Id);
	if (Enable)
	{
		if (TextId)
		{
			pText = _GetLang(TextId);
		}

		BUTTON_SetText(hButton, pText);
	}
	else
	{
		WM_DisableWindow(hButton);
	}

	BUTTON_SetFont      (hButton,    NUMPAD_BUTTON_FONT);
	BUTTON_SetBkColor   (hButton, 0, NUMPAD_BUTTON_BKCOLOR0);
	BUTTON_SetBkColor   (hButton, 1, NUMPAD_BUTTON_BKCOLOR1);
	BUTTON_SetBkColor   (hButton, 2, NUMPAD_BUTTON_BKCOLOR2);
	BUTTON_SetTextColor (hButton, 0, Color);
	BUTTON_SetTextColor (hButton, 1, PressedColor);
	BUTTON_SetTextColor (hButton, 2, Color);
	BUTTON_SetFocussable(hButton, 1);
	WIDGET_SetEffect    (hButton, NUMPAD_BUTTON_EFFECT);

	return hButton;
}

/*
*********************************************************************************************************
*	函 数 名: _CreateNumPad
*	功能说明: 创建小键盘
*	形    参：hParent  父窗口
*             x        x轴坐标
*             y        y轴坐标
*	返 回 值: 无
*********************************************************************************************************
*/
static WM_HWIN _CreateNumPad(WM_HWIN hParent, int x, int y)
{
	int w, h, bx, by, bw, bh, dx, dy;

	/* 位置和大小 */
	bx = NUMPAD_BORDER;
	by = NUMPAD_BORDER;
	bw = NUMPAD_BUTTON_WIDTH;
	bh = NUMPAD_BUTTON_HEIGHT;
	dx = NUMPAD_BUTTON_DIST_X;
	dy = NUMPAD_BUTTON_DIST_Y;
	w  = (bx * 2) + (bw * 11/2) + (dx * 5);
	h  = (by * 2) + (bh * 4)    + (dy * 3);
	x -= (w >> 1);
	y -= (h >> 1);

	/* 创建键盘窗口   */
	_hNumPad = WM_CreateWindowAsChild(x, y, w, h, hParent, WM_CF_SHOW, _cbNumPad, 0);

	/* 为键盘窗口创建按钮 */
	_CreateNumPadButton(_hNumPad, "1", NUMPAD_ID_1, bx + 0*(bw+dx),   by + 0*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "2", NUMPAD_ID_2, bx + 1*(bw+dx),   by + 0*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "3", NUMPAD_ID_3, bx + 2*(bw+dx),   by + 0*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "4", NUMPAD_ID_4, bx + 0*(bw+dx),   by + 1*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "5", NUMPAD_ID_5, bx + 1*(bw+dx),   by + 1*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "6", NUMPAD_ID_6, bx + 2*(bw+dx),   by + 1*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "7", NUMPAD_ID_7, bx + 0*(bw+dx),   by + 2*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "8", NUMPAD_ID_8, bx + 1*(bw+dx),   by + 2*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "9", NUMPAD_ID_9, bx + 2*(bw+dx),   by + 2*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "*", NUMPAD_ID_X, bx + 0*(bw+dx),   by + 3*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  0, 0) ;
	_CreateNumPadButton(_hNumPad, "0", NUMPAD_ID_0, bx + 1*(bw+dx),   by + 3*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  1, 0);
	_CreateNumPadButton(_hNumPad, "#", NUMPAD_ID_Y, bx + 2*(bw+dx),   by + 3*(bh+dy), bw,     bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  0, 0);
	_CreateNumPadButton(_hNumPad, "",  NUMPAD_ID_A, bx + 3*bw + 5*dx, by + 0*(bh+dy), bw*5/2, bh, NUMPAD_BUTTON_RED0,    NUMPAD_BUTTON_RED1,    1, TEXT_ID_ABBRUCH);
	_CreateNumPadButton(_hNumPad, "",  NUMPAD_ID_B, bx + 3*bw + 5*dx, by + 1*(bh+dy), bw*5/2, bh, NUMPAD_BUTTON_BLUE0,   NUMPAD_BUTTON_BLUE1,   1, TEXT_ID_KORREKTUR);
	_CreateNumPadButton(_hNumPad, "",  NUMPAD_ID_C, bx + 3*bw + 5*dx, by + 2*(bh+dy), bw*5/2, bh, NUMPAD_BUTTON_COLOR0,  NUMPAD_BUTTON_COLOR1,  0, 0);
	_CreateNumPadButton(_hNumPad, "",  NUMPAD_ID_D, bx + 3*bw + 5*dx, by + 3*(bh+dy), bw*5/2, bh, NUMPAD_BUTTON_GREEN0,  NUMPAD_BUTTON_GREEN1,  1, TEXT_ID_BESTAETIGEN);

	return _hNumPad;
}

/*
*********************************************************************************************************
*	函 数 名: _DeleteNumPad
*	功能说明: 删除键盘
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void _DeleteNumPad(void)
{
	WM_DeleteWindow(_hNumPad);
}

/*
*********************************************************************************************************
*	函 数 名: _cbBkWindow
*	功能说明: 桌面窗口回调函数
*	形    参：pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
//static void _cbBkWindow(WM_MESSAGE* pMsg)
//{
//	switch (pMsg->MsgId)
//	{
//		case WM_PAINT:
//			{
//				int x, y, w, h;
//
//				GUI_SetBkColor(MAIN_BKCOLOR);
//				GUI_SetColor(MAIN_TEXTCOLOR);
//				GUI_SetFont(MAIN_FONT);
//				GUI_Clear();
//				//x = MAIN_LOGO_OFFSET_X + MAIN_BORDER;
//				//y = MAIN_LOGO_OFFSET_Y + ((MAIN_TITLE_HEIGHT - MAIN_LOGO_BITMAP->YSize) >> 1);
//				GUI_DrawBitmap(MAIN_LOGO_BITMAP, 0, 0);
//				//x = MAIN_BORDER;
//				//y = MAIN_TITLE_HEIGHT;
//				//w = LCD_GetXSize() - (MAIN_BORDER * 2);
//				//h = LCD_GetYSize()  - (MAIN_BORDER + MAIN_TITLE_HEIGHT);
//				//_DrawDownRect(FRAME_EFFECT, x, y, x + w - 1, y + h - 1);
//			}
//			break;
//		default:
//			WM_DefaultProc(pMsg);
//	}
//}

/*
*********************************************************************************************************
*	函 数 名: _cbThanks
*	功能说明: 第11个界面，最后一个感谢使用界面
*	形    参：pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbThanks(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;
	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;

				case GUI_KEY_BACKTAB:
					{
						GUI_PID_STATE CurrentState;
						CurrentState.x = WM_GetWindowOrgX(hWin)+4;
						CurrentState.y = WM_GetWindowOrgY(hWin)+4;
						CurrentState.Layer = 0;
						CurrentState.Pressed = 1;
						GUI_PID_StoreState(&CurrentState);
					}
					break;
            }
            break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_VIELEN_DANK), FRAME_WIDTH >> 1, 30);
			GUI_DrawBitmap(&bmLogo_armfly, (FRAME_WIDTH - bmLogo_armfly.XSize) >> 1, 80);
			break;

		case WM_TOUCH:
			if (((GUI_PID_STATE *)pMsg->Data.p)->Pressed == 1)
			{
				_DeleteFrame();
				WM_DeleteWindow(_hTitle);
				//_CreateFrame(&_cbLanguage);
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbRemoveMoney
*	功能说明: 第10个界面，取钱界面
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbRemoveMoney(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;
	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;

				case GUI_KEY_BACKTAB:
					{
						GUI_PID_STATE CurrentState;
						CurrentState.x = WM_GetWindowOrgX(hWin)+3;
						CurrentState.y = WM_GetWindowOrgY(hWin)+3;
						CurrentState.Layer = 0;
						CurrentState.Pressed = 1;
						GUI_PID_StoreState(&CurrentState);
					}
					break;

				case GUI_KEY_LEFT:
					break;
            }
            break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_GELD_ENTNEHMEN), FRAME_WIDTH >> 1, 100);
			break;

		case WM_TOUCH:
			if (((GUI_PID_STATE *)pMsg->Data.p)->Pressed == 1)
			{
				/* 最后一个界面 */
				_DeleteFrame();
				_CreateFrame(&_cbThanks);
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbRemoveCard
*	功能说明: 第9个界面，取出银行卡界面
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbRemoveCard(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;

				case GUI_KEY_BACKTAB:
					{
						GUI_PID_STATE CurrentState;
						CurrentState.x = WM_GetWindowOrgX(hWin)+2;
						CurrentState.y = WM_GetWindowOrgY(hWin)+2;
						CurrentState.Layer = 0;
						CurrentState.Pressed = 1;
						GUI_PID_StoreState(&CurrentState);
					}
					break;
            }
            break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_KARTE_ENTNEHMEN), FRAME_WIDTH >> 1, 100);
			break;

		case WM_TOUCH:
			if (((GUI_PID_STATE *)pMsg->Data.p)->Pressed == 1)
			{
				/* 取钱界面 */
				_DeleteFrame();
				_CreateFrame(&_cbRemoveMoney);
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbWait
*	功能说明: 第8个界面，进入等待界面
*	形    参：pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbWait(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;
	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
		    GUI_CURSOR_SelectAnim(&GUI_CursorAnimHourglassM);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;

				case GUI_KEY_BACKTAB:
					{
						GUI_PID_STATE CurrentState;
						CurrentState.x = WM_GetWindowOrgX(hWin)+1;
						CurrentState.y = WM_GetWindowOrgY(hWin)+1;
						CurrentState.Layer = 0;
						CurrentState.Pressed = 1;
						GUI_PID_StoreState(&CurrentState);
					}
					break;
            }
            break;
		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_BITTE_GEDULD), FRAME_WIDTH >> 1, 100);
			break;

		case WM_TOUCH:
			if (((GUI_PID_STATE *)pMsg->Data.p)->Pressed == 1)
			{
				GUI_CURSOR_SelectAnim(NULL);
				/* 进入取出银行卡界面 */
				_DeleteFrame();
				_CreateFrame(&_cbRemoveCard);
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbSelectMoney
*	功能说明: 第7个界面，取款界面
*	形    参：pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbSelectMoney(WM_MESSAGE* pMsg);
static void _cbEnterAmount(WM_MESSAGE* pMsg)
{
	char acBuffer[5] = {0};
	char * pBuffer;
	WM_HWIN hItem, hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			/* 设置聚焦 */
			WM_SetFocus(hWin);
			_CreateNumPad(hWin, FRAME_WIDTH >> 1, (55 + FRAME_HEIGHT)>>1);
			hItem = EDIT_CreateEx((FRAME_WIDTH >> 1) - 50, 30, 100, 25, hWin, WM_CF_SHOW, 0, GUI_ID_EDIT0, 4);
			EDIT_SetFont(hItem, &GUI_FontYahei);
			EDIT_SetTextAlign(hItem, GUI_TA_CENTER);
			WIDGET_SetEffect(hItem, DEFAULT_WIDGET_EFFECT);
			WM_DisableWindow(hItem);
			break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_BETRAG_EINGEBEN), FRAME_WIDTH >> 1, 0);/**/
			break;

		case MSG_PIN_ERROR:
			/* 这里是保证必须得选择金额，如果没有选择金额就点击进入下一个界面会出错 */
			if (!_PIN_Digits)
			{
				break;
			}
			/* 输入金额后，由于没有break语句会自动的进入下一个界面 */
		case MSG_PIN_OK:
			_DeleteNumPad();
			_DeleteFrame();
			_CreateFrame(&_cbWait);
			break;

		case MSG_PIN_CANCEL:
			/* 返回到上一个界面 */
			_DeleteNumPad();
			_DeleteFrame();
			_CreateFrame(&_cbSelectMoney);
			break;

		case MSG_PIN_CHANGED:
			pBuffer = acBuffer;
		    /* 将十进制数值转换成字符在编辑框显示 */
			GUI_AddDec(_PIN_Value, _PIN_Digits, &pBuffer);
			EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), acBuffer);
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbSelectMoney
*	功能说明: 第六个界面，取款界面
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbSelectMoney(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
			_CreateButton(hWin,  "50,00 RMB",     GUI_ID_BUTTON0,  5,  30, 200,  40, 0);
			_CreateButton(hWin, "100,00 RMB",     GUI_ID_BUTTON1,  5,  80, 200,  40, 0);
			_CreateButton(hWin, "150,00 RMB",     GUI_ID_BUTTON2,  5, 130, 200,  40, 0);
			_CreateButton(hWin, "200,00 RMB",     GUI_ID_BUTTON3,  5, 180, 200,  40, 0);
			_CreateButton(hWin, "250,00 RMB",     GUI_ID_BUTTON4, FRAME_WIDTH-200-5,  30, 200,  40, 0);
			_CreateButton(hWin, "300,00 RMB",     GUI_ID_BUTTON5, FRAME_WIDTH-200-5,  80, 200,  40, 0);
			_CreateButton(hWin, "350,00 RMB",     GUI_ID_BUTTON6, FRAME_WIDTH-200-5, 130, 200,  40, 0);
			_CreateButton(hWin, "",               GUI_ID_BUTTON7, FRAME_WIDTH-200-5, 180, 200,  40, TEXT_ID_ANDERER_BETRAG);
			break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_AUSWAHL_BETRAG), FRAME_WIDTH >> 1, 2);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;
				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;
            }
            break;

		case WM_NOTIFY_PARENT:
			if (pMsg->Data.v == WM_NOTIFICATION_RELEASED)
			{
				int Id = WM_GetId(pMsg->hWinSrc);

				switch (Id)
				{
					/* 如果按下的是按键7 进入自定义金额显示 */
					case GUI_ID_BUTTON7:
						_DeleteFrame();
						_CreateFrame(&_cbEnterAmount);
						break;

					/* 选择指定金额 */
					default:
						_DeleteFrame();
						_CreateFrame(&_cbWait);
						break;
				}
			}
			break;
		/*
		case WM_NOTIFY_PARENT:
		if (pMsg->Data.v == WM_NOTIFICATION_RELEASED) {
		_DeleteFrame();
		_CreateFrame(&_cbWait);
		}
		break;
		*/
		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbShowBalance
*	功能说明: 第五个界面，显示余额
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbShowBalance(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			/* 设置聚焦 */
			WM_SetFocus(hWin);
			break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_KONTOSTAND), FRAME_WIDTH >> 1, 100);
			break;

		case WM_TOUCH:
			if (((GUI_PID_STATE *)pMsg->Data.p)->Pressed == 1)
			{
				/* 显示完余额后进入最后一个界面 */
				_DeleteFrame();
				_CreateFrame(&_cbThanks);
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbMainMenu
*	功能说明: 第四个界面，选择进入余额显示或者取款界面
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbMainMenu(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;
	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			WM_SetFocus(hWin);
			_CreateButton(hWin, "", GUI_ID_BUTTON0, (FRAME_WIDTH >> 1) - 150, 70, 300,  60, TEXT_ID_GELD_ABHEBEN);
			_CreateButton(hWin, "", GUI_ID_BUTTON1, (FRAME_WIDTH >> 1) - 150, 160, 300,  60, TEXT_ID_KONTOSTAND_ANSEHEN);
			break;

		case WM_PAINT:
			_PaintFrame();
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_AUSWAHL_TREFFEN), FRAME_WIDTH >> 1, 15);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_ESCAPE:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_TAB:
					WM_SetFocusOnNextChild(hWin);
					break;
            }
            break;

		case WM_NOTIFY_PARENT:
			if (pMsg->Data.v == WM_NOTIFICATION_RELEASED)
			{
				int Id = WM_GetId(pMsg->hWinSrc);
				switch (Id)
				{
					/* 通过两个按钮，选择进入相应的界面 */
					case GUI_ID_BUTTON0:
						_DeleteFrame();
						_CreateFrame(&_cbSelectMoney);
						break;

					case GUI_ID_BUTTON1:
						_DeleteFrame();
						_CreateFrame(&_cbShowBalance);
						break;
				}
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbEnterPIN
*	功能说明: 第三个界面，输入密码
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbInsertCard(WM_MESSAGE* pMsg);
static void _cbEnterPIN(WM_MESSAGE* pMsg)
{
	WM_HWIN hItem, hWin = pMsg->hWin;
	static int PinError;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:

			/* 创建后，设置聚焦 */
			PinError = 0;
			WM_SetFocus(hWin);

		    /* 创建小键盘,数值55是下面编辑框的高度和起始位置的和30+25=55*/
			_CreateNumPad(hWin, FRAME_WIDTH >> 1, (55 + FRAME_HEIGHT)>>1);
			hItem = EDIT_CreateEx((FRAME_WIDTH >> 1) - 50, 30, 100, 25, hWin, WM_CF_SHOW, 0, GUI_ID_EDIT0, 4);
			EDIT_SetFont(hItem, &GUI_FontYahei);
			EDIT_SetTextAlign(hItem, GUI_TA_CENTER);
			WIDGET_SetEffect(hItem, DEFAULT_WIDGET_EFFECT);
			WM_DisableWindow(hItem);
			break;

		case WM_PAINT:
			_PaintFrame();
		    /* 为了方便显示，这里将密码输入错误时的文本和这个界面的小标题显示在一个位置上 */
			if (PinError)
			{
				_PIN_Digits = 0;
				GUI_SetColor(GUI_RED);
				GUI_DispStringHCenterAt(_GetLang(TEXT_ID_FALSCHE_PIN), FRAME_WIDTH >> 1, 0);
				WM_CreateTimer(hWin, 0, 1000, 0);
			}
			else
			{
				GUI_DispStringHCenterAt(_GetLang(TEXT_ID_PIN_EINGEBEN), FRAME_WIDTH >> 1, 0);
			}

			/* 这里是将密码输入错误时的文本和这个界面的小标题显示在不同位置上 */
			/*
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_PIN_EINGEBEN), FRAME_WIDTH >> 1, 0);
			if (PinError)
			{
				_PIN_Digits = 0;
				GUI_SetColor(GUI_RED);
				GUI_DispStringHCenterAt(_GetLang(TEXT_ID_FALSCHE_PIN), FRAME_WIDTH >> 1, 0);
				WM_CreateTimer(hWin, 0, 1000, 0);
			}
			*/
			break;

		case WM_TIMER:
			/* 密码输入错误，1s后将编辑框中的密码清除 */
			PinError = 0;
			EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "");
			/* 使能重绘 */
			WM_InvalidateWindow(hWin);
			break;

		case MSG_PIN_ERROR:
			/* 密码输入错误，使能重绘 */
			PinError++;
			WM_InvalidateWindow(hWin);
			break;

		case MSG_PIN_OK:
			/* 密码输入正确，进入下一个界面 */
			_DeleteNumPad();
			_DeleteFrame();
			_CreateFrame(&_cbMainMenu);
			break;

		case MSG_PIN_CANCEL:
			/* 取消，返回到上一个界面 */
			_DeleteNumPad();
			_DeleteFrame();
			_CreateFrame(&_cbInsertCard);
			break;

		case MSG_PIN_CHANGED:
			/* 输入密码实时更新 */
			if (_PIN_Digits == 0)
			{
				EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "");
			}
			else if (_PIN_Digits == 1)
			{
				EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "x");
			}
			else if (_PIN_Digits == 2)
			{
				EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "xx");
			}
			else if (_PIN_Digits == 3)
			{
				EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "xxx");
			} else if (_PIN_Digits == 4)
			{
				EDIT_SetText(WM_GetDialogItem(hWin, GUI_ID_EDIT0), "xxxx");
			}
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}

/*
*********************************************************************************************************
*	函 数 名: _cbInsertCard
*	功能说明: 第二个界面，等待用户插入银行卡
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbInsertCard(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			/* 设置聚焦 */
			WM_SetFocus(hWin);
			break;

		case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
				case GUI_KEY_BACKTAB:
                    GUI_EndDialog(hWin, 1);
                    break;

				case GUI_KEY_ENTER:
					//WM_SetFocusOnNextChild(hWin);
					break;

				//case GUI_KEY_BACKTAB:
				//	{
				//		GUI_PID_STATE CurrentState;
				//		CurrentState.x = WM_GetWindowOrgX(hWin);
				//		CurrentState.y = WM_GetWindowOrgY(hWin);
				//		CurrentState.Layer = 0;
				//		CurrentState.Pressed = 1;
				//		//GUI_PID_StoreState(&CurrentState);
				//	}
				//	break;
            }
            break;

		case WM_PAINT:
			_PaintFrame();
			_DrawLogoBox(0, &bmLogo_ECCard);
			_DrawLogoBox(1, &bmLogo_Maestro);
			_DrawLogoBox(2, &bmLogo_MasterCard);
			_DrawLogoBox(3, &bmLogo_VisaCard);
			_DrawLogoBox(4, &bmLogo_AmericanExpress);
			GUI_DispStringHCenterAt(_GetLang(TEXT_ID_KARTE_EINFUEHREN),
                                      FRAME_WIDTH >> 1, 15);
			break;

		default:
			WM_DefaultProc(pMsg);
	}
}


/*
*********************************************************************************************************
*	函 数 名: _cbLanguage
*	功能说明: 第一个界面，用于中文和英语的选择
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbShowDesktop(WM_MESSAGE* pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			GUI_DispStringHCenterAt("WM_CREATE", FRAME_WIDTH >> 1, 64);
			/* 设置聚焦 */
			WM_SetFocus(hWin);
			break;

		case WM_PAINT:
			_PaintFrame();
            /* 绘制背景图片 */
            //GUI_DrawBitmap(MAIN_LOGO_BITMAP, 0, 28);

			GUI_DispStringHCenterAt("Please select your language", FRAME_WIDTH >> 1, 32);
            break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_BACKTAB:
                    _DeleteFrame();
                    _CreateFrame(&_cbDrawHomePage);
                    break;

                case GUI_KEY_MUSIC://播放音乐
                    break;

        		case GUI_KEY_SLEEPMODE://进入睡眠模式
                    break;

                case GUI_KEY_DOWN://音量减小
                    break;

                case GUI_KEY_UP://音量增加
                    break;

                case KEY_DOWN_MUX://锁屏
                    break;

                case KEY_DOWN_MUX_LONG://解锁
                    break;

                default:
                    break;
            }
		default:
			WM_DefaultProc(pMsg);
            break;
	}
}

static void _cbDrawHomePage(WM_MESSAGE * pMsg)
{
  //const void * pData;
  //WM_HWIN      hItem;
  //U32          FileSize;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  // USER START (Optionally insert additional message handling)
  case WM_CREATE:
	/* 设置聚焦 */
    GUI_DispStringHCenterAt("WM_CREATE",FRAME_WIDTH >> 1,32);
	WM_SetFocus(pMsg->hWin);
	break;

  case WM_PAINT:
      _PaintFrame();
      GUI_DispStringHCenterAt("WM_PAINT",FRAME_WIDTH >> 1,64);
      break;

  case WM_KEY:
	switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
    {
		case GUI_KEY_BACKTAB:
			_DeleteFrame();
            _CreateFrame(&_cbShowDesktop);
			break;

        case GUI_KEY_MUSIC://播放音乐
            break;

		case GUI_KEY_SLEEPMODE://进入睡眠模式
            break;

        case GUI_KEY_DOWN://音量减小
            break;

        case GUI_KEY_UP://音量增加
            break;

        case KEY_DOWN_MUX://锁屏
            break;

        case KEY_DOWN_MUX_LONG://解锁
            break;

        default:
            break;
    }
    break;

  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*
*********************************************************************************************************
*	函 数 名: MainTask
*	功能说明: GUI主函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MainTask(void)
{
    /* 初始化 */
	GUI_Init();

    /****************************************************************************
     * 关于多缓冲和窗口内存设备的设置说明
     * 1. 使能多缓冲是调用的如下函数，用户要在LCDConf_Lin_Template.c文件中
     *    配置了多缓冲，调用此函数才有效：WM_MULTIBUF_Enable(1);
     * 2. 窗口使能使用内存设备是调用函数：WM_SetCreateFlags(WM_CF_MEMDEV);
     * 3. 如果emWin的配置多缓冲和窗口内存设备都支持，二选一即可，且务必优先
     *    选择使用多缓冲，实际使 用STM32F429BIT6 + 32位SDRAM + RGB565/RGB888
     *    平台测试，多缓冲可以有效的降低窗口移动或者滑动时的撕裂感，
     *    并有效的提高流畅性，通过使能窗口使用内存设备是做不到的。
     * 4. 所有emWin例子默认是开启三缓冲。
    *****************************************************************************/
#if 1
    WM_MULTIBUF_Enable(1);
#else
    /* 创建使用内存设备 */
	WM_SetCreateFlags(WM_CF_MEMDEV);
#endif

	/* 使能桌面窗口也使用内存设备 */
    WM_EnableMemdev(WM_HBKWIN);

	/* 使能UTF8解码 */
	GUI_UC_SetEncodeUTF8();

	/* 设置桌面窗口的回调函数 */
	//WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
	//WM_SetCallback(WM_HBKWIN, &_cbLanguage);

	/* 进入主界面 */
	_CreateFrame(&_cbDrawHomePage);

	while(1)
	{
		GUI_Delay(10);
	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
