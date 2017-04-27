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
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_AmericanExpress;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_ECCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_GeldKarte;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_Maestro;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_MasterCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_VisaCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armfly;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armflySmall;

extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;

extern GUI_CONST_STORAGE GUI_BITMAP bma;
extern GUI_CONST_STORAGE GUI_BITMAP bmb;
extern GUI_CONST_STORAGE GUI_BITMAP bmc;
extern GUI_CONST_STORAGE GUI_BITMAP bmd;
extern GUI_CONST_STORAGE GUI_BITMAP bme;
extern GUI_CONST_STORAGE GUI_BITMAP bmf;
extern GUI_CONST_STORAGE GUI_BITMAP bmg;
extern GUI_CONST_STORAGE GUI_BITMAP bmh;
extern GUI_CONST_STORAGE GUI_BITMAP bmi;
extern GUI_CONST_STORAGE GUI_BITMAP bmj;

/*
*********************************************************************************************************
*                                       宏定义
*********************************************************************************************************
*/
#define MAIN_BORDER               0
#define MAIN_TITLE_HEIGHT         0

#define FRAME_BKCOLOR             0xD0D0D0
#define FRAME_TEXTCOLOR           0x000000
#define FRAME_FONT                (&GUI_FontYahei)
#define FRAME_EFFECT              (&WIDGET_Effect_Simple)
#define FRAME_BORDER              FRAME_EFFECT->EffectSize
#define FRAME_WIDTH               (LCD_GetXSize() - (FRAME_BORDER * 2) - (MAIN_BORDER * 2))
#define FRAME_HEIGHT              (LCD_GetYSize() - (FRAME_BORDER * 2) - (MAIN_BORDER + MAIN_TITLE_HEIGHT))

/*
*********************************************************************************************************
*                                       宏定义-常数
*********************************************************************************************************
*/
#define MSG_CARD_INSERTED   (WM_USER + 0)
#define MSG_CARD_REMOVED    (WM_USER + 1)
#define MSG_MONEY_REMOVED   (WM_USER + 2)
#define MSG_PIN_CHANGED     (WM_USER + 3)
#define MSG_PIN_OK          (WM_USER + 4)
#define MSG_PIN_CANCEL      (WM_USER + 5)
#define MSG_PIN_ERROR       (WM_USER + 6)

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


/* 用于桌面ICONVIEW图标的创建 */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

/*
*********************************************************************************************************
*                                       静态数据
*********************************************************************************************************
*/

static WM_HWIN    _hLastFrame;

WM_HWIN  hWinInfo;   /* 通过ICONVIEW所打开窗口的句柄 */
WM_HWIN  hWinICON;   /* ICONVIEW控件句柄 */
WM_HWIN  hWinMain;   /* 主窗口句柄, ICONVIEW控件建立在这个窗口上面 */
uint8_t	s_ucSelIconIndex = 0;	/* 选择的ICON，默认不选择任何 */


extern RTC_T g_tRTC;


/* 用于桌面ICONVIEW图标的创建 */
static const BITMAP_ITEM _aBitmapItem[] =
{
	{&bma,    "Setting"},
	{&bmb,    "Music"},
	{&bmc,    "TDate"},
	{&bmd,    "Sleep"},
	{&bme,    "Lanuage"},
	{&bmf,    "About"},
};


static void _cbDrawHomePage(WM_MESSAGE * pMsg);

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
*	函 数 名: _cbLanguage
*	功能说明: 第一个界面，用于中文和英语的选择
*	形    参: pMsg  参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbShowDesktop(WM_MESSAGE* pMsg)
{
    unsigned char i = 0;
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

			/*在指定位置创建指定尺寸的ICONVIEW 小工具*/
        	hWinICON = ICONVIEW_CreateEx(22, 				/* 小工具的最左像素（在父坐标中）*/
        						     30, 					/* 小工具的最上像素（在父坐标中）*/
        							 220,    				/* 小工具的水平尺寸（单位：像素）*/
        							 280, 	/* 小工具的垂直尺寸（单位：像素）*/
        	                         hWin, 				        /* 父窗口的句柄。如果为0，则新小工具将成为桌面（顶级窗口）的子窗口 */
        							 WM_CF_SHOW | WM_CF_HASTRANS,       /* 窗口创建标记。为使小工具立即可见，通常使用 WM_CF_SHOW */
        	                         0,//ICONVIEW_CF_AUTOSCROLLBAR_V, 	/* 默认是0，如果不够现实可设置增减垂直滚动条 */
        							 GUI_ID_ICONVIEW0, 			        /* 小工具的窗口ID */
        							 82, 				    /* 图标的水平尺寸 */
        							 80);/* 图标的垂直尺寸 */


        	/* 向ICONVIEW 小工具添加新图标 */
        	for (i = 0; i < GUI_COUNTOF(_aBitmapItem); i++)
        	{
        		ICONVIEW_AddBitmapItem(hWinICON, _aBitmapItem[i].pBitmap, _aBitmapItem[i].pText);
        	}

        	/* 设置小工具的背景色 32 位颜色值的前8 位可用于alpha混合处理效果*/
        	ICONVIEW_SetBkColor(hWinICON, ICONVIEW_CI_SEL, GUI_WHITE | 0x80000000);

        	/* 设置字体 */
        	ICONVIEW_SetFont(hWinICON, &GUI_FontYahei);

        	/* 设置图标在x 或y 方向上的间距。*/
        	ICONVIEW_SetSpace(hWinICON, GUI_COORD_Y, 10);
            ICONVIEW_SetSpace(hWinICON, GUI_COORD_X, 20);

        	/* 设置对齐方式 在5.22版本中最新加入的 */
        	ICONVIEW_SetIconAlign(hWinICON, ICONVIEW_IA_HCENTER|ICONVIEW_IA_TOP);

        	WM_CreateTimer(WM_GetClientWindow(hWinMain), /* 接受信息的窗口的句柄 */
        				   1, 	             /* 用户定义的Id。如果不对同一窗口使用多个定时器，此值可以设置为零。 */
        				   20,                           /* 周期，此周期过后指定窗口应收到消息*/
        				   0);	                         /* 留待将来使用，应为0 */
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
  static unsigned char icount = 0;
  char TempDate[] = {'2','0','0','0','-','0','1','-','0','1',' ','0','8',':','0','0',':','0','0',' ','W','7'};
  // USER END

  switch (pMsg->MsgId) {
  // USER START (Optionally insert additional message handling)
  case WM_CREATE:
	/* 设置聚焦 */
    //GUI_DispStringHCenterAt("WM_CREATE",FRAME_WIDTH >> 1,32);
	WM_SetFocus(pMsg->hWin);
	break;

  case WM_PAINT:
      _PaintFrame();

      TempDate[0] = '0' + g_tRTC.Year /1000;
      TempDate[1] = '0' + g_tRTC.Year /100 % 10;
      TempDate[2] = '0' + g_tRTC.Year /10 % 10;
      TempDate[3] = '0' + g_tRTC.Year % 10;

      TempDate[5] = '0' + g_tRTC.Mon / 10;
      TempDate[6] = '0' + g_tRTC.Mon % 10;

      TempDate[8] = '0' + g_tRTC.Day /10;
      TempDate[9] = '0' + g_tRTC.Day % 10;

      TempDate[11] = '0' + g_tRTC.Hour / 10;
      TempDate[12] = '0' + g_tRTC.Hour % 10;

      TempDate[14] = '0' + g_tRTC.Min / 10;
      TempDate[15] = '0' + g_tRTC.Min % 10;

      TempDate[17] = '0' + g_tRTC.Sec / 10;
      TempDate[18] = '0' + g_tRTC.Sec % 10;
      if((icount++) % 2 == 0) {
        TempDate[21] = ' ';
        TempDate[20] = ' ';
      }else{
        TempDate[21] = '0' + g_tRTC.Week;
        TempDate[20] = 'W';
      }


      GUI_DispStringHCenterAt(TempDate, FRAME_WIDTH >> 1,64);
      //WM_RestartTimer(pMsg->Data.v, 300);
      //WM_InvalidateWindow(pMsg->hWin);
      break;

  case WM_TIMER:
      WM_RestartTimer(pMsg->Data.v, 300);
      WM_InvalidateWindow(pMsg->hWin);
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
    //WM_HTIMER hTimer;

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

	/* 进入主界面 */
	_CreateFrame(&_cbDrawHomePage);

    /* 创建定时器 */
    //hTimer = WM_CreateTimer(WM_HBKWIN, 0, 300, 0);

	while(1)
	{
		GUI_Delay(10);
	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
