/*
*********************************************************************************************************
*
*	模块名称 : 音乐播放器应用界面设计
*	文件名称 : App_MucisDlg.c
*	版    本 : V1.0
*	说    明 : 音乐播放器界面设计。
*              1. 支持上一曲，下一曲，快进和快退，常用的采样率和码率都支持，单声道和立体声也都支持。
*              2. emWin任务是低优先级任务，音乐解码任务是高优先级任务，两个任务之间通过任务消息队列和事件
*                 标志进行通信。
*              3. 首次使用先点击歌曲列表，歌曲名会被记录到listview控件里面，然后就可以任意操作了。
*                 如果文件夹中歌曲较多，首次打开会稍慢些，主要是因为要获取每首歌曲的播放时间。以后打开
*                 就比较快了，主要是对歌曲列表对话框做了隐藏和显示处理，而不是重复的创建和删除。
*              4. 歌曲列表对话框做了模态处理，这样用户打开此对话框后只能操作这个对话框，而不能操作主界面。
*
*	修改记录 :
*		版本号    日期         作者         说明
*       V1.0    2016-12-30   Eric2013       首发
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"



/*
*********************************************************************************************************
*	                                  用于本文件的调试
*********************************************************************************************************
*/
#if 0
	#define printf_audiodbg printf
#else
	#define printf_audiodbg(...)
#endif


/*
*********************************************************************************************************
*				                      宏定义
*********************************************************************************************************
*/
#define ID_WINDOW_0 		(GUI_ID_USER + 0x00)
#define ID_IMAGE_0  		(GUI_ID_USER + 0x01)
#define ID_SLIDER_0 		(GUI_ID_USER + 0x02)
#define ID_BUTTON_0 		(GUI_ID_USER + 0x03)
#define ID_BUTTON_1 		(GUI_ID_USER + 0x04)
#define ID_BUTTON_2 		(GUI_ID_USER + 0x05)
#define ID_BUTTON_3 		(GUI_ID_USER + 0x06)
#define ID_BUTTON_4 		(GUI_ID_USER + 0x07)
#define ID_BUTTON_5 		(GUI_ID_USER + 0x08)
#define ID_BUTTON_6 		(GUI_ID_USER + 0x09)
#define ID_SLIDER_1  	    (GUI_ID_USER + 0x0A)
#define ID_TEXT_0 			(GUI_ID_USER + 0x0B)
#define ID_TEXT_1 			(GUI_ID_USER + 0x0C)
#define ID_TEXT_2 			(GUI_ID_USER + 0x0D)
#define ID_TEXT_3 			(GUI_ID_USER + 0x0E)

#define ID_BUTTON_7 		(GUI_ID_USER + 0x0F)

/* 不同定时器的句柄 */
#define ID_TIMER_SPEC       0
#define ID_TIMER_PROCESS    1

/* 歌曲路径字符长度 */
#define MusicPathSzie       100


/*
*********************************************************************************************************
*				                    引用外部变量和函数
*********************************************************************************************************
*/
/* 声明一个支持5个消息的消息邮箱 */
//extern os_mbx_declare (mailbox, 5);
//extern MusicMsg_T s_tMusicMsg;  /* 用于给音乐播放任务发消息，发送音乐类型和歌曲路径 */


/*
*********************************************************************************************************
*				                         变量
*********************************************************************************************************
*/
const char s_MusicPathDir[] = {"M0:\\Music\\"};  /* 存储器中歌曲存放的路径 */
WM_HWIN  hWin_HomePage = WM_HWIN_NULL;               /* 音乐播放对话框句柄 */

/*
*********************************************************************************************************
*				                         任务对话框初始化选项
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateMusic[] =
{

	{ BUTTON_CreateIndirect, "HomePage",  ID_BUTTON_6,    0,   0,  100, 100, 0, 0, 0 },

	{ BUTTON_CreateIndirect, "MusicSet",   ID_BUTTON_7,   0,   110,  100, 100, 0, 0, 0 },

	/* 声音大小 */
	{ SLIDER_CreateIndirect, "Speaker",    ID_SLIDER_1, 10, 220, 200, 20, 0, 0x0, 0  },

};

/*
*********************************************************************************************************
*	函 数 名: Caculate_RTC
*	功能说明: 显示RTC时间
*	形    参：pMsg 指针参数
*	返 回 值: 无
*********************************************************************************************************
*/
#if 0
//extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_InitTypeDef  RTC_InitStructure;
extern RTC_AlarmTypeDef RTC_AlarmStructure;
//extern RTC_DateTypeDef  RTC_DateStructure;
static void Caculate_RTC(WM_MESSAGE * pMsg)
{
	  char buf[30];
	  WM_HWIN hWin = pMsg->hWin;

	  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

	  sprintf(buf,
	          "%0.2d:%0.2d:%0.2d",
			  RTC_TimeStructure.RTC_Hours,
			  RTC_TimeStructure.RTC_Minutes,
			  RTC_TimeStructure.RTC_Seconds);
	 TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_10), buf);

	  sprintf(buf,
	          "20%0.2d/%0.2d/%0.2d",
			  RTC_DateStructure.RTC_Year,
			  RTC_DateStructure.RTC_Month,
			  RTC_DateStructure.RTC_Date);
	  TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_9), buf);
}
#endif
/*
*********************************************************************************************************
*	函 数 名: _cbDialogMusic
*	功能说明: 音乐播放对话框回调消息
*	形    参: pMsg  消息指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbWinCallBack(WM_MESSAGE * pMsg)
{
	static  WM_HTIMER hTimerSpec;

	(void)hTimerSpec;

	switch (pMsg->MsgId)
	{
		/* 对话框初始化消息 */
		case WM_INIT_DIALOG:
            WM_SetFocus(pMsg->hWin);
            printf("_cbDialogMainPage -> WM_CREATE\n");
			break;

        //case WM_CREATE:
        //    WM_SetFocus(pMsg->hWin);
        //    printf("_cbDialogMainPage -> WM_CREATE\n");
        //    break;

		/* 定时器回调消息 */
		case WM_TIMER:
			//Id = WM_GetTimerId(pMsg->Data.v);
	        break;

		/* 重绘消息处理，使用了多缓冲 */
		case WM_PRE_PAINT:
			GUI_MULTIBUF_Begin();
		    break;

		case WM_PAINT:
			//GUI_SetBkColor(GUI_WHITE);
			//GUI_Clear();
			break;

		case WM_POST_PAINT:
			GUI_MULTIBUF_End();
		    break;

		case WM_NOTIFY_PARENT:
			//Id    = WM_GetId(pMsg->hWinSrc);
			//NCode = pMsg->Data.v;
			break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_BACKTAB:
                    GUI_EndDialog(pMsg->hWin, 0);
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

		default:
			WM_DefaultProc(pMsg);
			break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: App_Music
*	功能说明: 创建音乐播放对话框
*	形    参: hWin  父窗口
*	返 回 值: 无
*********************************************************************************************************
*/
void App_HomePage(WM_HWIN hWin)
{
	hWin_HomePage = GUI_CreateDialogBox(_aDialogCreateMusic,
	                                GUI_COUNTOF(_aDialogCreateMusic),
                                  	_cbWinCallBack,
	                                hWin,
	                                0,
	                                0);
    //return hWin_HomePage;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
