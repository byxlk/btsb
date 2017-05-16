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

/*
*********************************************************************************************************
*				                      宏定义
*********************************************************************************************************
*/
#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_TEXT_0 (GUI_ID_USER + 0x01)
#define ID_TEXT_1 (GUI_ID_USER + 0x02)
#define ID_TEXT_2 (GUI_ID_USER + 0x03)
#define ID_TEXT_3 (GUI_ID_USER + 0x04)
#define ID_TEXT_4 (GUI_ID_USER + 0x05)
#define ID_TEXT_5 (GUI_ID_USER + 0x06)
#define ID_TEXT_6 (GUI_ID_USER + 0x07)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x08)
#define ID_PROGBAR_1 (GUI_ID_USER + 0x09)

/*
*********************************************************************************************************
*				                    引用外部变量和函数
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*				                         变量
*********************************************************************************************************
*/
WM_HWIN  hWin_About = WM_HWIN_NULL;               /* 音乐播放对话框句柄 */

/*
*********************************************************************************************************
*				                         任务对话框初始化选项
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate_About[] =
{
    { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 1, 1, 240, 320, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "SN: SA1620123456", ID_TEXT_0, 24, 29, 181, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "VPING", ID_TEXT_1, 66, 71, 99, 42, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Sleep abc", ID_TEXT_2, 66, 132, 92, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Firmware Version: V1.0.0", ID_TEXT_3, 16, 183, 210, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Change Theme", ID_TEXT_4, 51, 226, 131, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Copyright @2017", ID_TEXT_5, 37, 260, 161, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "www.vping.com", ID_TEXT_6, 43, 284, 146, 20, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 5, 50, 232, 5, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_1, 2, 204, 238, 4, 0, 0x0, 0 },
};

/*
*********************************************************************************************************
*	函 数 名: _cbDialogMusic
*	功能说明: 音乐播放对话框回调消息
*	形    参: pMsg  消息指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbWinCallBack_About(WM_MESSAGE * pMsg)
{
	int NCode, Id;
    WM_HWIN hItem;
    WM_HWIN hWin = pMsg->hWin;

    //printf("[%s : %d] MsgId = %d\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);

    switch (pMsg->MsgId)
    {
        case WM_INIT_DIALOG:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
            TEXT_SetFont(hItem, GUI_FONT_20_1);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
            TEXT_SetFont(hItem, GUI_FONT_32B_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
            WM_SetFocus(pMsg->hWin);
            break;
        case WM_PRE_PAINT:
			GUI_MULTIBUF_Begin();
			break;

		case WM_POST_PAINT:
			GUI_MULTIBUF_End();
			break;
		case WM_CREATE:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            break;
		case WM_PAINT:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            break;
		case WM_NOTIFY_PARENT:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;
            switch (Id)
            {
                case GUI_ID_OK:
                    if(NCode==WM_NOTIFICATION_RELEASED)
                        GUI_EndDialog(hWin, 0);
                    break;
                case GUI_ID_CANCEL:
                    if(NCode==WM_NOTIFICATION_RELEASED)
                        GUI_EndDialog(hWin, 0);
                    break;

                default:
                    break;
            }
            break;
        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_Menu:
                    printf("[%s : %d] GUI_KEY_Menu \r\n",__FUNCTION__,__LINE__);
                    GUI_EndDialog(hWin, 1);
                    break;
                case GUI_KEY_PlayPause:
                    printf("[%s : %d] GUI_KEY_PlayPause \r\n",__FUNCTION__,__LINE__);
                    break;
                case GUI_KEY_Direction_Up:
                    printf("[%s : %d] GUI_KEY_Direction_Up \r\n",__FUNCTION__,__LINE__);
				case GUI_KEY_Direction_Down:
                    printf("[%s : %d] GUI_KEY_Direction_Down \r\n",__FUNCTION__,__LINE__);
                    break;
                case GUI_KEY_Vol_Dec://音量减小
                    break;
                case GUI_KEY_Vol_Plus://音量增加
                    break;
                case GUI_KEY_LockScreen://锁屏
                    break;
                case GUI_KEY_UnLock://解锁
                    break;
				default:
					break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
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
WM_HWIN App_About(WM_HWIN hWin)
{
#if 1
	hWin_About = GUI_CreateDialogBox(_aDialogCreate_About,
	                                GUI_COUNTOF(_aDialogCreate_About),
                                  	_cbWinCallBack_About,
	                                hWin,
	                                0,
	                                0);
#else
	hWin_About = WM_CreateWindowAsChild(0, 0,
                                  LCD_GetXSize(),
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbWinCallBack, 0);
#endif
    return hWin_About;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
