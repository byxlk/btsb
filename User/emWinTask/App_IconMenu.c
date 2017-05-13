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
*				                      宏定义
*********************************************************************************************************
*/
#define ID_FRAMEWIN_0  (GUI_ID_USER + 0x04)
#define ID_ICONVIEW_0  (GUI_ID_USER + 0x06)
#define ID_TEXT_0  (GUI_ID_USER + 0x07)

extern GUI_CONST_STORAGE GUI_BITMAP bma;
extern GUI_CONST_STORAGE GUI_BITMAP bmb;
extern GUI_CONST_STORAGE GUI_BITMAP bmc;
extern GUI_CONST_STORAGE GUI_BITMAP bmd;
extern GUI_CONST_STORAGE GUI_BITMAP bme;
extern GUI_CONST_STORAGE GUI_BITMAP bmf;

extern WM_HWIN  hWin_Bluetooth;
extern WM_HWIN  hWin_Music;
extern WM_HWIN  hWin_Sleep;
extern WM_HWIN  hWin_Language;
extern WM_HWIN  hWin_DateTime;
extern WM_HWIN  hWin_About;
extern WM_HWIN  hWin_HomePage;

extern void App_Bluetooth(WM_HWIN hWin);
extern void App_Music(WM_HWIN hWin);
extern void App_Sleep(WM_HWIN hWin);
extern void App_Language(WM_HWIN hWin);
extern void App_DateTime(WM_HWIN hWin);
extern void App_About(WM_HWIN hWin);
extern void App_HomePage(WM_HWIN hWin);


/* 用于桌面ICONVIEW图标的创建 */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

#if 0
/* 用于桌面ICONVIEW图标的创建 */
static const BITMAP_ITEM _aBitmapItem[] =
{
	{&bma,    "BT"},
	{&bmb,    "Music"},
	{&bmc,    "Sleep"},
	{&bmd,    "Language"},
	{&bme,    "Date"},
	{&bmf,    "About"},
};

/*
*******************************************************************************
*                                  应用程序入口函数
*******************************************************************************
*/
static void (* _appModules[])( WM_HWIN hWin) =
{
	App_Bluetooth,
	App_Music,
	App_Sleep,
	App_Language,
	App_DateTime,
	App_About,
};
#endif

static const GUI_WIDGET_CREATE_INFO _aDialogCreate_IconMenu[] = {
	{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 320, 240, 0, 0x64, 0 },
	{ ICONVIEW_CreateIndirect, "Iconview", ID_ICONVIEW_0, 15, 50, 280, 140, 0,0x00400040/*xSizeItem-ySizeItem*/, 0 },
	{ TEXT_CreateIndirect, "Text", ID_TEXT_0, 15, 10, 280, 20, 0, 0x64, 0 },
};
#if 0
/*
*******************************************************************************
*	函 数 名: _CreateICONVIEW
*	功能说明: 创建ICONVIEW
*	形    参：hParent   父窗口
*             pBm       ICONVIEW上的位图
*             BitmapNum ICONVIEW上图标个数
*             x         x轴坐标
*             y         y轴坐标
*             w         ICONVIEW宽
*             h         ICONVIEW高
*	返 回 值: 无
*******************************************************************************
*/
static WM_HWIN _CreateICONVIEW(WM_HWIN hParent,
                                          const BITMAP_ITEM *pBm,
                                          int BitmapNum, int Id,
                                          int x, int y, int w, int h)
{
	WM_HWIN hIcon;
	int i;

    /*在指定位置创建指定尺寸的ICONVIEW 小工具*/
	hIcon = ICONVIEW_CreateEx(x, 				/* 小工具的最左像素（在父坐标中）*/
						     y, 					/* 小工具的最上像素（在父坐标中）*/
							 w,    				/* 小工具的水平尺寸（单位：像素）*/
							 h, 	                /* 小工具的垂直尺寸（单位：像素）*/
	                         hParent, 			/* 父窗口的句柄。如果为0，则新小工具将成为桌面（顶级窗口）的子窗口 */
							 WM_CF_SHOW | WM_CF_HASTRANS,       /*窗口创建标记。为使小工具立即可见，通常使用 WM_CF_SHOW */
	                         0,//ICONVIEW_CF_AUTOSCROLLBAR_V, 	/* 默认是0，如果不够现实可设置增减垂直滚动条 */
							 Id, 			        /* 小工具的窗口ID */
							 82, 				    /* 图标的水平尺寸 */
							 80);/* 图标的垂直尺寸 */

	/* 向ICONVIEW 小工具添加新图标 */
	for (i = 0; i < BitmapNum; i++)
	{
		ICONVIEW_AddBitmapItem(hIcon, pBm[i].pBitmap, pBm[i].pText);
	}

	/* 设置小工具的背景色 32 位颜色值的前8 位可用于alpha混合处理效果*/
	ICONVIEW_SetBkColor(hIcon, ICONVIEW_CI_SEL, GUI_DARKBLUE | 0x80000000);
    //ICONVIEW_SetBkColor(hIcon, ICONVIEW_CI_BK, GUI_WHITE | 0x00000000);
    ICONVIEW_SetTextColor(hIcon, ICONVIEW_CI_BK, GUI_BLACK);
    ICONVIEW_SetTextColor(hIcon, ICONVIEW_CI_SEL, GUI_BLACK);

	/* 设置字体 */
	ICONVIEW_SetFont(hIcon, &GUI_FontYahei);

	/* 设置图标在x 或y 方向上的间距。*/
	ICONVIEW_SetSpace(hIcon, GUI_COORD_Y, 12);
    ICONVIEW_SetSpace(hIcon, GUI_COORD_X, 20);

    //ICONVIEW_SetFrame(hIcon, GUI_COORD_X, 0);
    //ICONVIEW_SetFrame(hIcon, GUI_COORD_Y, 0);

	/* 设置对齐方式 在5.22版本中最新加入的 */
	ICONVIEW_SetIconAlign(hIcon, ICONVIEW_IA_HCENTER | ICONVIEW_IA_TOP);

	return hIcon;
}

/*
*********************************************************************************************************
*	函 数 名: _cbDialogInfo
*	功能说明: 主窗口的回调函数
*	形    参：pMsg   参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void _cbDesktopDisplayProc(WM_MESSAGE * pMsg)
{
    int NCode, Id;
	WM_MESSAGE pMsgInfo;
    GUI_ALPHA_STATE AlphaState;

    //printf("[%s : %d] MsgId = %d\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);

	switch (pMsg->MsgId)
	{
        case WM_CREATE:
            printf("[%s : %d] WM_CREATE(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            s_ucSelDesktopIndex = 0;
            s_ucSelIconIndex = 0;
            s_ucEnteryAppFlag = 0;

            App_HomePage(pMsg->hWin);
            hWinICON = _CreateICONVIEW(pMsg->hWin,
                             _aBitmapItem, GUI_COUNTOF(_aBitmapItem),
                             GUI_ID_ICONVIEW0,
                             LCD_GetXSize()+22, 30, 210, 280);
            //WM_HideWindow(hWinICON);
            WM_SetFocus(pMsg->hWin);
            break;

        case WM_PRE_PAINT:
            //printf("[%s : %d] WM_PRE_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            GUI_MULTIBUF_Begin();
            break;

        case WM_PAINT:
            printf("[%s : %d] WM_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            if(s_ucSelDesktopIndex == 0) {
                //WM_HideWindow(hWinICON);
                //WM_ShowWindow(hWin_HomePage);
                //App_HomePage(pMsg->hWin);
            //    WM_DeleteWindow(hWinICON);
            //    WM_SetFocus(pMsg->hWin);
            } else {
                //WM_HideWindow(hWin_HomePage);
                //WM_ShowWindow(hWinICON);
            //    WM_DeleteWindow(hWin_HomePage);
            //    WM_SetFocus(hWinICON);
                //WM_DeleteWindow(hWinICON);
            }
            break;
        case WM_POST_PAINT:
            //printf("[%s : %d] WM_POST_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            GUI_MULTIBUF_End();
            break;

		case WM_TIMER:
			/* 显示时间和日期 */
			//Caculate_RTC(pMsg);
			/* 重启定时器 */
			//WM_RestartTimer(pMsg->Data.v, 1000);
			break;

		case WM_NOTIFY_PARENT:
            printf("[%s : %d] WM_NOTIFY_PARENT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
			Id    = WM_GetId(pMsg->hWinSrc);
			NCode = pMsg->Data.v;
			switch (Id)
			{
				/* 第一个界面上的图标 */
				case GUI_ID_ICONVIEW0:
					switch (NCode)
					{
						/* ICON控件点击消息 */
						case WM_NOTIFICATION_CLICKED:
							break;

                        case WM_NOTIFICATION_SEL_CHANGED:
                            break;

						/* ICON控件释放消息 */
						case WM_NOTIFICATION_RELEASED:
                            printf("WM_NOTIFICATION_RELEASED\r\n");
							break;
                        case WM_NOTIFICATION_CHILD_DELETED:
                            printf("WM_NOTIFICATION_CHILD_DELETED\r\n");
                            break;
					}
					break;
			}
			break;

        case MSG_SetICONFocus:
            hWinICON = _CreateICONVIEW(pMsg->hWin,
                             _aBitmapItem, GUI_COUNTOF(_aBitmapItem),
                             GUI_ID_ICONVIEW0,
                             LCD_GetXSize()+22, 30, 210, 280);
            WM_SetFocus(hWinICON);
            ICONVIEW_SetSel(hWinICON, s_ucSelIconIndex);
            printf("[lsl]s_ucSelIconIndex = %d \r\n",s_ucSelIconIndex);
            break;
        case MSG_ShowApp:
            //s_ucSelIconIndex  = ICONVIEW_GetSel(pMsg->hWinSrc);
            WM_HideWindow(hWin_HomePage);
            WM_MoveTo(pMsg->hWin, 0, 0);
            printf("Start Entery App s_ucSelIconIndex = %d \r\n",s_ucSelIconIndex);
            //_appModules[s_ucSelIconIndex](pMsg->hWin);
            _appModules[s_ucSelIconIndex](pMsg->hWin);
            s_ucEnteryAppFlag = 1;
            printf("Quit App and show menu.\r\n");
            //WM_SetFocus(pMsg->hWin);
            break;

        case WM_KEY:
            printf("Current Focus Windows: %d \r\n",WM_GetFocussedWindow());
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_Menu:
                    WM_SetFocus(pMsg->hWin);
                    printf("[%s : %d] GUI_KEY_Menu \r\n",__FUNCTION__,__LINE__);
                    if(s_ucEnteryAppFlag == 1)
                    {
                        s_ucEnteryAppFlag = 0;
                        WM_ShowWindow(hWin_HomePage);
                        printf("[kkkkk]s_ucSelIconIndex = %d\r\n",s_ucSelIconIndex);
                        switch(s_ucSelIconIndex)
                        {
                        case 0:
                            WM_DeleteWindow(hWin_Bluetooth);
                            printf("Exit hWin_Bluetooth\r\n");
                            break;
                        case 1:
                            WM_DeleteWindow(hWin_Music);
                            printf("Exit hWin_Music\r\n");
                            break;
                        case 2:
                            WM_DeleteWindow(hWin_Sleep);
                            printf("Exit hWin_Sleep\r\n");
                            break;
                        case 3:
                            WM_DeleteWindow(hWin_Language);
                            printf("Exit hWin_Language\r\n");
                            break;
                        case 4:
                            WM_DeleteWindow(hWin_DateTime);
                            printf("Exit hWin_DateTime\r\n");
                            break;
                        case 5:
                            WM_DeleteWindow(hWin_About);
                            printf("Exit hWin_About\r\n");
                            break;
                        }
                        //default: break;
                    }
                    else
                    {
                        if(s_ucSelDesktopIndex == 0) {
                            s_ucSelDesktopIndex = 1;
                            s_ucSelIconIndex = 0;
                        }
                        else
                        {
                            //WM_SetFocus(pMsg->hWin);
                            s_ucSelDesktopIndex = 0;
                        }
                    }

                    printf("s_ucSelDesktopIndex = %d\r\n",s_ucSelDesktopIndex);
                    if(s_ucSelDesktopIndex == 0) {
                        //WM_HideWindow(hWinICON);
                        //WM_ShowWindow(hWin_HomePage);
                        //App_HomePage(pMsg->hWin);
                        //WM_DeleteWindow(hWinICON);
                        //WM_SetFocus(pMsg->hWin);
                        WM_MoveTo(hWinMain, 0, 0);
                        WM_ShowWindow(hWin_HomePage);
                    } else {
                        //WM_HideWindow(hWin_HomePage);
                        //WM_ShowWindow(hWinICON);
                        //WM_DeleteWindow(hWin_HomePage);
                        WM_MoveTo(hWinMain, -LCD_GetXSize(), 0);
                        WM_ShowWindow(hWinICON);
                    }
                    //WM_Paint(pMsg->hWin);
                    break;
                case GUI_KEY_PlayPause:
                    printf("[%s : %d] GUI_KEY_PlayPause \r\n",__FUNCTION__,__LINE__);
                    printf("s_ucSelDesktopIndex = %d \r\n",s_ucSelDesktopIndex);
                    if(s_ucSelDesktopIndex == 1)
                    {
                        pMsgInfo.MsgId = MSG_ShowApp;
            			pMsgInfo.hWinSrc = hWinICON;
            			pMsgInfo.Data.v = WM_NOTIFICATION_RELEASED;
            			WM_SendMessage(pMsg->hWin, &pMsgInfo);
                    }
                    break;
                case GUI_KEY_Direction_Up:
                    printf("[%s : %d] GUI_KEY_Direction_Up \r\n",__FUNCTION__,__LINE__);
                    if(s_ucSelDesktopIndex) {
                        switch(s_ucSelIconIndex) {
                            case 0: s_ucSelIconIndex = 5;break;
                            case 1: s_ucSelIconIndex = 4; break;
                            case 2: s_ucSelIconIndex = 0; break;
                            case 3: s_ucSelIconIndex = 1; break;
                            case 4: s_ucSelIconIndex = 2; break;
                            case 5: s_ucSelIconIndex = 3; break;
                            //default: break;
                        }
                        //WM_SetFocus(hWinICON);
                        //ICONVIEW_SetSel(hWinICON, s_ucSelIconIndex);

                        pMsgInfo.MsgId = MSG_SetICONFocus;
            			pMsgInfo.hWinSrc = hWinICON;
            			pMsgInfo.Data.v = WM_NOTIFICATION_SEL_CHANGED;
            			WM_SendMessage(pMsg->hWin, &pMsgInfo);
                    }
                    break;
                case GUI_KEY_Direction_Down:
                    printf("[%s : %d] GUI_KEY_Direction_Down \r\n",__FUNCTION__,__LINE__);
                    if(s_ucSelDesktopIndex) {
                        switch(s_ucSelIconIndex) {
                            case 0: s_ucSelIconIndex = 2; break;
                            case 1: s_ucSelIconIndex = 3; break;
                            case 2: s_ucSelIconIndex = 4; break;
                            case 3: s_ucSelIconIndex = 5; break;
                            case 4: s_ucSelIconIndex = 1; break;
                            case 5: s_ucSelIconIndex = 0; break;
                            default: break;
                        }
                        //WM_SetFocus(hWinICON);
                        //ICONVIEW_SetSel(hWinICON, s_ucSelIconIndex);
                        pMsgInfo.MsgId = MSG_SetICONFocus;
            			pMsgInfo.hWinSrc = hWinICON;
            			pMsgInfo.Data.v = WM_NOTIFICATION_SEL_CHANGED;
            			WM_SendMessage(pMsg->hWin, &pMsgInfo);
                    }
                    break;
                case GUI_KEY_Direction_Right:
                case GUI_KEY_Direction_Left:
                    //if(s_ucSelDesktopIndex)  WM_SetFocus(hWinICON);
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

#endif

static void _cbWinCallBack_IconMenu(WM_MESSAGE * pMsg) {
	WM_HWIN hItem;
	int     NCode;
	int     Id;

	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG://初始化消息,创建窗口/控件时有效,比如在这里设置一些控件的初始参数
		hItem = pMsg->hWin;
		FRAMEWIN_SetText(hItem, "neqee.com");
		FRAMEWIN_SetFont(hItem, GUI_FONT_20_1);
		FRAMEWIN_SetTextColor(hItem, (0x00FF0000));

		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetTextColor(hItem, (0x008000FF));
		TEXT_SetFont(hItem, GUI_FONT_20_1);
		TEXT_SetText(hItem, "neqee");

		hItem = WM_GetDialogItem(pMsg->hWin, ID_ICONVIEW_0);
		ICONVIEW_SetIconAlign(hItem, ICONVIEW_IA_TOP);
		ICONVIEW_SetTextColor(hItem, ICONVIEW_CI_UNSEL, GUI_BLUE);
		ICONVIEW_SetTextColor(hItem, ICONVIEW_CI_SEL, GUI_GREEN);
		ICONVIEW_SetFrame(hItem, GUI_COORD_X, 0);//设置图标到IconView边框的间距
		ICONVIEW_SetFrame(hItem, GUI_COORD_Y, 0);
		ICONVIEW_SetSpace(hItem, GUI_COORD_X, (280-64*4)/3);//设置图标和图标之间的间距
		ICONVIEW_SetSpace(hItem, GUI_COORD_Y, (140-64*2)/1);
		ICONVIEW_SetFont(hItem, GUI_FONT_16_1);
		ICONVIEW_AddBitmapItem(hItem, &bma, "neqee");//添加图标项
		ICONVIEW_AddBitmapItem(hItem, &bmb, "SOS");
		ICONVIEW_AddBitmapItem(hItem, &bmc, "Phone");
		ICONVIEW_AddBitmapItem(hItem, &bmd, "Camera");
		ICONVIEW_AddBitmapItem(hItem, &bme, "Email");
		ICONVIEW_AddBitmapItem(hItem, &bmf, "NoteBook");
		break;
	case WM_NOTIFY_PARENT://操作触发消息处理(操作屏幕程序会跑到这里),比如点击按键、点击编辑框(任何的操作)等等......
		Id    = WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
		switch(Id) {
		case ID_ICONVIEW_0:
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED://点击图标
				switch(ICONVIEW_GetSel(WM_GetDialogItem(pMsg->hWin, ID_ICONVIEW_0)))
				{
				case 0: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "neqee"); break;
				case 1: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "SOS"); break;
				case 2: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "Phone"); break;
				case 3: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "Camera"); break;
				case 4: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "Email"); break;
				case 5: TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), "NoteBook"); break;
				}
				break;
			case WM_NOTIFICATION_RELEASED:
				break;
			case WM_NOTIFICATION_MOVED_OUT:
				break;
			case WM_NOTIFICATION_SCROLL_CHANGED:
				break;
			case WM_NOTIFICATION_SEL_CHANGED:
				break;
			}
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
WM_HWIN CreateWindow_IconMenu(WM_HWIN hParent)
{
    WM_HWIN hWin;

    hParent = GUI_CreateDialogBox(_aDialogCreate_IconMenu,
                                        GUI_COUNTOF(_aDialogCreate_IconMenu),
                                        _cbWinCallBack_IconMenu,
                                        hParent,
                                        0,
                                        0);
    return hWin;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
