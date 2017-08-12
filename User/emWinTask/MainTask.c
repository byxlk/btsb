/*
*********************************************************************************************************
*
*	模块名称 : 智能家居界面设计
*	文件名称 : MainTask.c
*	版    本 : V3.0
*	说    明 : 实验内容
*              1. 本实例有两个个值得大家学习的地方:
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"
#include "bsp.h"

/*
*********************************************************************************************************
*                                     宏定义
*********************************************************************************************************
*/
#define ID_WINDOW_0    (GUI_ID_USER + 0x00)
#define ID_FRAMEWIN_0  (GUI_ID_USER + 0x01)
#define ID_GRAPH_0     (GUI_ID_USER + 0x02)
#define ID_TEXT_0      (GUI_ID_USER + 0x03)
#define ID_TEXT_1      (GUI_ID_USER + 0x04)
#define ID_TEXT_2      (GUI_ID_USER + 0x05)
#define ID_TEXT_3      (GUI_ID_USER + 0x06)
#define ID_TEXT_4      (GUI_ID_USER + 0x07)
#define ID_TEXT_5      (GUI_ID_USER + 0x08)
#define ID_TEXT_6      (GUI_ID_USER + 0x09)
#define ID_TEXT_7      (GUI_ID_USER + 0x0A)
#define ID_TEXT_8      (GUI_ID_USER + 0x0B)
#define ID_TEXT_9      (GUI_ID_USER + 0x0C)
#define ID_TEXT_10     (GUI_ID_USER + 0x0D)

#define MAIN_BORDER               0
#define MAIN_TITLE_HEIGHT        0

#define FRAME_BKCOLOR             0xD0D0D0
#define FRAME_TEXTCOLOR           0x000000
#define FRAME_FONT                (&GUI_FontYahei)
#define FRAME_EFFECT              (&WIDGET_Effect_Simple)
#define FRAME_BORDER              FRAME_EFFECT->EffectSize
#define FRAME_WIDTH               (LCD_GetXSize())
#define FRAME_HEIGHT              (LCD_GetYSize())


#define ID_TimerTime    1
#define RECOMMENDED_MEMORY (1024L * 24)

/*
*********************************************************************************************************
*                                       引用外部定义
*********************************************************************************************************
*/
extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;


extern WM_HWIN CreateWindow_HomePage(WM_HWIN hParent);

/*
*********************************************************************************************************
*                                      变量和数组
*********************************************************************************************************
*/
static GUI_MEMDEV_Handle   hMempic;
WM_HWIN  hWin_HomePage;
WM_HWIN  hWin_IconMenu;

//WM_HWIN  hWinInfo;   /* 通过ICONVIEW所打开窗口的句柄 */
//WM_HWIN  hWinICON;   /* ICONVIEW控件句柄 */
//WM_HWIN  hWinDesktop;   /* 主窗口句柄, ICONVIEW控件建立在这个窗口上面 */

uint8_t s_ucSelDesktopIndex = 0;
//uint8_t s_ucEnteryAppFlag = 0;

/* 实际的测试需要是图像宽度的4倍即可，切记(也就是保证每个像素如果是32位数据的情况) */
static char _acBuffer[240 * 32];

RTC_TimeTypeDef  RTC_TimeStructure;
RTC_DateTypeDef  RTC_DateStructure;


/*
*******************************************************************************
*	函 数 名: _GetData
*	功能说明: 被函数GUI_BMP_DrawEx()调用
*	形    参：p             FIL类型数据
*             NumBytesReq   请求读取的字节数
*             ppData        数据指针
*             Off           如果Off = 1
，那么将重新从其实位置读取
*	返 回 值: 返回读取的字节数
*******************************************************************************
*/
static int _pfGetData(void * p, const U8 ** ppData,
                        unsigned NumBytesReq,
                        U32 Off)
{
//	U32 i;
	static int FileAddress = 0;
	UINT NumBytesRead = 0;
	FIL *PicFile;

	PicFile = (FIL *)p;

	/*
	* 检测缓存大小
	*/
	if (NumBytesReq > sizeof(_acBuffer)) {
	    NumBytesReq = sizeof(_acBuffer);
	}

	/*
	* 设置读取位置
	*/
	if(Off == 1) FileAddress = 0;
	else FileAddress = Off;
	result =f_lseek(PicFile, FileAddress);

	/*
	* 读取数据到缓存, 由于FatFS+官方SD卡的方案存在DMA传输上的4字节对齐问题，
	* 这里以小于等于一个SD卡扇区大小来操作，超过512字节会出错。
	*/
	//for(i = 0; i < NumBytesReq / 512; i++)
	//{
	//	result = f_read(PicFile, &_acBuffer[512*i], 512, &bw);
	//	NumBytesRead += bw;
	//}

	result = f_read(PicFile, _acBuffer, NumBytesReq, &NumBytesRead);
	//NumBytesRead += bw;

	/*
	* 让指针ppData指向读取的函数
	*/
	*ppData = (const U8 *)&_acBuffer[0];

	/*
	* 返回读取的字节数
	*/
	return NumBytesRead;
}

/*
*******************************************************************************
*	函 数 名: _ShowBMPEx
*	功能说明: 显示BMP图片
*	形    参: sFilename 要显示图片的名字
*	返 回 值: 无
*******************************************************************************
*/
static void _ShowBMPEx(const char * sFilename)
{
    //OS_ERR      	err;

    /* 挂载文件系统 */
    MountFS(&fs, 0);

    /* 打开文件 */
	result = f_open(&file, sFilename,
	                 FA_OPEN_EXISTING | FA_READ | FA_OPEN_ALWAYS);
	if (result != FR_OK)
	{
		return;
	}

//	XSize = GUI_BMP_GetXSizeEx(_GetData, &file);
//	YSize = GUI_BMP_GetYSizeEx(_GetData, &file);

	GUI_BMP_DrawEx(_pfGetData, &file, 0, 0);

    f_close(&file);

    /* 卸载文件系统 */
    MountFS(NULL, 0);
}


/*
*********************************************************************************************************
*	函 数 名: Caculate_RTC
*	功能说明: 显示RTC时间
*	形    参：pMsg 指针参数
*	返 回 值: 无
*********************************************************************************************************
*/
 static void Caculate_RTC(WM_MESSAGE * pMsg)
{
	  char buf[30];
	  WM_HWIN hWin = pMsg->hWin;

	  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

	  sprintf(buf,
	          "%0.2d:%0.2d",
			  RTC_TimeStructure.RTC_Hours,
			  RTC_TimeStructure.RTC_Seconds);
	 TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_0), buf);

	  sprintf(buf,
	          "20%0.2d/%0.2d/%0.2d",
			  RTC_DateStructure.RTC_Year,
			  RTC_DateStructure.RTC_Month,
			  RTC_DateStructure.RTC_Date);
	  TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_7), buf);
}


static void GUI_KeyEvent_Response(WM_KEY_INFO *KeyInfo)
{
    switch (KeyInfo->Key)
    {
        case GUI_KEY_Menu:
            _LOGD("GUI_KEY_Menu \r\n");
            if(s_ucSelDesktopIndex == 0) {
                s_ucSelDesktopIndex = 1;
                WM_HideWindow(hWin_HomePage);
                WM_ShowWindow(hWin_IconMenu);
                //if(hWinICON == HBWIN_NULL)
                //    hWinICON = CreateWindow_IconMenu(WM_HBKWIN);
                //if(hWinMain != HBWIN_NULL) {
                //    WM_DeleteWindow(hWinMain);
                //    hWinMain = HBWIN_NULL;
                //}
            } else {
                s_ucSelDesktopIndex = 0;
                WM_HideWindow(hWin_IconMenu);
                WM_ShowWindow(hWin_HomePage);
                //if(hWinICON != HBWIN_NULL) {
                //    WM_DeleteWindow(hWinICON);
                //    hWinICON = HBWIN_NULL;
                //}
                //if(hWinMain == HBWIN_NULL)
                //    hWinMain = CreateWindow_HomePage(WM_HBKWIN);
            }
            break;
        case GUI_KEY_PlayPause_Long:
            _LOGD("GUI_KEY_PlayPause_Long \r\n");
            break;
        case GUI_KEY_Direction_Up:
            _LOGD("GUI_KEY_Direction_Up \r\n");
        case GUI_KEY_Direction_Down:
            _LOGD("GUI_KEY_Direction_Down \r\n");
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
}


/*
*******************************************************************************
*	函 数 名: _cbBkWindow
*	功能说明: 桌面窗口的回调函数,
这里主要是绘制背景窗口和界面切换时，切换标志的绘制
*	形    参: pMsg  WM_MESSAGE类型指针变量
*	返 回 值: 无
*******************************************************************************
*/
static void _cbBackGround(WM_MESSAGE * pMsg)  //桌面背景的回调函数
{
    //_LOGD("MsgId = %d\r\n",pMsg->MsgId);
	switch (pMsg->MsgId)
	{
	    case WM_CREATE:
            WM_SetFocus(pMsg->hWin);
            break;
	    case WM_TIMER:
            WM_InvalidateWindow(pMsg->hWin);
            WM_RestartTimer(pMsg->Data.v, 100);
            break;
        case WM_PAINT:/* 重绘消息*/
            //_LOGD("WM_PAINT(%d)\r\n",pMsg->MsgId);
            //GUI_MEMDEV_Select(hMempic);
            _ShowBMPEx("bg.bmp");
        	//GUI_MEMDEV_Select(0);
            //GUI_MEMDEV_WriteAt(hMempic, 0, 0);
			break;
        case WM_PRE_PAINT:
            GUI_MULTIBUF_Begin();
            break;
        case WM_POST_PAINT:
            GUI_MULTIBUF_End();
            break;
        case WM_KEY:
            GUI_KeyEvent_Response(((WM_KEY_INFO *)(pMsg->Data.p)));
            break;

		default:
			WM_DefaultProc(pMsg);
			break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MainTask
*	功能说明: 主函数
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void MainTask(void)
{
    WM_HWIN    hWinMain;

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
#if 0
    GUI_Init(); /* 初始化并创建对话框 */
    WM_MULTIBUF_Enable(1);
#else
	WM_SetCreateFlags(WM_CF_MEMDEV); /* 创建使用内存设备 */
    GUI_Init(); /* 初始化并创建对话框 */
    //WM_EnableMemdev(WM_HBKWIN); /* 使能桌面窗口也使用内存设备 */
#endif
    //WM_MOTION_Enable(1);    /* 使能滑动 */
    //WM_MOTION_SetDefaultPeriod(50);

	/* 使能UTF-8解码用于汉字显示 */
	GUI_UC_SetEncodeUTF8();

    _LOGD("FreeBytes = %d\r\n",GUI_ALLOC_GetNumFreeBytes());
    if (GUI_ALLOC_GetNumFreeBytes() < RECOMMENDED_MEMORY) {
        _LOGE("Not enough memory available.Recomended Memory = 0x%x\r\n",RECOMMENDED_MEMORY);
        return;
    }

	FRAMEWIN_SetDefaultFont(&GUI_FontComic18B_ASCII);
    FRAMEWIN_SetDefaultTextAlign(GUI_TA_CENTER);
    TEXT_SetDefaultFont(&GUI_FontComic18B_ASCII);
    TEXT_SetDefaultTextColor(GUI_WHITE);
    FRAMEWIN_SetDefaultBarColor(1, GUI_MAGENTA);

	//hMempic = GUI_MEMDEV_CreateFixed(0, 0,
    //                             LCD_GetXSize(),
    //                             LCD_GetYSize(),
    //                             GUI_MEMDEV_HASTRANS,
    //                             GUI_MEMDEV_APILIST_16,
    //                             GUICC_M565);
    //GUI_MEMDEV_Select(hMempic);
    //_ShowBMPEx("bg.bmp");
    //GUI_MEMDEV_Select(0);

    WM_SetDesktopColor(GUI_BLACK);
    hWinMain = WM_CreateWindow(0, 0, LCD_GetXSize(), LCD_GetYSize(),
                                 WM_CF_SHOW, _cbBackGround, 0);
    WM_CreateTimer(hWinMain, 0, 10, 0);

#if 1
    hWin_HomePage = CreateWindow_HomePage(0);
    //WM_CreateTimer(hWin_HomePage, 0, 10, 0);

    hWin_IconMenu = CreateWindow_IconMenu(0);
    WM_HideWindow(hWin_IconMenu);
    WM_SetFocus(hWinMain);

#else
    hWinMain = WM_CreateWindowAsChild(0, 0,
                                  LCD_GetXSize()*2,
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbDesktopDisplayProc, 0);

    hWinICON = _CreateICONVIEW(hWinMain,
                             _aBitmapItem, GUI_COUNTOF(_aBitmapItem),
                             GUI_ID_ICONVIEW0,
                             LCD_GetXSize()+22, 30, 210, 280);

    WM_CreateTimer(WM_GetClientWindow(hWinMain), /* 接受信息的窗口的句柄 */
                       ID_TimerTime,                 /* 用户定义的Id。如果不对同一窗口使用多个定时器，此值可以设置为零。 */
                       400,                         /* 周期，此周期过后指定窗口应收到消息*/
                       0);                           /* 留待将来使用，应为0 */
#endif
	while(1)
	{
		GUI_Delay(100);
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
