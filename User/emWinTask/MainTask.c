/*
*********************************************************************************************************
*
*	模块名称 : 智能家居界面设计
*	文件名称 : MainTask.c
*	版    本 : V3.0
*	说    明 : 实验内容
*              1. 本实例有两个个值得大家学习的地方:
*                 (1). ICONVIEW控件的使用。
*                 (2). 所有界面支持触摸也支持按键操作。
*              2. 按键K2实现对话框的删除。
*                 按键K3实现ICONVIEW的聚焦。
*				  摇杆UP键实现ICONVIEW选项的上移。
*				  摇杆DOWN键实现ICONVIEW选项的下移。
*				  摇杆LIGHT键实现ICONVIEW选项的左移。
*				  摇杆RIGHT键实现ICONVIEW选项的右移。
*				  摇杆OK键实现对话框的创建。
*
*	修改记录 :
*		版本号    日期          作者          说明
*		V1.0    2014-06-21    Eric2013        首发
*             								 实际测试中发现三个问题，值得大家注意：
*                							 (1). 使用ICONVIEW的WM_NOTIFICATION_CLICKED消息会使得打开一次对话框后，再将其关闭。
*                      							  再次操作ICONVIEW的时候需要点击两次才有效果。
*                 							 (2). 显示游标后，设置显示图片为565格式，GUI_DrawBitmap(&bmButtonLine, 0, 272-44);无法显示了。
*                      							  显示游标后，设置显示图片为32位ARGB格式，GUI_DrawBitmap(&bmButtonLine, 0, 272-44);可以显示。
*                 							 (3). 使能hWinMain窗口使用内存设备后（WM_SetCreateFlags(WM_CF_MEMDEV);），窗口上建立了ICONVIEW.
*                      							  4.3寸可以正常显示，7寸和5寸屏不能，使用外部SRAM作为emWin动态内存或者修改LCDConf.C中的5寸
*                      							  和7寸的输出都是480*272解决，使用原始尺寸将造成图标不显示，向此窗口发送消息造成死机。
*	    V2.0    2015-04-15    Eric2013       1. 升级固件库到V1.5.0
*                                            2. 升级BSP板级支持包
*                                            3. 升级fatfs到0.11
*                                            4. 升级STemWin到5.26
*                                            5. 更改为新的四点触摸校准算法，并重查编排触摸检测任务
*                                            6. 添加7寸800*480分辨率电容屏支持，添加3.5寸480*320的ILI9488支持。
*                                            7. 重新编排uCOS-III的任务。
*                                            8. V2.0版本使用外部2MB SRAM作为emWin动态内存，V1.0版本存在的问题已经得到解决。
*	    V3.0    2015-12-18  Eric2013         1. 升级BSP板级支持包
*                                            2. 升级STemWin到5.28
*                                            3. 添加4.3寸和5寸电容屏支持。
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
*                                       引用外部定义
*********************************************************************************************************
*/
extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;

extern GUI_CONST_STORAGE GUI_BITMAP bmTopLine;
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
extern GUI_CONST_STORAGE GUI_BITMAP bmButtonLine;

extern WM_HWIN  hWin_Bluetooth;
extern WM_HWIN  hWin_Music;
extern WM_HWIN  hWin_Sleep;
extern WM_HWIN  hWin_Language;
extern WM_HWIN  hWin_DateTime;
extern WM_HWIN  hWin_About;

extern void App_Bluetooth(WM_HWIN hWin);
extern void App_Music(WM_HWIN hWin);
extern void App_Sleep(WM_HWIN hWin);
extern void App_Language(WM_HWIN hWin);
extern void App_DateTime(WM_HWIN hWin);
extern void App_About(WM_HWIN hWin);

/*
*********************************************************************************************************
*                                      变量和数组
*********************************************************************************************************
*/
static GUI_MEMDEV_Handle   hMempic;

WM_HWIN  hWinInfo;   /* 通过ICONVIEW所打开窗口的句柄 */
WM_HWIN  hWinICON;   /* ICONVIEW控件句柄 */
WM_HWIN  hWinMain;
WM_HWIN  hWinDesktop;   /* 主窗口句柄, ICONVIEW控件建立在这个窗口上面 */

uint8_t	s_ucSelIconIndex = 0;	/* 选择的ICON，默认不选择任何 */
uint8_t s_ucSelDesktopIndex = 0;

/* 实际的测试需要是图像宽度的4倍即可，切记(也就是保证每个像素如果是32
位数据的情况) */
static char _acBuffer[240 * 48];

/* 用于桌面ICONVIEW图标的创建 */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

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
							 h, 	/* 小工具的垂直尺寸（单位：像素）*/
	                         hParent, 				        /* 父窗口的句柄。如果为0，则新小工具将成为桌面（顶级窗口）的子窗口 */
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
    //ICONVIEW_SetTextColor(hIcon, ICONVIEW_CI_SEL, GUI_RED);

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
*******************************************************************************
**************************
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
*******************************************************************************
**************************
*/
static WM_HWIN _CreateButton(WM_HWIN hParent, const char* pText, int Id, int x
, int y, int w, int h, unsigned TextId)
{
	WM_HWIN hButton;
	hButton = BUTTON_CreateEx(x, y, w, h, hParent, WM_CF_SHOW, 0, Id);

	/* 获取当前创建按钮要显示的文本 */

	BUTTON_SetText      (hButton,    pText);

	/* 设置接收输入焦点的能力 */
	//BUTTON_SetFocussable(hButton,    1);


	return hButton;
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
	WM_MESSAGE pMsgInfo;
	int NCode, Id;

	switch (pMsg->MsgId)
	{
		case WM_INIT_DIALOG:
			break;

        case WM_CREATE:
            WM_SetFocus(pMsg->hWin);
            break;

        case WM_PRE_PAINT:
            GUI_MULTIBUF_Begin();
            break;

        case WM_PAINT:
            WM_SetFocus(pMsg->hWin);
            break;

        case WM_POST_PAINT:
            GUI_MULTIBUF_End();
            break;

		case WM_TIMER:
			/* 显示时间和日期 */
			//Caculate_RTC(pMsg);
			/* 重启定时器 */
			//WM_RestartTimer(pMsg->Data.v, 1000);
			break;


        /*  设置ICON的聚焦 */
        case MSG_SetICONFocus:
            WM_SetFocus(hWinICON);
            break;

		case MSG_Delect:
			WM_DeleteWindow(hWinInfo); /* 删除通过ICON创建的对话框 */
			break;

		case WM_NOTIFY_PARENT:
			Id    = WM_GetId(pMsg->hWinSrc);      /* Id of widget */
			NCode = pMsg->Data.v;                 /* Notification code */

            if((Id == GUI_ID_ICONVIEW0) && (NCode == WM_NOTIFICATION_RELEASED))
            {
                //WM_SetFocus(WM_HBKWIN);
				s_ucSelIconIndex  = ICONVIEW_GetSel(pMsg->hWinSrc);
                printf("s_ucSelIconIndex = %d \n",s_ucSelIconIndex);
                _appModules[s_ucSelIconIndex](WM_HBKWIN);
            }
			break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_Menu:
                    if(s_ucSelDesktopIndex == 0) s_ucSelDesktopIndex = 1;
                    else s_ucSelDesktopIndex = 0;
                    WM_MoveTo(pMsg->hWin,
                               s_ucSelDesktopIndex * (-LCD_GetXSize()), 0);
                    //if(s_ucSelDesktopIndex == 0) WM_DeleteWindow(hWinInfo);
                    break;
                case GUI_KEY_PlayPause:
                    if(s_ucSelDesktopIndex == 1)
                    {
                        pMsgInfo.MsgId = WM_NOTIFY_PARENT;
            			pMsgInfo.hWinSrc = hWinICON;
            			pMsgInfo.Data.v = WM_NOTIFICATION_RELEASED;
            			WM_SendMessage(pMsg->hWin, &pMsgInfo);
                    }
                    break;
                case GUI_KEY_Direction_Up:
                case GUI_KEY_Direction_Down:
                case GUI_KEY_Direction_Right:
                case GUI_KEY_Direction_Left:
                    if(s_ucSelDesktopIndex)  WM_SetFocus(hWinICON);
                    break;

                case GUI_KEY_PGDOWN://音量减小
                    break;

                case GUI_KEY_PGUP://音量增加
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
static void _cbBkWindow(WM_MESSAGE * pMsg)
{
	switch (pMsg->MsgId)
	{
		/* 重绘消息*/
		case WM_PAINT:
            GUI_MEMDEV_Select(hMempic);
            _ShowBMPEx("bg.bmp");
        	GUI_MEMDEV_Select(0);
            GUI_MEMDEV_WriteAt(hMempic, 0, 0);
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
	/* 初始化并创建对话框 */
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
	WM_SetCreateFlags(WM_CF_MEMDEV | WM_CF_MEMDEV_ON_REDRAW);

    /* 使能桌面窗口也使用内存设备 */
    //WM_EnableMemdev(WM_HBKWIN);
#endif
    WM_MOTION_Enable(1);    /* 使能滑动 */
    WM_MOTION_SetDefaultPeriod(50);

	/* 使能UTF-8解码用于汉字显示 */
	GUI_UC_SetEncodeUTF8();

    hMempic = GUI_MEMDEV_CreateFixed(0, 0,
	                                 LCD_GetXSize(),
	                                 LCD_GetYSize(),
									 GUI_MEMDEV_HASTRANS,
									 GUI_MEMDEV_APILIST_16,
									 GUICC_M565);
	GUI_MEMDEV_Select(hMempic);
    _ShowBMPEx("bg.bmp");
	GUI_MEMDEV_Select(0);

    WM_SetCallback(WM_HBKWIN, _cbBkWindow);

    hWinMain = WM_CreateWindowAsChild(0, 0,
                                  LCD_GetXSize() * 2,
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbDesktopDisplayProc, 0);

    _CreateButton(hWinMain, "Test Button",
                    GUI_ID_BUTTON0,
                    (FRAME_WIDTH >> 1) - 100 , 80, 200,  50, 0);

    hWinICON = _CreateICONVIEW(hWinMain,
                             _aBitmapItem, GUI_COUNTOF(_aBitmapItem),
                             GUI_ID_ICONVIEW0,
                             LCD_GetXSize() + 22, 30, 220, 280);

    WM_CreateTimer(WM_GetClientWindow(hWinMain), /* 接受信息的窗口的句柄 */
                       ID_TimerTime,                 /* 用户定义的Id。如果不对同一窗口使用多个定时器，此值可以设置为零。 */
                       400,                         /* 周期，此周期过后指定窗口应收到消息*/
                       0);                           /* 留待将来使用，应为0 */

	while(1)
	{
		GUI_Delay(50);
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
