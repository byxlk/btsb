#include "DIALOG.h"

#define ID_FRAMEWIN_0	(GUI_ID_USER + 0x00)
#define ID_BUTTON_0		(GUI_ID_USER + 0x01)
#define ID_BUTTON_1		(GUI_ID_USER + 0x02)

static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {	//控件结构体数组
	{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 240, 320, FRAMEWIN_CF_MOVEABLE, 0x64, 0 },
	{ BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 5, 180, 60, 25, 0, 0x0, 0 },
	{ BUTTON_CreateIndirect, "Button", ID_BUTTON_1, 75, 180, 60, 25, 0, 0x0, 0 },
};

extern WM_HWIN hPage[4];

static void _cbDialog(WM_MESSAGE * pMsg) {
	WM_HWIN hItem;
	int		 NCode;
	int		 Id;

	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG://初始化消息,创建窗口/控件时有效,比如在这里设置一些控件的初始参数
		hItem = pMsg->hWin;
		FRAMEWIN_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
		FRAMEWIN_SetText(hItem, "Page 0");
		FRAMEWIN_SetTitleHeight(hItem, 18);
		FRAMEWIN_SetFont(hItem, GUI_FONT_16B_1);
		FRAMEWIN_SetTextColor(hItem, (0x008000FF));
		FRAMEWIN_AddCloseButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_AddMaxButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_AddMinButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_SetClientColor(pMsg->hWin, GUI_GREEN);

		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
		BUTTON_SetText(hItem, "Return");

		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
		BUTTON_SetText(hItem, "Next");
		break;
	case WM_PAINT:	//窗口重绘消息,这个比较难说明白,反正在Framewin或Window窗体之中我们一般是用控
					//件,如果要在Framewin或Window窗体之中显示文字或绘制直线、矩形、圆等在这里实现
		GUI_SetColor(GUI_BLUE);								//设置前景颜色
		GUI_SetFont((GUI_FONT *)&GUI_Font16_ASCII);			//设置当前字库
		GUI_SetTextMode(GUI_TEXTMODE_TRANS);				//设置文字透明显示
		GUI_DispStringAt("neqee.com", 3, 3);				//重绘显示文字
		break;
	case WM_NOTIFY_PARENT://操作触发消息处理(操作屏幕程序会跑到这里),比如点击按键、点击编辑框(任何的操作)等等......
		Id		= WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
		switch(Id) {
		case ID_BUTTON_0:									//ID为ID_BUTTON_0的按键被点击
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED:					//按下动作消息
				break;
			case WM_NOTIFICATION_RELEASED:					//弹起动作消息
				break;
			}
			break;
		case ID_BUTTON_1:									//ID为ID_BUTTON_1的按键被点击
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED:					//按下动作消息
				break;
			case WM_NOTIFICATION_RELEASED:					//弹起动作消息
				WM_HideWindow(hPage[0]);					//隐藏Page 0
				WM_ShowWindow(hPage[1]);					//显示Page 1
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

static void _cbBackGround(WM_MESSAGE* pMsg) {	//桌面背景的回调函数
	switch (pMsg->MsgId)
	{
	case WM_PAINT:	//窗口重绘消息,这个比较难说明白,反正在Framewin或Window窗体之中我们一般是用控
					//件,如果要在Framewin或Window窗体之中显示文字或绘制直线、矩形、圆等在这里实现
		GUI_SetBkColor(GUI_RED);				//设置背景颜色
		GUI_Clear();							//清屏
		break;
	default: WM_DefaultProc(pMsg); break;
	}
}

WM_HWIN CreateFramewin0(WM_HWIN hParent);
WM_HWIN CreateFramewin0(WM_HWIN hParent) {
	WM_HWIN hWin;
	WM_SetCallback(WM_HBKWIN, _cbBackGround);	//设置桌面背景的回调函数
	hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, hParent, 0, 0);
	return hWin;
}
