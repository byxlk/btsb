#include "DIALOG.h"

#define ID_FRAMEWIN_0	(GUI_ID_USER + 0x00)
#define ID_BUTTON_0		(GUI_ID_USER + 0x01)
#define ID_BUTTON_1		(GUI_ID_USER + 0x02)

static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
	{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 320, 240, FRAMEWIN_CF_MOVEABLE, 0x64, 0 },
	{ BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 5, 180, 60, 25, 0, 0x0, 0 },
	{ BUTTON_CreateIndirect, "Button", ID_BUTTON_1, 75, 180, 60, 25, 0, 0x0, 0 },
};

extern WM_HWIN hPage[4];

static void _cbDialog(WM_MESSAGE * pMsg) {
	WM_HWIN hItem;
	int		 NCode;
	int		 Id;

	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG:
		hItem = pMsg->hWin;
		FRAMEWIN_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
		FRAMEWIN_SetText(hItem, "Page 3");
		FRAMEWIN_SetTitleHeight(hItem, 18);
		FRAMEWIN_SetFont(hItem, GUI_FONT_16B_1);
		FRAMEWIN_SetTextColor(hItem, (0x008000FF));
		FRAMEWIN_AddCloseButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_AddMaxButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_AddMinButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);
		FRAMEWIN_SetClientColor(pMsg->hWin, GUI_YELLOW);

		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
		BUTTON_SetText(hItem, "Return");

		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
		BUTTON_SetText(hItem, "Next");
		break;
	case WM_PAINT:
		GUI_SetColor(GUI_BLUE);								//����ǰ����ɫ
		GUI_SetFont((GUI_FONT *)&GUI_Font16_ASCII);			//���õ�ǰ�ֿ�
		GUI_SetTextMode(GUI_TEXTMODE_TRANS);				//��������͸����ʾ
		GUI_DispStringAt("neqee.com", 3, 3);				//�ػ���ʾ����
		break;
	case WM_NOTIFY_PARENT:
		Id		= WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
		switch(Id) {
		case ID_BUTTON_0:
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED:
				break;
			case WM_NOTIFICATION_RELEASED:
				WM_HideWindow(hPage[3]);					//����Page 3
				WM_ShowWindow(hPage[1]);					//��ʾPage 1
				break;
			}
			break;
		case ID_BUTTON_1:
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED:
				break;
			case WM_NOTIFICATION_RELEASED:
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

WM_HWIN CreateFramewin3(WM_HWIN hParent);
WM_HWIN CreateFramewin3(WM_HWIN hParent) {
	WM_HWIN hWin;
	hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, hParent, 0, 0);
	return hWin;
}
