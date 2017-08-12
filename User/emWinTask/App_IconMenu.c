/*
*********************************************************************************************************
*
*	ģ������ : ���ֲ�����Ӧ�ý������
*	�ļ����� : App_MucisDlg.c
*	��    �� : V1.0
*	˵    �� : ���ֲ�����������ơ�
*              1. ֧����һ������һ��������Ϳ��ˣ����õĲ����ʺ����ʶ�֧�֣���������������Ҳ��֧�֡�
*              2. emWin�����ǵ����ȼ��������ֽ��������Ǹ����ȼ�������������֮��ͨ��������Ϣ���к��¼�
*                 ��־����ͨ�š�
*              3. �״�ʹ���ȵ�������б��������ᱻ��¼��listview�ؼ����棬Ȼ��Ϳ�����������ˡ�
*                 ����ļ����и����϶࣬�״δ򿪻�����Щ����Ҫ����ΪҪ��ȡÿ�׸����Ĳ���ʱ�䡣�Ժ��
*                 �ͱȽϿ��ˣ���Ҫ�ǶԸ����б�Ի����������غ���ʾ�����������ظ��Ĵ�����ɾ����
*              4. �����б�Ի�������ģ̬���������û��򿪴˶Ի����ֻ�ܲ�������Ի��򣬶����ܲ��������档
*
*	�޸ļ�¼ :
*		�汾��    ����         ����         ˵��
*       V1.0    2016-12-30   Eric2013       �׷�
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"

/*
*********************************************************************************************************
*				                      �궨��
*********************************************************************************************************
*/
#define ID_FRAMEWIN_0    (GUI_ID_USER + 0x01)
#define ID_ICONVIEW_0    (GUI_ID_USER + 0x02)
#define ID_TEXT_0    (GUI_ID_USER + 0x03)

static WM_CALLBACK * _pcbClient;
uint8_t	s_ucSelIconIndex = 0;	/* ѡ���ICON��Ĭ�ϲ�ѡ���κ� */


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

extern WM_HWIN App_Bluetooth(WM_HWIN hWin);
extern WM_HWIN App_Music(WM_HWIN hWin);
extern WM_HWIN App_Sleep(WM_HWIN hWin);
extern WM_HWIN App_Language(WM_HWIN hWin);
extern WM_HWIN App_DateTime(WM_HWIN hWin);
extern WM_HWIN App_About(WM_HWIN hWin);
extern WM_HWIN App_HomePage(WM_HWIN hWin);


/* ��������ICONVIEWͼ��Ĵ��� */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

/* ��������ICONVIEWͼ��Ĵ��� */
static const BITMAP_ITEM _aIconBmp[] =
{
	{&bma,    "BT"},
	{&bmb,    "Music"},
	{&bmc,    "Sleep"},
	{&bmd,    "Language"},
	{&bme,    "DateTime"},
	{&bmf,    "About"},
};

/*
*******************************************************************************
*                                  Ӧ�ó�����ں���
*******************************************************************************
*/
static WM_HWIN (* _appModules[])( WM_HWIN hWin) =
{
	App_Bluetooth,
	App_Music,
	App_Sleep,
	App_Language,
	App_DateTime,
	App_About,
};

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate_IconMenu[] = {
  { FRAMEWIN_CreateIndirect, "App Menu", ID_FRAMEWIN_0, 0, 0, 240, 320, FRAMEWIN_CF_MOVEABLE, 0x0, 0 },
  { ICONVIEW_CreateIndirect, "Iconview", ID_ICONVIEW_0, 38, 76, 164, 220, 0, 0x00400040/*xSizeItem-ySizeItem*/, 0 },
  { TEXT_CreateIndirect, "App Menu", ID_TEXT_0, 80, 36, 80, 20, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};
#if 0
/*
*******************************************************************************
*	�� �� ��: _CreateICONVIEW
*	����˵��: ����ICONVIEW
*	��    �Σ�hParent   ������
*             pBm       ICONVIEW�ϵ�λͼ
*             BitmapNum ICONVIEW��ͼ�����
*             x         x������
*             y         y������
*             w         ICONVIEW��
*             h         ICONVIEW��
*	�� �� ֵ: ��
*******************************************************************************
*/
static WM_HWIN _CreateICONVIEW(WM_HWIN hParent,
                                          const BITMAP_ITEM *pBm,
                                          int BitmapNum, int Id,
                                          int x, int y, int w, int h)
{
	WM_HWIN hIcon;
	int i;

    /*��ָ��λ�ô���ָ���ߴ��ICONVIEW С����*/
	hIcon = ICONVIEW_CreateEx(x, 				/* С���ߵ��������أ��ڸ������У�*/
						     y, 					/* С���ߵ��������أ��ڸ������У�*/
							 w,    				/* С���ߵ�ˮƽ�ߴ磨��λ�����أ�*/
							 h, 	                /* С���ߵĴ�ֱ�ߴ磨��λ�����أ�*/
	                         hParent, 			/* �����ڵľ�������Ϊ0������С���߽���Ϊ���棨�������ڣ����Ӵ��� */
							 WM_CF_SHOW | WM_CF_HASTRANS,       /*���ڴ�����ǡ�ΪʹС���������ɼ���ͨ��ʹ�� WM_CF_SHOW */
	                         0,//ICONVIEW_CF_AUTOSCROLLBAR_V, 	/* Ĭ����0�����������ʵ������������ֱ������ */
							 Id, 			        /* С���ߵĴ���ID */
							 82, 				    /* ͼ���ˮƽ�ߴ� */
							 80);/* ͼ��Ĵ�ֱ�ߴ� */

	/* ��ICONVIEW С���������ͼ�� */
	for (i = 0; i < BitmapNum; i++)
	{
		ICONVIEW_AddBitmapItem(hIcon, pBm[i].pBitmap, pBm[i].pText);
	}

	/* ����С���ߵı���ɫ 32 λ��ɫֵ��ǰ8 λ������alpha��ϴ���Ч��*/
	ICONVIEW_SetBkColor(hIcon, ICONVIEW_CI_SEL, GUI_DARKBLUE | 0x80000000);
    //ICONVIEW_SetBkColor(hIcon, ICONVIEW_CI_BK, GUI_WHITE | 0x00000000);
    ICONVIEW_SetTextColor(hIcon, ICONVIEW_CI_BK, GUI_BLACK);
    ICONVIEW_SetTextColor(hIcon, ICONVIEW_CI_SEL, GUI_BLACK);

	/* �������� */
	ICONVIEW_SetFont(hIcon, &GUI_FontYahei);

	/* ����ͼ����x ��y �����ϵļ�ࡣ*/
	ICONVIEW_SetSpace(hIcon, GUI_COORD_Y, 12);
    ICONVIEW_SetSpace(hIcon, GUI_COORD_X, 20);

    //ICONVIEW_SetFrame(hIcon, GUI_COORD_X, 0);
    //ICONVIEW_SetFrame(hIcon, GUI_COORD_Y, 0);

	/* ���ö��뷽ʽ ��5.22�汾�����¼���� */
	ICONVIEW_SetIconAlign(hIcon, ICONVIEW_IA_HCENTER | ICONVIEW_IA_TOP);

	return hIcon;
}

/*
*********************************************************************************************************
*	�� �� ��: _cbDialogInfo
*	����˵��: �����ڵĻص�����
*	��    �Σ�pMsg   ����ָ��
*	�� �� ֵ: ��
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
			/* ��ʾʱ������� */
			//Caculate_RTC(pMsg);
			/* ������ʱ�� */
			//WM_RestartTimer(pMsg->Data.v, 1000);
			break;

		case WM_NOTIFY_PARENT:
            printf("[%s : %d] WM_NOTIFY_PARENT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
			Id    = WM_GetId(pMsg->hWinSrc);
			NCode = pMsg->Data.v;
			switch (Id)
			{
				/* ��һ�������ϵ�ͼ�� */
				case GUI_ID_ICONVIEW0:
					switch (NCode)
					{
						/* ICON�ؼ������Ϣ */
						case WM_NOTIFICATION_CLICKED:
							break;

                        case WM_NOTIFICATION_SEL_CHANGED:
                            break;

						/* ICON�ؼ��ͷ���Ϣ */
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

                case GUI_KEY_Vol_Dec://������С
                    break;

                case GUI_KEY_Vol_Plus://��������
                    break;

                case GUI_KEY_LockScreen://����
                    break;

                case GUI_KEY_UnLock://����
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

static void _cbWinCallBack_IconMenu(WM_MESSAGE * pMsg)
{
    int     i;
	int     Id;
	int     NCode;
	WM_HWIN hItem;

	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG://��ʼ����Ϣ,��������/�ؼ�ʱ��Ч,��������������һЩ�ؼ��ĳ�ʼ����
        _LOGD("WM_INIT_DIALOG(%d)\r\n",pMsg->MsgId);
#if 1
		// Initialization of 'App Menu'
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetTextColor(hItem, (0x008000FF));
		TEXT_SetFont(hItem, GUI_FONT_20_1);
		TEXT_SetText(hItem, "App Menu");

		hItem = WM_GetDialogItem(pMsg->hWin, ID_ICONVIEW_0);
        /* ����С���ߵı���ɫ 32 λ��ɫֵ��ǰ8 λ������alpha��ϴ���Ч��*/
		//ICONVIEW_SetBkColor(hIcon, ICONVIEW_CI_SEL, GUI_DARKBLUE | 0x80000000);
		//ICONVIEW_SetIconAlign(hItem, ICONVIEW_IA_TOP);
        ICONVIEW_SetIconAlign(hItem, ICONVIEW_IA_HCENTER | ICONVIEW_IA_TOP);
		ICONVIEW_SetTextColor(hItem, ICONVIEW_CI_UNSEL, GUI_BLUE);
		ICONVIEW_SetTextColor(hItem, ICONVIEW_CI_SEL, GUI_GREEN);
		ICONVIEW_SetFrame(hItem, GUI_COORD_X, 0);//����ͼ�굽IconView�߿�ļ��
		ICONVIEW_SetFrame(hItem, GUI_COORD_Y, 0);
		ICONVIEW_SetSpace(hItem, GUI_COORD_X, (164-64*2)/1);//����ͼ���ͼ��֮��ļ��
		ICONVIEW_SetSpace(hItem, GUI_COORD_Y, (220-64*3)/2);
        /* �������� */
		//ICONVIEW_SetFont(hIcon, &GUI_FontYahei);
		ICONVIEW_SetFont(hItem, GUI_FONT_16_1);

        /* ��ICONVIEW С���������ͼ�� */
		for (i = 0; i < GUI_COUNTOF(_aIconBmp); i++)
		{
			ICONVIEW_AddBitmapItem(hItem, _aIconBmp[i].pBitmap, _aIconBmp[i].pText);
		}
#endif
		break;
    //case WM_PRE_PAINT:
    //    GUI_MULTIBUF_Begin();
    //    break;

    //case WM_POST_PAINT:
    //    GUI_MULTIBUF_End();
    //    break;

    case WM_PAINT:
        //_LOGD("WM_PAINT(%d)\r\n",pMsg->MsgId);
		GUI_SetAlpha(0xFF);
        GUI_Clear();
        return;
	case WM_NOTIFY_PARENT://����������Ϣ����(������Ļ������ܵ�����),����������������༭��(�κεĲ���)�ȵ�......
		Id    = WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
        _LOGD("WM_NOTIFY_PARENT(%d)\r\n",pMsg->MsgId);
		switch(Id) {
		case ID_ICONVIEW_0:
			switch(NCode) {
			case WM_NOTIFICATION_CLICKED://���ͼ��
			    s_ucSelIconIndex = ICONVIEW_GetSel(WM_GetDialogItem(pMsg->hWin, ID_ICONVIEW_0));
                TEXT_SetText(WM_GetDialogItem(pMsg->hWin, ID_TEXT_0), _aIconBmp[s_ucSelIconIndex].pText);
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
	if (_pcbClient) _pcbClient(pMsg);
}

static void _cbFrame(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    GUI_SetAlpha(0xFF);
    break;
  }
  FRAMEWIN_Callback(pMsg);
  GUI_SetAlpha(0);
}

/*
*********************************************************************************************************
*	�� �� ��: App_Music
*	����˵��: �������ֲ��ŶԻ���
*	��    ��: hWin  ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN CreateWindow_IconMenu(WM_HWIN hParent)
{
	WM_MESSAGE Msg = {0};
    WM_HWIN    hFrame;
    WM_HWIN    hClient;

    hFrame = GUI_CreateDialogBox(_aDialogCreate_IconMenu,
                                 GUI_COUNTOF(_aDialogCreate_IconMenu),
                                 0, hParent, 0, 0);
    WM_SetHasTrans(hFrame);           // Set transparency
    WM_SetCallback(hFrame, _cbFrame); // Overwrite callback

    hClient    = WM_GetClientWindow(hFrame);         // Get handle of client window
    WM_SetHasTrans(hClient);                         // Set transparency
    _pcbClient = WM_SetCallback(hClient, _cbWinCallBack_IconMenu); // Overwrite callback
    Msg.MsgId = WM_INIT_DIALOG;                      // Send WM_INIT_DIALOG
    WM_SendMessage(hClient, &Msg);

    return hFrame;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
