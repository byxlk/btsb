/*
*********************************************************************************************************
*
*	ģ������ : ���ܼҾӽ������
*	�ļ����� : MainTask.c
*	��    �� : V3.0
*	˵    �� : ʵ������
*              1. ��ʵ����������ֵ�ô��ѧϰ�ĵط�:
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"
#include "bsp.h"

#define RECOMMENDED_MEMORY (1024L * 30)

/*
*********************************************************************************************************
*                                       �����ⲿ����
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
WM_HWIN  hWin_HomePage;

extern void App_Bluetooth(WM_HWIN hWin);
extern void App_Music(WM_HWIN hWin);
extern void App_Sleep(WM_HWIN hWin);
extern void App_Language(WM_HWIN hWin);
extern void App_DateTime(WM_HWIN hWin);
extern void App_About(WM_HWIN hWin);
extern void App_HomePage(WM_HWIN hWin);

extern WM_HWIN CreateWindow_HomePage(WM_HWIN hParent);
/*
*********************************************************************************************************
*                                      ����������
*********************************************************************************************************
*/
//static GUI_MEMDEV_Handle   hMempic;

WM_HWIN  hWinInfo;   /* ͨ��ICONVIEW���򿪴��ڵľ�� */
WM_HWIN  hWinICON;   /* ICONVIEW�ؼ���� */
WM_HWIN  hWinDesktop;   /* �����ھ��, ICONVIEW�ؼ������������������ */

uint8_t	s_ucSelIconIndex = 0;	/* ѡ���ICON��Ĭ�ϲ�ѡ���κ� */
uint8_t s_ucSelDesktopIndex = 0;
uint8_t s_ucEnteryAppFlag = 0;


/* ʵ�ʵĲ�����Ҫ��ͼ���ȵ�4�����ɣ��м�(Ҳ���Ǳ�֤ÿ�����������32λ���ݵ����) */
static char _acBuffer[240 * 32];

/* ��������ICONVIEWͼ��Ĵ��� */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

/* ��������ICONVIEWͼ��Ĵ��� */
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
*                                  Ӧ�ó�����ں���
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
*                                     �궨��
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
*	�� �� ��: _GetData
*	����˵��: ������GUI_BMP_DrawEx()����
*	��    �Σ�p             FIL��������
*             NumBytesReq   �����ȡ���ֽ���
*             ppData        ����ָ��
*             Off           ���Off = 1
����ô�����´���ʵλ�ö�ȡ
*	�� �� ֵ: ���ض�ȡ���ֽ���
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
	* ��⻺���С
	*/
	if (NumBytesReq > sizeof(_acBuffer)) {
	    NumBytesReq = sizeof(_acBuffer);
	}

	/*
	* ���ö�ȡλ��
	*/
	if(Off == 1) FileAddress = 0;
	else FileAddress = Off;
	result =f_lseek(PicFile, FileAddress);

	/*
	* ��ȡ���ݵ�����, ����FatFS+�ٷ�SD���ķ�������DMA�����ϵ�4�ֽڶ������⣬
	* ������С�ڵ���һ��SD��������С������������512�ֽڻ����
	*/
	//for(i = 0; i < NumBytesReq / 512; i++)
	//{
	//	result = f_read(PicFile, &_acBuffer[512*i], 512, &bw);
	//	NumBytesRead += bw;
	//}

	result = f_read(PicFile, _acBuffer, NumBytesReq, &NumBytesRead);
	//NumBytesRead += bw;

	/*
	* ��ָ��ppDataָ���ȡ�ĺ���
	*/
	*ppData = (const U8 *)&_acBuffer[0];

	/*
	* ���ض�ȡ���ֽ���
	*/
	return NumBytesRead;
}

/*
*******************************************************************************
*	�� �� ��: _ShowBMPEx
*	����˵��: ��ʾBMPͼƬ
*	��    ��: sFilename Ҫ��ʾͼƬ������
*	�� �� ֵ: ��
*******************************************************************************
*/
static void _ShowBMPEx(const char * sFilename)
{
    //OS_ERR      	err;

    /* �����ļ�ϵͳ */
    MountFS(&fs, 0);

    /* ���ļ� */
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

    /* ж���ļ�ϵͳ */
    MountFS(NULL, 0);
}

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
//    GUI_ALPHA_STATE AlphaState;

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
                        //WM_MoveTo(hWinMain, 0, 0);
                        //WM_ShowWindow(hWin_HomePage);
                    } else {
                        //WM_HideWindow(hWin_HomePage);
                        //WM_ShowWindow(hWinICON);
                        //WM_DeleteWindow(hWin_HomePage);
                        //WM_MoveTo(hWinMain, -LCD_GetXSize(), 0);
                        //WM_ShowWindow(hWinICON);
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

static void GUI_KeyEvent_Response(WM_KEY_INFO *KeyInfo)
{
    switch (KeyInfo->Key)
    {
        case GUI_KEY_Menu:
            _LOGD("GUI_KEY_Menu \r\n");
            if(s_ucSelDesktopIndex == 0) {
                s_ucSelDesktopIndex = 1;
                //WM_HideWindow(hWinMain);
                WM_ShowWindow(hWinICON);
                //if(hWinICON == HBWIN_NULL)
                //    hWinICON = CreateWindow_IconMenu(WM_HBKWIN);
                //if(hWinMain != HBWIN_NULL) {
                //    WM_DeleteWindow(hWinMain);
                //    hWinMain = HBWIN_NULL;
                //}
            } else {
                s_ucSelDesktopIndex = 0;
                WM_HideWindow(hWinICON);
                //WM_ShowWindow(hWinMain);
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
}


/*
*******************************************************************************
*	�� �� ��: _cbBkWindow
*	����˵��: ���洰�ڵĻص�����,
������Ҫ�ǻ��Ʊ������ںͽ����л�ʱ���л���־�Ļ���
*	��    ��: pMsg  WM_MESSAGE����ָ�����
*	�� �� ֵ: ��
*******************************************************************************
*/
static void _cbBackGround(WM_MESSAGE * pMsg)  //���汳���Ļص�����
{
    _LOGD("MsgId = %d\r\n",pMsg->MsgId);
	switch (pMsg->MsgId)
	{
	    case WM_TIMER:
            WM_InvalidateWindow(pMsg->hWin);
            WM_RestartTimer(pMsg->Data.v, 200);
            break;
        case WM_PAINT:/* �ػ���Ϣ*/
            _LOGD("WM_PAINT(%d)\r\n",pMsg->MsgId);
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
*	�� �� ��: MainTask
*	����˵��: ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MainTask(void)
{
    WM_HWIN    hWinMain;

     /****************************************************************************
     * ���ڶ໺��ʹ����ڴ��豸������˵��
     * 1. ʹ�ܶ໺���ǵ��õ����º������û�Ҫ��LCDConf_Lin_Template.c�ļ���
     *    �����˶໺�壬���ô˺�������Ч��WM_MULTIBUF_Enable(1);
     * 2. ����ʹ��ʹ���ڴ��豸�ǵ��ú�����WM_SetCreateFlags(WM_CF_MEMDEV);
     * 3. ���emWin�����ö໺��ʹ����ڴ��豸��֧�֣���ѡһ���ɣ����������
     *    ѡ��ʹ�ö໺�壬ʵ��ʹ ��STM32F429BIT6 + 32λSDRAM + RGB565/RGB888
     *    ƽ̨���ԣ��໺�������Ч�Ľ��ʹ����ƶ����߻���ʱ��˺�ѸУ�
     *    ����Ч����������ԣ�ͨ��ʹ�ܴ���ʹ���ڴ��豸���������ġ�
     * 4. ����emWin����Ĭ���ǿ��������塣

    *****************************************************************************/
#if 0
    GUI_Init(); /* ��ʼ���������Ի��� */
    WM_MULTIBUF_Enable(1);
#else
	WM_SetCreateFlags(WM_CF_MEMDEV); /* ����ʹ���ڴ��豸 */
    //WM_EnableMemdev(WM_HBKWIN); /* ʹ�����洰��Ҳʹ���ڴ��豸 */
    GUI_Init(); /* ��ʼ���������Ի��� */
#endif
    //WM_MOTION_Enable(1);    /* ʹ�ܻ��� */
    //WM_MOTION_SetDefaultPeriod(50);

	/* ʹ��UTF-8�������ں�����ʾ */
	GUI_UC_SetEncodeUTF8();

    _LOGD("FreeBytes = %d\r\n",GUI_ALLOC_GetNumFreeBytes());
    if (GUI_ALLOC_GetNumFreeBytes() < RECOMMENDED_MEMORY) {
        GUI_ErrorOut("Not enough memory available.");
        return;
    }

	FRAMEWIN_SetDefaultFont(&GUI_FontComic18B_ASCII);
    FRAMEWIN_SetDefaultTextAlign(GUI_TA_CENTER);
    TEXT_SetDefaultFont(&GUI_FontComic18B_ASCII);
    TEXT_SetDefaultTextColor(GUI_WHITE);
    FRAMEWIN_SetDefaultBarColor(1, GUI_MAGENTA);

    //hMempic = GUI_MEMDEV_CreateFixed(0, 0,
	//                                 LCD_GetXSize(),
	//                                 LCD_GetYSize(),
	//								 GUI_MEMDEV_HASTRANS,
	//								 GUI_MEMDEV_APILIST_16,
	//								 GUICC_M565);
	//GUI_MEMDEV_Select(hMempic);
    //_ShowBMPEx("bg.bmp");
	//GUI_MEMDEV_Select(0);
    WM_SetDesktopColor(GUI_BLACK);
    hWinMain = WM_CreateWindow(0, 0, LCD_GetXSize(), LCD_GetYSize(),
                             WM_CF_SHOW, _cbBackGround, 0);
    WM_CreateTimer(hWinMain, 0, 10, 0);

#if 1
    hWin_HomePage = CreateWindow_HomePage(0);
    WM_CreateTimer(hWin_HomePage, 0, 10, 0);

    //hWin_IconMenu = CreateWindow_IconMenu(0);


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

    WM_CreateTimer(WM_GetClientWindow(hWinMain), /* ������Ϣ�Ĵ��ڵľ�� */
                       ID_TimerTime,                 /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
                       400,                         /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
                       0);                           /* ��������ʹ�ã�ӦΪ0 */
#endif
	while(1)
	{
		GUI_Delay(50);
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
