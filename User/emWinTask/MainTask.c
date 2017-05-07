/*
*********************************************************************************************************
*
*	ģ������ : ���ܼҾӽ������
*	�ļ����� : MainTask.c
*	��    �� : V3.0
*	˵    �� : ʵ������
*              1. ��ʵ����������ֵ�ô��ѧϰ�ĵط�:
*                 (1). ICONVIEW�ؼ���ʹ�á�
*                 (2). ���н���֧�ִ���Ҳ֧�ְ���������
*              2. ����K2ʵ�ֶԻ����ɾ����
*                 ����K3ʵ��ICONVIEW�ľ۽���
*				  ҡ��UP��ʵ��ICONVIEWѡ������ơ�
*				  ҡ��DOWN��ʵ��ICONVIEWѡ������ơ�
*				  ҡ��LIGHT��ʵ��ICONVIEWѡ������ơ�
*				  ҡ��RIGHT��ʵ��ICONVIEWѡ������ơ�
*				  ҡ��OK��ʵ�ֶԻ���Ĵ�����
*
*	�޸ļ�¼ :
*		�汾��    ����          ����          ˵��
*		V1.0    2014-06-21    Eric2013        �׷�
*             								 ʵ�ʲ����з����������⣬ֵ�ô��ע�⣺
*                							 (1). ʹ��ICONVIEW��WM_NOTIFICATION_CLICKED��Ϣ��ʹ�ô�һ�ζԻ�����ٽ���رա�
*                      							  �ٴβ���ICONVIEW��ʱ����Ҫ������β���Ч����
*                 							 (2). ��ʾ�α��������ʾͼƬΪ565��ʽ��GUI_DrawBitmap(&bmButtonLine, 0, 272-44);�޷���ʾ�ˡ�
*                      							  ��ʾ�α��������ʾͼƬΪ32λARGB��ʽ��GUI_DrawBitmap(&bmButtonLine, 0, 272-44);������ʾ��
*                 							 (3). ʹ��hWinMain����ʹ���ڴ��豸��WM_SetCreateFlags(WM_CF_MEMDEV);���������Ͻ�����ICONVIEW.
*                      							  4.3�����������ʾ��7���5�������ܣ�ʹ���ⲿSRAM��ΪemWin��̬�ڴ�����޸�LCDConf.C�е�5��
*                      							  ��7����������480*272�����ʹ��ԭʼ�ߴ罫���ͼ�겻��ʾ����˴��ڷ�����Ϣ���������
*	    V2.0    2015-04-15    Eric2013       1. �����̼��⵽V1.5.0
*                                            2. ����BSP�弶֧�ְ�
*                                            3. ����fatfs��0.11
*                                            4. ����STemWin��5.26
*                                            5. ����Ϊ�µ��ĵ㴥��У׼�㷨�����ز���Ŵ����������
*                                            6. ���7��800*480�ֱ��ʵ�����֧�֣����3.5��480*320��ILI9488֧�֡�
*                                            7. ���±���uCOS-III������
*                                            8. V2.0�汾ʹ���ⲿ2MB SRAM��ΪemWin��̬�ڴ棬V1.0�汾���ڵ������Ѿ��õ������
*	    V3.0    2015-12-18  Eric2013         1. ����BSP�弶֧�ְ�
*                                            2. ����STemWin��5.28
*                                            3. ���4.3���5�������֧�֡�
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"
#include "bsp.h"


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
extern WM_HWIN  hWin_HomePage;

extern void App_Bluetooth(WM_HWIN hWin);
extern void App_Music(WM_HWIN hWin);
extern void App_Sleep(WM_HWIN hWin);
extern void App_Language(WM_HWIN hWin);
extern void App_DateTime(WM_HWIN hWin);
extern void App_About(WM_HWIN hWin);
extern void App_HomePage(WM_HWIN hWin);
/*
*********************************************************************************************************
*                                      ����������
*********************************************************************************************************
*/
static GUI_MEMDEV_Handle   hMempic;

WM_HWIN  hWinInfo;   /* ͨ��ICONVIEW���򿪴��ڵľ�� */
WM_HWIN  hWinICON;   /* ICONVIEW�ؼ���� */
WM_HWIN  hWinMain;
WM_HWIN  hWinDesktop;   /* �����ھ��, ICONVIEW�ؼ������������������ */

uint8_t	s_ucSelIconIndex = 0;	/* ѡ���ICON��Ĭ�ϲ�ѡ���κ� */
uint8_t s_ucSelDesktopIndex = 0;
uint8_t s_ucEnteryAppFlag = 0;
#if 0
GUI_RECT Icon_Rect[6] = {
    {22,  30.104, 110},          // 0
    {144, 30.206, 110},          // 1
    {22,  122.104, 202},          // 2
    {144, 122.206, 202},          // 3
    {22,  214.104, 294},          // 4
    {144, 214.206, 294}           // 5
};
#endif

/* ʵ�ʵĲ�����Ҫ��ͼ���ȵ�4�����ɣ��м�(Ҳ���Ǳ�֤ÿ�����������32
λ���ݵ����) */
static char _acBuffer[240 * 48];

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

#if 0
/*
*******************************************************************************
*	�� �� ��: _CreateButton
*	����˵��: ������ť
*	��    �Σ�hParent  ������
*             pText    ��������ʾ���ı�
*             Id       ��ťId
*             x        x������
*             y        y������
*             w        ��ť��
*             h        ��ť��
*             TextId   �ı���ID
*	�� �� ֵ: ��
*******************************************************************************
*/
static WM_HWIN _CreateButton(WM_HWIN hParent, const char* pText,
                            int Id, int x, int y, int w, int h, unsigned TextId)
{
	WM_HWIN hButton;
	hButton = BUTTON_CreateEx(x, y, w, h, hParent, WM_CF_SHOW, 0, Id);

	/* ��ȡ��ǰ������ťҪ��ʾ���ı� */

	BUTTON_SetText      (hButton,    pText);

	/* ���ý������뽹������� */
	//BUTTON_SetFocussable(hButton,    1);

	return hButton;
}
#endif

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
                hWinICON = _CreateICONVIEW(pMsg->hWin,
                             _aBitmapItem, GUI_COUNTOF(_aBitmapItem),
                             GUI_ID_ICONVIEW0,
                             LCD_GetXSize()+22, 30, 210, 280);
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

/*
*******************************************************************************
*	�� �� ��: _cbBkWindow
*	����˵��: ���洰�ڵĻص�����,
������Ҫ�ǻ��Ʊ������ںͽ����л�ʱ���л���־�Ļ���
*	��    ��: pMsg  WM_MESSAGE����ָ�����
*	�� �� ֵ: ��
*******************************************************************************
*/
static void _cbBkWindow(WM_MESSAGE * pMsg)
{
    //printf("[%s : %d] MsgId = %d\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
	switch (pMsg->MsgId)
	{
       case WM_PAINT:/* �ػ���Ϣ*/
            printf("[%s : %d] WM_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            GUI_MEMDEV_Select(hMempic);
            _ShowBMPEx("bg.bmp");
        	GUI_MEMDEV_Select(0);
            GUI_MEMDEV_WriteAt(hMempic, 0, 0);
			break;
       case WM_PRE_PAINT:
            //printf("[%s : %d] WM_PRE_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            GUI_MULTIBUF_Begin();
            break;
       case WM_POST_PAINT:
            //printf("[%s : %d] WM_PRE_PAINT(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            GUI_MULTIBUF_End();
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
	/* ��ʼ���������Ի��� */
	GUI_Init();
    //GUI_EnableAlpha(1);

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
#if 1
    WM_MULTIBUF_Enable(1);
#else
    /* ����ʹ���ڴ��豸 */
	WM_SetCreateFlags(WM_CF_MEMDEV | WM_CF_MEMDEV_ON_REDRAW);

    /* ʹ�����洰��Ҳʹ���ڴ��豸 */
    //WM_EnableMemdev(WM_HBKWIN);
#endif
    WM_MOTION_Enable(1);    /* ʹ�ܻ��� */
    //WM_MOTION_SetDefaultPeriod(50);

	/* ʹ��UTF-8�������ں�����ʾ */
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
                                  LCD_GetXSize()*2,
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbDesktopDisplayProc, 0);

#if 0
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
