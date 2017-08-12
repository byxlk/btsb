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
#define RECOMMENDED_MEMORY (1024L * 24)

/*
*********************************************************************************************************
*                                       �����ⲿ����
*********************************************************************************************************
*/
extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;


extern WM_HWIN CreateWindow_HomePage(WM_HWIN hParent);

/*
*********************************************************************************************************
*                                      ����������
*********************************************************************************************************
*/
static GUI_MEMDEV_Handle   hMempic;
WM_HWIN  hWin_HomePage;
WM_HWIN  hWin_IconMenu;

//WM_HWIN  hWinInfo;   /* ͨ��ICONVIEW���򿪴��ڵľ�� */
//WM_HWIN  hWinICON;   /* ICONVIEW�ؼ���� */
//WM_HWIN  hWinDesktop;   /* �����ھ��, ICONVIEW�ؼ������������������ */

uint8_t s_ucSelDesktopIndex = 0;
//uint8_t s_ucEnteryAppFlag = 0;

/* ʵ�ʵĲ�����Ҫ��ͼ���ȵ�4�����ɣ��м�(Ҳ���Ǳ�֤ÿ�����������32λ���ݵ����) */
static char _acBuffer[240 * 32];

RTC_TimeTypeDef  RTC_TimeStructure;
RTC_DateTypeDef  RTC_DateStructure;


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
*********************************************************************************************************
*	�� �� ��: Caculate_RTC
*	����˵��: ��ʾRTCʱ��
*	��    �Σ�pMsg ָ�����
*	�� �� ֵ: ��
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
        case WM_PAINT:/* �ػ���Ϣ*/
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
    GUI_Init(); /* ��ʼ���������Ի��� */
    //WM_EnableMemdev(WM_HBKWIN); /* ʹ�����洰��Ҳʹ���ڴ��豸 */
#endif
    //WM_MOTION_Enable(1);    /* ʹ�ܻ��� */
    //WM_MOTION_SetDefaultPeriod(50);

	/* ʹ��UTF-8�������ں�����ʾ */
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

    WM_CreateTimer(WM_GetClientWindow(hWinMain), /* ������Ϣ�Ĵ��ڵľ�� */
                       ID_TimerTime,                 /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
                       400,                         /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
                       0);                           /* ��������ʹ�ã�ӦΪ0 */
#endif
	while(1)
	{
		GUI_Delay(100);
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
