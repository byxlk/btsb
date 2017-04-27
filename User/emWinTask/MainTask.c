/*
*********************************************************************************************************
*
*	ģ������ : ATM��������
*	�ļ����� : MainTask.c
*	��    �� : V1.0
*	˵    �� : ʵ����������
*              1. ���н���֧�ְ�������
*                 ��1��K2�������ڿؼ�������л����л�ʱ����ѡ��Ŀؼ������ǳɫ�߿�
*                 ��2��K3�������ڽ�����һ������
*                 ��3��ҡ�˵�OK������ѡ����Ӧ�ؼ��󣬴����ؼ�����Ӧ�ĵĲ�����
*              2. ���ô�����ʽ��������ʱ��������뵽û�а�ť�ؼ�ʱ�����Ե����Ļ�м䲿�ֽ�����һҳ��
*
*	�޸ļ�¼ :
*		�汾��   ����         ����          ˵��
*		V1.0    2016-11-26   Eric2013  	    �װ�
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "MainTask.h"
#include "includes.h"

/*
*********************************************************************************************************
*                                         extern
*********************************************************************************************************
*/
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_AmericanExpress;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_ECCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_GeldKarte;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_Maestro;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_MasterCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_VisaCard;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armfly;
//extern GUI_CONST_STORAGE GUI_BITMAP bmLogo_armflySmall;

extern GUI_CONST_STORAGE GUI_FONT GUI_FontYahei;

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

/*
*********************************************************************************************************
*                                       �궨��
*********************************************************************************************************
*/
#define MAIN_BORDER               0
#define MAIN_TITLE_HEIGHT         0

#define FRAME_BKCOLOR             0xD0D0D0
#define FRAME_TEXTCOLOR           0x000000
#define FRAME_FONT                (&GUI_FontYahei)
#define FRAME_EFFECT              (&WIDGET_Effect_Simple)
#define FRAME_BORDER              FRAME_EFFECT->EffectSize
#define FRAME_WIDTH               (LCD_GetXSize() - (FRAME_BORDER * 2) - (MAIN_BORDER * 2))
#define FRAME_HEIGHT              (LCD_GetYSize() - (FRAME_BORDER * 2) - (MAIN_BORDER + MAIN_TITLE_HEIGHT))

/*
*********************************************************************************************************
*                                       �궨��-����
*********************************************************************************************************
*/
#define MSG_CARD_INSERTED   (WM_USER + 0)
#define MSG_CARD_REMOVED    (WM_USER + 1)
#define MSG_MONEY_REMOVED   (WM_USER + 2)
#define MSG_PIN_CHANGED     (WM_USER + 3)
#define MSG_PIN_OK          (WM_USER + 4)
#define MSG_PIN_CANCEL      (WM_USER + 5)
#define MSG_PIN_ERROR       (WM_USER + 6)

/*
*********************************************************************************************************
*                                       �ṹ��
*********************************************************************************************************
*/
typedef struct {
  int x;
  int y;
  int Pressed;
  int Duration;
} PID_EVENT;


/* ��������ICONVIEWͼ��Ĵ��� */
typedef struct
{
	const GUI_BITMAP * pBitmap;
	const char       * pText;
} BITMAP_ITEM;

/*
*********************************************************************************************************
*                                       ��̬����
*********************************************************************************************************
*/

static WM_HWIN    _hLastFrame;

WM_HWIN  hWinInfo;   /* ͨ��ICONVIEW���򿪴��ڵľ�� */
WM_HWIN  hWinICON;   /* ICONVIEW�ؼ���� */
WM_HWIN  hWinMain;   /* �����ھ��, ICONVIEW�ؼ������������������ */
uint8_t	s_ucSelIconIndex = 0;	/* ѡ���ICON��Ĭ�ϲ�ѡ���κ� */


extern RTC_T g_tRTC;


/* ��������ICONVIEWͼ��Ĵ��� */
static const BITMAP_ITEM _aBitmapItem[] =
{
	{&bma,    "Setting"},
	{&bmb,    "Music"},
	{&bmc,    "TDate"},
	{&bmd,    "Sleep"},
	{&bme,    "Lanuage"},
	{&bmf,    "About"},
};


static void _cbDrawHomePage(WM_MESSAGE * pMsg);

/*
*********************************************************************************************************
*	�� �� ��: _PaintFrame
*	����˵��: ��ܴ��ڵ��ػ溯��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _PaintFrame(void)
{
	GUI_RECT r;
	WM_GetClientRect(&r);
	GUI_SetBkColor(FRAME_BKCOLOR);
	GUI_SetColor(FRAME_TEXTCOLOR);
	GUI_SetFont(FRAME_FONT);
	GUI_SetTextMode(GUI_TM_TRANS);
	GUI_ClearRectEx(&r);
}

/*
*********************************************************************************************************
*	�� �� ��: _CreateFrame
*	����˵��: ������ܴ���
*	��    �Σ�cb  �ص�������ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static WM_HWIN _CreateFrame(WM_CALLBACK* cb)
{
#if 0
	//int x = 0;
	//int y = 0;
	//x = FRAME_BORDER + MAIN_BORDER;
	//y = FRAME_BORDER + MAIN_TITLE_HEIGHT;

	//_hLastFrame = WM_CreateWindowAsChild(x, y,
                                           FRAME_WIDTH,
                                           FRAME_HEIGHT,
                                           WM_HBKWIN,
                                           WM_CF_SHOW, cb, 0);
#else
	_hLastFrame = WM_CreateWindowAsChild(0, 0,
	                                         FRAME_WIDTH,
	                                         FRAME_HEIGHT,
	                                         WM_HBKWIN,
	                                         WM_CF_SHOW, cb, 0);
#endif
	return _hLastFrame;
}

/*
*********************************************************************************************************
*	�� �� ��: _DeleteFrame
*	����˵��: ɾ�������Ŀ�ܴ���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _DeleteFrame(void)
{
	WM_DeleteWindow(_hLastFrame);
	_hLastFrame = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: _cbLanguage
*	����˵��: ��һ�����棬�������ĺ�Ӣ���ѡ��
*	��    ��: pMsg  ����ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbShowDesktop(WM_MESSAGE* pMsg)
{
    unsigned char i = 0;
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
		case WM_CREATE:
			GUI_DispStringHCenterAt("WM_CREATE", FRAME_WIDTH >> 1, 64);
			/* ���þ۽� */
			WM_SetFocus(hWin);
			break;

		case WM_PAINT:
			_PaintFrame();

			/*��ָ��λ�ô���ָ���ߴ��ICONVIEW С����*/
        	hWinICON = ICONVIEW_CreateEx(22, 				/* С���ߵ��������أ��ڸ������У�*/
        						     30, 					/* С���ߵ��������أ��ڸ������У�*/
        							 220,    				/* С���ߵ�ˮƽ�ߴ磨��λ�����أ�*/
        							 280, 	/* С���ߵĴ�ֱ�ߴ磨��λ�����أ�*/
        	                         hWin, 				        /* �����ڵľ�������Ϊ0������С���߽���Ϊ���棨�������ڣ����Ӵ��� */
        							 WM_CF_SHOW | WM_CF_HASTRANS,       /* ���ڴ�����ǡ�ΪʹС���������ɼ���ͨ��ʹ�� WM_CF_SHOW */
        	                         0,//ICONVIEW_CF_AUTOSCROLLBAR_V, 	/* Ĭ����0�����������ʵ������������ֱ������ */
        							 GUI_ID_ICONVIEW0, 			        /* С���ߵĴ���ID */
        							 82, 				    /* ͼ���ˮƽ�ߴ� */
        							 80);/* ͼ��Ĵ�ֱ�ߴ� */


        	/* ��ICONVIEW С���������ͼ�� */
        	for (i = 0; i < GUI_COUNTOF(_aBitmapItem); i++)
        	{
        		ICONVIEW_AddBitmapItem(hWinICON, _aBitmapItem[i].pBitmap, _aBitmapItem[i].pText);
        	}

        	/* ����С���ߵı���ɫ 32 λ��ɫֵ��ǰ8 λ������alpha��ϴ���Ч��*/
        	ICONVIEW_SetBkColor(hWinICON, ICONVIEW_CI_SEL, GUI_WHITE | 0x80000000);

        	/* �������� */
        	ICONVIEW_SetFont(hWinICON, &GUI_FontYahei);

        	/* ����ͼ����x ��y �����ϵļ�ࡣ*/
        	ICONVIEW_SetSpace(hWinICON, GUI_COORD_Y, 10);
            ICONVIEW_SetSpace(hWinICON, GUI_COORD_X, 20);

        	/* ���ö��뷽ʽ ��5.22�汾�����¼���� */
        	ICONVIEW_SetIconAlign(hWinICON, ICONVIEW_IA_HCENTER|ICONVIEW_IA_TOP);

        	WM_CreateTimer(WM_GetClientWindow(hWinMain), /* ������Ϣ�Ĵ��ڵľ�� */
        				   1, 	             /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
        				   20,                           /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
        				   0);	                         /* ��������ʹ�ã�ӦΪ0 */
            break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_BACKTAB:
                    _DeleteFrame();
                    _CreateFrame(&_cbDrawHomePage);
                    break;

                case GUI_KEY_MUSIC://��������
                    break;

        		case GUI_KEY_SLEEPMODE://����˯��ģʽ
                    break;

                case GUI_KEY_DOWN://������С
                    break;

                case GUI_KEY_UP://��������
                    break;

                case KEY_DOWN_MUX://����
                    break;

                case KEY_DOWN_MUX_LONG://����
                    break;

                default:
                    break;
            }
		default:
			WM_DefaultProc(pMsg);
            break;
	}
}

static void _cbDrawHomePage(WM_MESSAGE * pMsg)
{
  //const void * pData;
  //WM_HWIN      hItem;
  //U32          FileSize;
  // USER START (Optionally insert additional variables)
  static unsigned char icount = 0;
  char TempDate[] = {'2','0','0','0','-','0','1','-','0','1',' ','0','8',':','0','0',':','0','0',' ','W','7'};
  // USER END

  switch (pMsg->MsgId) {
  // USER START (Optionally insert additional message handling)
  case WM_CREATE:
	/* ���þ۽� */
    //GUI_DispStringHCenterAt("WM_CREATE",FRAME_WIDTH >> 1,32);
	WM_SetFocus(pMsg->hWin);
	break;

  case WM_PAINT:
      _PaintFrame();

      TempDate[0] = '0' + g_tRTC.Year /1000;
      TempDate[1] = '0' + g_tRTC.Year /100 % 10;
      TempDate[2] = '0' + g_tRTC.Year /10 % 10;
      TempDate[3] = '0' + g_tRTC.Year % 10;

      TempDate[5] = '0' + g_tRTC.Mon / 10;
      TempDate[6] = '0' + g_tRTC.Mon % 10;

      TempDate[8] = '0' + g_tRTC.Day /10;
      TempDate[9] = '0' + g_tRTC.Day % 10;

      TempDate[11] = '0' + g_tRTC.Hour / 10;
      TempDate[12] = '0' + g_tRTC.Hour % 10;

      TempDate[14] = '0' + g_tRTC.Min / 10;
      TempDate[15] = '0' + g_tRTC.Min % 10;

      TempDate[17] = '0' + g_tRTC.Sec / 10;
      TempDate[18] = '0' + g_tRTC.Sec % 10;
      if((icount++) % 2 == 0) {
        TempDate[21] = ' ';
        TempDate[20] = ' ';
      }else{
        TempDate[21] = '0' + g_tRTC.Week;
        TempDate[20] = 'W';
      }


      GUI_DispStringHCenterAt(TempDate, FRAME_WIDTH >> 1,64);
      //WM_RestartTimer(pMsg->Data.v, 300);
      //WM_InvalidateWindow(pMsg->hWin);
      break;

  case WM_TIMER:
      WM_RestartTimer(pMsg->Data.v, 300);
      WM_InvalidateWindow(pMsg->hWin);
      break;

  case WM_KEY:
	switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
    {
		case GUI_KEY_BACKTAB:
			_DeleteFrame();
            _CreateFrame(&_cbShowDesktop);
			break;

        case GUI_KEY_MUSIC://��������
            break;

		case GUI_KEY_SLEEPMODE://����˯��ģʽ
            break;

        case GUI_KEY_DOWN://������С
            break;

        case GUI_KEY_UP://��������
            break;

        case KEY_DOWN_MUX://����
            break;

        case KEY_DOWN_MUX_LONG://����
            break;

        default:
            break;
    }
    break;

  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*
*********************************************************************************************************
*	�� �� ��: MainTask
*	����˵��: GUI������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MainTask(void)
{
    //WM_HTIMER hTimer;

    /* ��ʼ�� */
	GUI_Init();

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
	WM_SetCreateFlags(WM_CF_MEMDEV);
#endif

	/* ʹ�����洰��Ҳʹ���ڴ��豸 */
    WM_EnableMemdev(WM_HBKWIN);

	/* ʹ��UTF8���� */
	GUI_UC_SetEncodeUTF8();

	/* �������洰�ڵĻص����� */
	//WM_SetCallback(WM_HBKWIN, &_cbBkWindow);

	/* ���������� */
	_CreateFrame(&_cbDrawHomePage);

    /* ������ʱ�� */
    //hTimer = WM_CreateTimer(WM_HBKWIN, 0, 300, 0);

	while(1)
	{
		GUI_Delay(10);
	}
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
