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
*	                                  ���ڱ��ļ��ĵ���
*********************************************************************************************************
*/
#if 0
	#define printf_audiodbg printf
#else
	#define printf_audiodbg(...)
#endif


/*
*********************************************************************************************************
*				                      �궨��
*********************************************************************************************************
*/
#define ID_WINDOW_0 		(GUI_ID_USER + 0x00)
#define ID_IMAGE_0  		(GUI_ID_USER + 0x01)
#define ID_SLIDER_0 		(GUI_ID_USER + 0x02)
#define ID_BUTTON_0 		(GUI_ID_USER + 0x03)
#define ID_BUTTON_1 		(GUI_ID_USER + 0x04)
#define ID_BUTTON_2 		(GUI_ID_USER + 0x05)
#define ID_BUTTON_3 		(GUI_ID_USER + 0x06)
#define ID_BUTTON_4 		(GUI_ID_USER + 0x07)
#define ID_BUTTON_5 		(GUI_ID_USER + 0x08)
#define ID_BUTTON_6 		(GUI_ID_USER + 0x09)
#define ID_SLIDER_1  	    (GUI_ID_USER + 0x0A)
#define ID_TEXT_0 			(GUI_ID_USER + 0x0B)
#define ID_TEXT_1 			(GUI_ID_USER + 0x0C)
#define ID_TEXT_2 			(GUI_ID_USER + 0x0D)
#define ID_TEXT_3 			(GUI_ID_USER + 0x0E)

#define ID_BUTTON_7 		(GUI_ID_USER + 0x0F)

/* ��ͬ��ʱ���ľ�� */
#define ID_TIMER_SPEC       0
#define ID_TIMER_PROCESS    1

/* ����·���ַ����� */
#define MusicPathSzie       100


/*
*********************************************************************************************************
*				                    �����ⲿ�����ͺ���
*********************************************************************************************************
*/
/* ����һ��֧��5����Ϣ����Ϣ���� */
//extern os_mbx_declare (mailbox, 5);
//extern MusicMsg_T s_tMusicMsg;  /* ���ڸ����ֲ���������Ϣ�������������ͺ͸���·�� */


/*
*********************************************************************************************************
*				                         ����
*********************************************************************************************************
*/
const char s_MusicPathDir[] = {"M0:\\Music\\"};  /* �洢���и�����ŵ�·�� */
WM_HWIN  hWin_HomePage = WM_HWIN_NULL;               /* ���ֲ��ŶԻ����� */

/*
*********************************************************************************************************
*				                         ����Ի����ʼ��ѡ��
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateMusic[] =
{

	{ BUTTON_CreateIndirect, "HomePage",  ID_BUTTON_6,    0,   0,  100, 100, 0, 0, 0 },

	{ BUTTON_CreateIndirect, "MusicSet",   ID_BUTTON_7,   0,   110,  100, 100, 0, 0, 0 },

	/* ������С */
	{ SLIDER_CreateIndirect, "Speaker",    ID_SLIDER_1, 10, 220, 200, 20, 0, 0x0, 0  },

};

/*
*********************************************************************************************************
*	�� �� ��: Caculate_RTC
*	����˵��: ��ʾRTCʱ��
*	��    �Σ�pMsg ָ�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#if 0
//extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_InitTypeDef  RTC_InitStructure;
extern RTC_AlarmTypeDef RTC_AlarmStructure;
//extern RTC_DateTypeDef  RTC_DateStructure;
static void Caculate_RTC(WM_MESSAGE * pMsg)
{
	  char buf[30];
	  WM_HWIN hWin = pMsg->hWin;

	  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

	  sprintf(buf,
	          "%0.2d:%0.2d:%0.2d",
			  RTC_TimeStructure.RTC_Hours,
			  RTC_TimeStructure.RTC_Minutes,
			  RTC_TimeStructure.RTC_Seconds);
	 TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_10), buf);

	  sprintf(buf,
	          "20%0.2d/%0.2d/%0.2d",
			  RTC_DateStructure.RTC_Year,
			  RTC_DateStructure.RTC_Month,
			  RTC_DateStructure.RTC_Date);
	  TEXT_SetText(WM_GetDialogItem(hWin,ID_TEXT_9), buf);
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: _cbDialogMusic
*	����˵��: ���ֲ��ŶԻ���ص���Ϣ
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbWinCallBack(WM_MESSAGE * pMsg)
{
	static  WM_HTIMER hTimerSpec;

	(void)hTimerSpec;

	switch (pMsg->MsgId)
	{
		/* �Ի����ʼ����Ϣ */
		case WM_INIT_DIALOG:
            WM_SetFocus(pMsg->hWin);
            printf("_cbDialogMainPage -> WM_CREATE\n");
			break;

        //case WM_CREATE:
        //    WM_SetFocus(pMsg->hWin);
        //    printf("_cbDialogMainPage -> WM_CREATE\n");
        //    break;

		/* ��ʱ���ص���Ϣ */
		case WM_TIMER:
			//Id = WM_GetTimerId(pMsg->Data.v);
	        break;

		/* �ػ���Ϣ����ʹ���˶໺�� */
		case WM_PRE_PAINT:
			GUI_MULTIBUF_Begin();
		    break;

		case WM_PAINT:
			//GUI_SetBkColor(GUI_WHITE);
			//GUI_Clear();
			break;

		case WM_POST_PAINT:
			GUI_MULTIBUF_End();
		    break;

		case WM_NOTIFY_PARENT:
			//Id    = WM_GetId(pMsg->hWinSrc);
			//NCode = pMsg->Data.v;
			break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_BACKTAB:
                    GUI_EndDialog(pMsg->hWin, 0);
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

		default:
			WM_DefaultProc(pMsg);
			break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: App_Music
*	����˵��: �������ֲ��ŶԻ���
*	��    ��: hWin  ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void App_HomePage(WM_HWIN hWin)
{
	hWin_HomePage = GUI_CreateDialogBox(_aDialogCreateMusic,
	                                GUI_COUNTOF(_aDialogCreateMusic),
                                  	_cbWinCallBack,
	                                hWin,
	                                0,
	                                0);
    //return hWin_HomePage;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
