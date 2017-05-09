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

/*
*********************************************************************************************************
*				                      �궨��
*********************************************************************************************************
*/
#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_TEXT_0 (GUI_ID_USER + 0x01)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x02)
#define ID_PROGBAR_1 (GUI_ID_USER + 0x03)
#define ID_TEXT_1 (GUI_ID_USER + 0x04)

/*
*********************************************************************************************************
*				                    �����ⲿ�����ͺ���
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*				                         ����
*********************************************************************************************************
*/
WM_HWIN  hWin_Bluetooth = WM_HWIN_NULL;               /* ���ֲ��ŶԻ����� */

/*
*********************************************************************************************************
*				                         ����Ի����ʼ��ѡ��
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateMusic[] =
{
    { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, -1, -1, 240, 320, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Bluetooth", ID_TEXT_0, 16, 28, 80, 20, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 6, 48, 228, 5, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_1, 10, 191, 222, 5, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Avaliable Device", ID_TEXT_1, 14, 167, 139, 20, 0, 0x0, 0 },
};

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
	int NCode, Id;
    WM_HWIN hItem;
    WM_HWIN hWin = pMsg->hWin;

    switch (pMsg->MsgId)
    {
        case WM_INIT_DIALOG:
            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
            TEXT_SetFont(hItem, GUI_FONT_20_1);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            WM_SetFocus(pMsg->hWin);
            break;
		case WM_CREATE:
            break;
		case WM_PAINT:
            break;
        case WM_PRE_PAINT:
            GUI_MULTIBUF_Begin();
            break;

        case WM_POST_PAINT:
            GUI_MULTIBUF_End();
            break;

        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_Menu:
                    GUI_EndDialog(hWin, 1);
                    break;
                case GUI_KEY_PlayPause:
                    break;
				case GUI_KEY_Direction_Down:
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
*********************************************************************************************************
*	�� �� ��: App_Music
*	����˵��: �������ֲ��ŶԻ���
*	��    ��: hWin  ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void App_Bluetooth(WM_HWIN hWin)
{
#if 1
	hWin_Bluetooth = GUI_CreateDialogBox(_aDialogCreateMusic,
	                                GUI_COUNTOF(_aDialogCreateMusic),
                                  	_cbWinCallBack,
	                                hWin,
	                                0,
	                                0);
#else
	hWin_Bluetooth = WM_CreateWindowAsChild(0, 0,
                                  LCD_GetXSize(),
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbWinCallBack, 0);
#endif
    //return hWin_Bluetooth;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
