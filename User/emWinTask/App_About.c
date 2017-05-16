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
#define ID_TEXT_1 (GUI_ID_USER + 0x02)
#define ID_TEXT_2 (GUI_ID_USER + 0x03)
#define ID_TEXT_3 (GUI_ID_USER + 0x04)
#define ID_TEXT_4 (GUI_ID_USER + 0x05)
#define ID_TEXT_5 (GUI_ID_USER + 0x06)
#define ID_TEXT_6 (GUI_ID_USER + 0x07)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x08)
#define ID_PROGBAR_1 (GUI_ID_USER + 0x09)

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
WM_HWIN  hWin_About = WM_HWIN_NULL;               /* ���ֲ��ŶԻ����� */

/*
*********************************************************************************************************
*				                         ����Ի����ʼ��ѡ��
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate_About[] =
{
    { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 1, 1, 240, 320, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "SN: SA1620123456", ID_TEXT_0, 24, 29, 181, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "VPING", ID_TEXT_1, 66, 71, 99, 42, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Sleep abc", ID_TEXT_2, 66, 132, 92, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Firmware Version: V1.0.0", ID_TEXT_3, 16, 183, 210, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Change Theme", ID_TEXT_4, 51, 226, 131, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Copyright @2017", ID_TEXT_5, 37, 260, 161, 20, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "www.vping.com", ID_TEXT_6, 43, 284, 146, 20, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 5, 50, 232, 5, 0, 0x0, 0 },
    { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_1, 2, 204, 238, 4, 0, 0x0, 0 },
};

/*
*********************************************************************************************************
*	�� �� ��: _cbDialogMusic
*	����˵��: ���ֲ��ŶԻ���ص���Ϣ
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbWinCallBack_About(WM_MESSAGE * pMsg)
{
	int NCode, Id;
    WM_HWIN hItem;
    WM_HWIN hWin = pMsg->hWin;

    //printf("[%s : %d] MsgId = %d\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);

    switch (pMsg->MsgId)
    {
        case WM_INIT_DIALOG:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
            TEXT_SetFont(hItem, GUI_FONT_20_1);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
            TEXT_SetFont(hItem, GUI_FONT_32B_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

            hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
            TEXT_SetFont(hItem, GUI_FONT_20_1);
            TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
            WM_SetFocus(pMsg->hWin);
            break;
        case WM_PRE_PAINT:
			GUI_MULTIBUF_Begin();
			break;

		case WM_POST_PAINT:
			GUI_MULTIBUF_End();
			break;
		case WM_CREATE:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            break;
		case WM_PAINT:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            break;
		case WM_NOTIFY_PARENT:
            printf("[%s : %d] WM_INIT_DIALOG(%d)\r\n",__FUNCTION__,__LINE__,pMsg->MsgId);
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;
            switch (Id)
            {
                case GUI_ID_OK:
                    if(NCode==WM_NOTIFICATION_RELEASED)
                        GUI_EndDialog(hWin, 0);
                    break;
                case GUI_ID_CANCEL:
                    if(NCode==WM_NOTIFICATION_RELEASED)
                        GUI_EndDialog(hWin, 0);
                    break;

                default:
                    break;
            }
            break;
        case WM_KEY:
            switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
            {
                case GUI_KEY_Menu:
                    printf("[%s : %d] GUI_KEY_Menu \r\n",__FUNCTION__,__LINE__);
                    GUI_EndDialog(hWin, 1);
                    break;
                case GUI_KEY_PlayPause:
                    printf("[%s : %d] GUI_KEY_PlayPause \r\n",__FUNCTION__,__LINE__);
                    break;
                case GUI_KEY_Direction_Up:
                    printf("[%s : %d] GUI_KEY_Direction_Up \r\n",__FUNCTION__,__LINE__);
				case GUI_KEY_Direction_Down:
                    printf("[%s : %d] GUI_KEY_Direction_Down \r\n",__FUNCTION__,__LINE__);
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
WM_HWIN App_About(WM_HWIN hWin)
{
#if 1
	hWin_About = GUI_CreateDialogBox(_aDialogCreate_About,
	                                GUI_COUNTOF(_aDialogCreate_About),
                                  	_cbWinCallBack_About,
	                                hWin,
	                                0,
	                                0);
#else
	hWin_About = WM_CreateWindowAsChild(0, 0,
                                  LCD_GetXSize(),
                                  LCD_GetYSize(),
                                  WM_HBKWIN,
                                  WM_CF_MOTION_X | WM_CF_SHOW | WM_CF_HASTRANS,
                                  _cbWinCallBack, 0);
#endif
    return hWin_About;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
