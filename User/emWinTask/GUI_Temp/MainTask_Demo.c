#include "dialog.h"

WM_HWIN hPage[4];

void MainTask(void)
{
    GUI_Init();									//��ʼ��emWin/ucGUI
    hPage[0] = CreateFramewin0(WM_HBKWIN);		//��������,�����������汳��
    hPage[1] = CreateFramewin1(WM_HBKWIN);		//��������,�����������汳��
    hPage[2] = CreateFramewin2(hPage[1]);		//��������,��������Page 1
    hPage[3] = CreateFramewin3(WM_HBKWIN);		//��������,�����������汳��
    WM_HideWindow(hPage[1]);					//����Page 1,��ΪPage 2��Page 1���Ӵ���,�����ȻҲ��������
    WM_HideWindow(hPage[3]);					//����Page 3

    while(1) {GUI_Delay(20);}					//����GUI_Delay������ʱ20MS(����Ŀ���ǵ���GUI_Exec()����)
}
