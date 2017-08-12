#include "dialog.h"

WM_HWIN hPage[4];

void MainTask(void)
{
    GUI_Init();									//初始化emWin/ucGUI
    hPage[0] = CreateFramewin0(WM_HBKWIN);		//创建窗体,父窗体是桌面背景
    hPage[1] = CreateFramewin1(WM_HBKWIN);		//创建窗体,父窗体是桌面背景
    hPage[2] = CreateFramewin2(hPage[1]);		//创建窗体,父窗体是Page 1
    hPage[3] = CreateFramewin3(WM_HBKWIN);		//创建窗体,父窗体是桌面背景
    WM_HideWindow(hPage[1]);					//隐藏Page 1,因为Page 2是Page 1的子窗口,因此自然也跟着隐藏
    WM_HideWindow(hPage[3]);					//隐藏Page 3

    while(1) {GUI_Delay(20);}					//调用GUI_Delay函数延时20MS(最终目的是调用GUI_Exec()函数)
}
