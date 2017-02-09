#include "bsp.h"
#include "stm32f2xx.h"


//void SystemClockInit(void)
//{
//    RCC_DeInit();
//    RCC_HSICmd(ENABLE);
//    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//�ȴ�HSIʹ�ܳɹ�


//}

/*
*********************************************************************************************************
*	�� �� ��: bsp_Init
*	����˵��: ��ʼ��Ӳ���豸��ֻ��Ҫ����һ�Ρ��ú�������CPU�Ĵ���������ļĴ�������ʼ��һЩȫ�ֱ�����
*			 ȫ�ֱ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_Init(void)
{
	/*
	 * ����ST�̼���������ļ��Ѿ�ִ����CPUϵͳʱ�ӵĳ�ʼ�������Բ����ٴ��ظ�����ϵͳʱ�ӡ�
	 * �����ļ�������CPU��ʱ��Ƶ�ʡ��ڲ�Flash�����ٶȺͿ�ѡ���ⲿSRAM FSMC��ʼ����
      *
	 * ϵͳʱ��ȱʡ����Ϊ72MHz�������Ҫ���ģ������޸� system_stm32f10x.c �ļ�
	*/

	/* ʹ��CRCУ��, ���ڿ���STemWin��ʹ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
	/* ���ȼ���������Ϊ4��������0-15����ռʽ���ȼ���0�������ȼ����������������ȼ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	bsp_InitDWT();       /* ��ʼDWT */
	bsp_RTC_InitConfig(); /* ��ʼ��RTCģ�飬����Ĭ��ʱ��:2016-01-01 08:00:00 */
	bsp_InitUart(); 	 /* ��ʼ������ */
	//bsp_InitLed(); 		 /* ��ʼLEDָʾ�ƶ˿� */
	//bsp_InitKey();		 /* ��ʼ������ */
	
	//bsp_InitI2C();       /* ����I2C���� */
	
	//bsp_InitSPIBus();	 /* ����SPI���� */
	
	//bsp_InitExtSRAM();   /* ��ʼDWT */
	LCD_InitHard();	     /* ��ʼ����ʾ��Ӳ��(����GPIO��FSMC,��LCD���ͳ�ʼ��ָ��) */
	//TOUCH_InitHard();    /* ��ʼ������ */
	
	/* �����ļ�ϵͳ */
	//result = f_mount(&fs, "0:/", 0);
	
}

