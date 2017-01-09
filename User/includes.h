/*
*********************************************************************************************************
*
*	ģ������ : ͷ�ļ�����
*	�ļ����� : includes.h
*	��    �� : V1.0
*	˵    �� : ��ǰʹ��ͷ�ļ�����
*
*	�޸ļ�¼ :
*		�汾��    ����        ����     ˵��
*		V1.0    2015-08-02  Eric2013   �״η���
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef  __INCLUDES_H__
#define  __INCLUDES_H__

/*
*********************************************************************************************************
*                                         ��׼��
*********************************************************************************************************
*/
#include  <stdarg.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include <string.h>

/*
*********************************************************************************************************
*                                         ������
*********************************************************************************************************
*/
#include "stm32f2xx.h"

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
//#include "bsp_pwm.h"
//#include "bsp_uart_fifo.h"
//#include "bsp_led.h"
//#include "bsp_timer.h"
//#include "bsp_key.h"
//#include "bsp_dwt.h"

//#include "bsp_msg.h"

//#include "bsp_beep.h"

//#include "bsp_tim_pwm.h"

//#include "bsp_cpu_flash.h"
//#include "bsp_sdio_sd.h"
//#include "bsp_i2c_gpio.h"
//#include "bsp_eeprom_24xx.h"
//#include "bsp_si4730.h"
//#include "bsp_hmc5883l.h"
//#include "bsp_mpu6050.h"
//#include "bsp_bh1750.h"
//#include "bsp_bmp180.h"
//#include "bsp_wm8978.h"
//#include "bsp_gt811.h"

//#include "bsp_fsmc_sram.h"
//#include "bsp_nand_flash.h"
//#include "bsp_nor_flash.h"

//#include "LCD_RA8875.h"
//#include "LCD_SPFD5420.h"
//#include "LCD_ILI9488.h"
//#include "bsp_ra8875_port.h"
//#include "bsp_tft_lcd.h"

//#include "bsp_touch.h"


//#include "bsp_oled.h"
//#include "bsp_sim800.h"
//#include "bsp_ra8875_flash.h"

//#include "bsp_spi_bus.h"
//#include "bsp_spi_flash.h"
//#include "bsp_tm7705.h"
//#include "bsp_vs1053b.h"
//#include "bsp_tsc2046.h"

//#include "bsp_ds18b20.h"
//#include "bsp_dac8501.h"
//#include "bsp_dht11.h"

//#include "bsp_ir_decode.h"
//#include "bsp_ps2.h"

//#include "bsp_modbus.h"
//#include "bsp_rs485_led.h"
//#include "bsp_user_lib.h"

//#include "bsp_dac8501.h"
//#include "bsp_dac8562.h"

//#include "bsp_esp8266.h"
//#include "bsp_step_moto.h"

/*
*********************************************************************************************************
*                                           OS
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

/*
*********************************************************************************************************
*                                           �궨��
*********************************************************************************************************
*/
/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

#define  USE_FreeRTOS      1

#if USE_FreeRTOS == 1
	#define DISABLE_INT()    taskENTER_CRITICAL()
	#define ENABLE_INT()     taskEXIT_CRITICAL()
#else
	/* ����ȫ���жϵĺ� */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */
#endif

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/*
	EXTI9_5_IRQHandler ���жϷ�������ɢ�ڼ��������� bsp�ļ��С�
	��Ҫ���ϵ� stm32f4xx_it.c �С�

	���������б�ʾEXTI9_5_IRQHandler��ں������зŵ� stm32f4xx_it.c��
*/
#define EXTI9_5_ISR_MOVE_OUT

/*
*********************************************************************************************************
*                                        APP / BSP
*********************************************************************************************************
*/

/* ���������е��� */
extern void vSetupSysInfoTest(void);

/* �ṩ������C�ļ����õĺ��� */
//void bsp_Init(void);
//void bsp_Idle(void);
//void BSP_Tick_Init (void);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
