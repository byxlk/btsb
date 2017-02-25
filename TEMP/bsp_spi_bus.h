/*
*********************************************************************************************************
*
*	ģ������ : SPI��������
*	�ļ����� : bsp_spi_bus.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_SPI_BUS_H
#define __BSP_SPI_BUS_H

//#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */

/*
	��SPIʱ�������2��Ƶ����֧�ֲ���Ƶ��
	�����SPI1��2��ƵʱSCKʱ�� = 42M��4��ƵʱSCKʱ�� = 21M
	�����SPI3, 2��ƵʱSCKʱ�� = 21M
*/
#define SPI_SPEED_42M		SPI_BaudRatePrescaler_2
#define SPI_SPEED_21M		SPI_BaudRatePrescaler_4
#define SPI_SPEED_5_2M		SPI_BaudRatePrescaler_8
#define SPI_SPEED_2_6M		SPI_BaudRatePrescaler_16
#define SPI_SPEED_1_3M		SPI_BaudRatePrescaler_32
#define SPI_SPEED_0_6M		SPI_BaudRatePrescaler_64

void bsp_InitSPIBus(void);

void bsp_spiWriteByte0(uint8_t _ucByte);
uint8_t bsp_spiReadByte0(void);

void bsp_spiWriteByte1(uint8_t _ucByte);
uint8_t bsp_spiReadByte1(void);

uint8_t bsp_SpiBusBusy(void);

void bsp_SPI_Init(uint16_t _cr1);

void bsp_SpiBusEnter(void);
void bsp_SpiBusExit(void);
uint8_t bsp_SpiBusBusy(void);
void bsp_SetSpiSck(uint8_t _data);

//////////////////////////////////////////////////////////////////////////////////////////////
uint8_t SPI_disk_status(void);
uint8_t SPI_disk_initialize(void);

DRESULT SPI_disk_read(
       	uint8_t *buff,		/* Data buffer to store read data */
	uint32_t sector,	/* Start sector in LBA */
	uint32_t count	);	/* Number of sectors to read */

DRESULT SPI_disk_write(
       	const uint8_t *buff,		/* Data buffer to store read data */
	uint32_t sector,	/* Start sector in LBA */
	uint32_t count	);	/* Number of sectors to read */

DRESULT SPI_disk_ioctl(
	uint8_t cmd,		/* Control code */
	void *buff );	/* Buffer to send/receive control data */


#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
