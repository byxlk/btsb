/*
*********************************************************************************************************
*
*	ģ������ : SPI��������
*	�ļ����� : bsp_spi_bus.h
*	��    �� : V1.1
*	˵    �� : SPI���ߵײ��������ṩSPI���á��շ����ݡ����豸����SPI֧�֡�ͨ�����л���Ӳ��SPI��������ģ��
*	�޸ļ�¼ :
*		�汾��  ����        ����    ˵��
*       v1.0    2015-05-18 armfly  �װ档������FLASH��TSC2046��VS1053��AD7705��ADS1256��SPI�豸������
*									���շ����ݵĺ������л��ܷ��ࡣ�������ͬ�ٶȵ��豸��Ĺ������⡣
*		V1.1	2015-05-21 armfly  Ӳ��SPIʱ��û�п���GPIOBʱ�ӣ��ѽ����
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/*
	������STM32-V4 ��������߷���
	PA5/SPI1_SCK
	PA6/SPI1_MISO
	PA7/SPI1_MOSI

	SPI1��ʱ��Դ�� APB2Periph
*/

/* ����SPI���ߵ� GPIO�˿� */
#define SPI_BUS   SPI1
#define RCC_CS 	RCC_AHB1Periph_GPIOA
#define PORT_CS	GPIOA
#define PIN_CS	GPIO_Pin_4
#define SOURCE_CS GPIO_PinSource4
#define GPIO_AF_CS GPIO_AF_SPI1

#define RCC_SCK 	RCC_AHB1Periph_GPIOA
#define PORT_SCK	GPIOA
#define PIN_SCK	GPIO_Pin_5
#define SOURCE_SCK GPIO_PinSource5
#define GPIO_AF_SCK GPIO_AF_SPI1

#define RCC_MISO 	RCC_AHB1Periph_GPIOA
#define PORT_MISO	GPIOA
#define PIN_MISO	GPIO_Pin_6
#define SOURCE_MISO GPIO_PinSource6
#define GPIO_AF_MISO GPIO_AF_SPI1

#define RCC_MOSI 	RCC_AHB1Periph_GPIOA
#define PORT_MOSI	GPIOA
#define PIN_MOSI	GPIO_Pin_7
#define SOURCE_MOSI GPIO_PinSource7
#define GPIO_AF_MOSI GPIO_AF_SPI1

#ifdef SOFT_SPI		/* ����SPI */
	#define SCK_0()		PORT_SCK->BRR = PIN_SCK
	#define SCK_1()		PORT_SCK->BSRR = PIN_SCK

	#define MOSI_0()	PORT_MOSI->BRR = PIN_MOSI
	#define MOSI_1()	PORT_MOSI->BSRR = PIN_MOSI

	#define MISO_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_MISO, PIN_MISO) == Bit_SET)
#endif

#ifdef HARD_SPI
	#define SPI_HARD	SPI1
	#define RCC_SPI		RCC_APB2Periph_SPI1
	
	/* SPI or I2S mode selection masks */
	#define SPI_Mode_Select      ((uint16_t)0xF7FF)
	#define I2S_Mode_Select      ((uint16_t)0x0800) 
	
	/* SPI registers Masks */
	#define CR1_CLEAR_Mask       ((uint16_t)0x3040)
	#define I2SCFGR_CLEAR_Mask   ((uint16_t)0xF040)

	/* SPI SPE mask */
	#define CR1_SPE_Set          ((uint16_t)0x0040)
	#define CR1_SPE_Reset        ((uint16_t)0xFFBF)

	/* Deselect sFLASH: Chip Select pin high */
	#define bsp_SPI_CS_HIGH() GPIO_SetBits(PORT_CS, PIN_CS)

	/* Select Flash: Chip Select pin low */
	#define bsp_SPI_CS_LOW() GPIO_ResetBits(PORT_CS, PIN_CS)
#endif

uint8_t g_spi_busy = 0;		/* SPI ���߹�����־ */

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitSPIBus
*	����˵��: ����SPI���ߡ� ֻ���� SCK�� MOSI�� MISO���ߵ����á�������ƬѡCS��Ҳ����������оƬ���е�INT��BUSY��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitSPIBus(void)
{
#ifdef SOFT_SPI		/* ����SPI */
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_SCK | RCC_MOSI | RCC_MISO, ENABLE);	

	/* ����SPI����SCK��MISO �� MOSIΪ��������ģʽ */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* �������ģʽ */
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_MOSI;
	GPIO_Init(PORT_MOSI, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		/* MISO ����Ϊ�������� */
	GPIO_InitStructure.GPIO_Pin = PIN_MISO;
	GPIO_Init(PORT_MISO, &GPIO_InitStructure);
#endif

#ifdef HARD_SPI		/* Ӳ��SPI */
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;	

	/* ���� SPI ʱ�� */
	//RCC_APB2PeriphClockCmd(RCC_SPI, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* ʹ�� GPIO ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_SCK | RCC_MOSI | RCC_MISO, ENABLE);	

	/* ���� SPI����SCK��MISO �� MOSIΪ��������ģʽ */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = PIN_SCK;	
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);
	GPIO_PinAFConfig(PORT_SCK,SOURCE_SCK, GPIO_AF_SCK);
	
	GPIO_InitStructure.GPIO_Pin = PIN_MISO;	
	GPIO_Init(PORT_MISO, &GPIO_InitStructure);
	GPIO_PinAFConfig(PORT_MISO,SOURCE_MISO, GPIO_AF_MISO);

	GPIO_InitStructure.GPIO_Pin = PIN_MOSI;	
	GPIO_Init(PORT_MOSI, &GPIO_InitStructure);
	GPIO_PinAFConfig(PORT_MOSI,SOURCE_MOSI, GPIO_AF_MOSI);

	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);

	bsp_SPI_CS_HIGH();

	/*!< SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_BUS, &SPI_InitStructure);

	/*!< Enable the sFLASH_SPI  */
	SPI_Cmd(SPI_BUS, ENABLE);		/* ʹ��SPI  */
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SPI_Init
*	����˵��: ����STM32�ڲ�SPIӲ���Ĺ���ģʽ�� �򻯿⺯�������ִ��Ч�ʡ� ������SPI�ӿڼ��л���
*	��    ��: _cr1 �Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#ifdef HARD_SPI		/* Ӳ��SPI */
void bsp_SPI_Init(uint16_t _cr1)
{
	SPI_HARD->CR1 = ((SPI_HARD->CR1 & CR1_CLEAR_Mask) | _cr1);
	  
	//SPI_Cmd(SPI_HARD, DISABLE);			/* �Ƚ�ֹSPI  */	    
    SPI_HARD->CR1 &= CR1_SPE_Reset;	/* Disable the selected SPI peripheral */

	//SPI_Cmd(SPI_HARD, ENABLE);			/* ʹ��SPI  */		    
    SPI_HARD->CR1 |= CR1_SPE_Set;	  /* Enable the selected SPI peripheral */
}
#endif

#ifdef SOFT_SPI		/* ����SPI */
/*
*********************************************************************************************************
*	�� �� ��: bsp_SpiDelay
*	����˵��: ʱ���ӳ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_spiDelay(void)
{
	uint32_t i;

	for (i = 0; i < 2; i++);
}
#endif

/*
*********************************************************************************************************
*	�� �� ��: bsp_spiWrite0
*	����˵��: ��SPI���߷���һ���ֽڡ�SCK�����زɼ�����, SCK����ʱΪ�͵�ƽ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_spiWriteByte0(uint8_t _ucByte)
{
#ifdef SOFT_SPI		/* ����SPI */
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			MOSI_1();
		}
		else
		{
			MOSI_0();
		}
		bsp_spiDelay();
		SCK_1();
		_ucByte <<= 1;
		bsp_spiDelay();
		SCK_0();
	}
	bsp_spiDelay();
#endif

#ifdef HARD_SPI		/* Ӳ��SPI */
	/* �ȴ����ͻ������� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* ����һ���ֽ� */
	SPI_I2S_SendData(SPI1, _ucByte);

	/* �ȴ����ݽ������ */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* ��ȡ���յ������� */
	SPI_I2S_ReceiveData(SPI1);
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_spiRead0
*	����˵��: ��SPI���߽���8��bit���ݡ� SCK�����زɼ�����, SCK����ʱΪ�͵�ƽ��
*	��    ��: ��
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t bsp_spiReadByte0(void)
{
#ifdef SOFT_SPI		/* ����SPI */
	uint8_t i;
	uint8_t read = 0;

	for (i = 0; i < 8; i++)
	{
		read = read<<1;

		if (MISO_IS_HIGH())
		{
			read++;
		}
		SCK_1();
		bsp_spiDelay();
		SCK_0();
		bsp_spiDelay();
	}
	return read;
#endif

#ifdef HARD_SPI		/* Ӳ��SPI */
	uint8_t read;

	/* �ȴ����ͻ������� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* ����һ���ֽ� */
	SPI_I2S_SendData(SPI1, 0);

	/* �ȴ����ݽ������ */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* ��ȡ���յ������� */
	read = SPI_I2S_ReceiveData(SPI1);

	/* ���ض��������� */
	return read;
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_spiWrite1
*	����˵��: ��SPI���߷���һ���ֽڡ�  SCK�����زɼ�����, SCK����ʱΪ�ߵ�ƽ
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_spiWriteByte1(uint8_t _ucByte)
{
#ifdef SOFT_SPI		/* ����SPI */
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			MOSI_1();
		}
		else
		{
			MOSI_0();
		}
		SCK_0();
		_ucByte <<= 1;
		bsp_spiDelay();
		SCK_1();				/* SCK�����زɼ�����, SCK����ʱΪ�ߵ�ƽ */
		bsp_spiDelay();
	}
#endif

#ifdef HARD_SPI		/* Ӳ��SPI */
	/* �ȴ����ͻ������� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* ����һ���ֽ� */
	SPI_I2S_SendData(SPI1, _ucByte);

	/* �ȴ����ݽ������ */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* ��ȡ���յ������� */
	SPI_I2S_ReceiveData(SPI1);
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_spiRead1
*	����˵��: ��SPI���߽���8��bit���ݡ�  SCK�����زɼ�����, SCK����ʱΪ�ߵ�ƽ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t bsp_spiReadByte1(void)
{
#ifdef SOFT_SPI		/* ����SPI */
	uint8_t i;
	uint8_t read = 0;

	for (i = 0; i < 8; i++)
	{
		SCK_0();
		bsp_spiDelay();
		read = read << 1;
		if (MISO_IS_HIGH())
		{
			read++;
		}
		SCK_1();
		bsp_spiDelay();
	}
	return read;
#endif

#ifdef HARD_SPI		/* Ӳ��SPI */
	uint8_t read;

	/* �ȴ����ͻ������� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* ����һ���ֽ� */
	SPI_I2S_SendData(SPI1, 0);

	/* �ȴ����ݽ������ */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* ��ȡ���յ������� */
	read = SPI_I2S_ReceiveData(SPI1);

	/* ���ض��������� */
	return read;
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SpiBusEnter
*	����˵��: ռ��SPI����
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ��æ  1��ʾæ
*********************************************************************************************************
*/
void bsp_SpiBusEnter(void)
{
	g_spi_busy = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SpiBusExit
*	����˵��: �ͷ�ռ�õ�SPI����
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ��æ  1��ʾæ
*********************************************************************************************************
*/
void bsp_SpiBusExit(void)
{
	g_spi_busy = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SpiBusBusy
*	����˵��: �ж�SPI����æ�������Ǽ������SPIоƬ��Ƭѡ�ź��Ƿ�Ϊ1
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ��æ  1��ʾæ
*********************************************************************************************************
*/
uint8_t bsp_SpiBusBusy(void)
{
	return g_spi_busy;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetSpiSck
*	����˵��: ��������ģʽ������SCK GPIO��״̬���ں���CS=0֮ǰ�����ã����ڲ�ͬ�����SPI�豸���л���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#ifdef SOFT_SPI		/* ����SPI */
void bsp_SetSpiSck(uint8_t _data)
{
	if (_data == 0)
	{
		SCK_0();
	}
	else
	{
		SCK_1();
	}
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
uint8_t SPI_disk_status(void)
{

    return 0;
}

uint8_t SPI_disk_initialize(void)
{

    return 0;
}

DRESULT SPI_disk_read(
    uint8_t *buff,		/* Data buffer to store read data */
	uint32_t sector,	/* Start sector in LBA */
	uint32_t count	)	/* Number of sectors to read */
{

    return RES_OK;
}

DRESULT SPI_disk_write(
   	const uint8_t *buff,		/* Data buffer to store read data */
	uint32_t sector,	/* Start sector in LBA */
	uint32_t count	)	/* Number of sectors to read */
{

    return RES_OK;
}

DRESULT SPI_disk_ioctl(
	uint8_t cmd,		/* Control code */
	void *buff)		/* Buffer to send/receive control data */
{

    return RES_OK;
}
/***************************** ���������� www.armfly.com (END OF FILE) *********************************/