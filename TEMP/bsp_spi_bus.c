/*
*********************************************************************************************************
*
*	ģ������ : SPI��������
*	�ļ����� : bsp_spi_bus.h
*	��    �� : V1.1
*	˵    �� : SPI���ߵײ��������ṩSPI���á��շ����ݡ����豸����SPI֧�֡�ͨ�����л���Ӳ��SPI�������ģ��
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

#ifdef SOFT_SPI		/* ���SPI */
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

uint8_t g_spi_busy = 0;		/* SPI ���߹����־ */

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
#ifdef SOFT_SPI		/* ���SPI */
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

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* MISO ����Ϊ�������� */
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

#ifdef SOFT_SPI		/* ���SPI */
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
#ifdef SOFT_SPI		/* ���SPI */
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
#ifdef SOFT_SPI		/* ���SPI */
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
#ifdef SOFT_SPI		/* ���SPI */
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
#ifdef SOFT_SPI		/* ���SPI */
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
*	����˵��: �������ģʽ������SCK GPIO��״̬���ں���CS=0֮ǰ�����ã����ڲ�ͬ�����SPI�豸���л���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#ifdef SOFT_SPI		/* ���SPI */
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

///////////////////////////////////////////////////////////////////////////////////////////
/*
*********************************************************************************************************
*	�� �� ��: bsp_CfgSPIForSFlash
*	����˵��: ����STM32�ڲ�SPIӲ���Ĺ���ģʽ���ٶȵȲ��������ڷ���SPI�ӿڵĴ���Flash��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_CfgSPIForSFlash(void)
{
	SPI_InitTypeDef  SPI_InitStructure;

	/* ��SPIʱ�� */
	ENABLE_SPI_RCC();

	/* ����SPIӲ������ */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	/* ���ݷ���2��ȫ˫�� */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		/* STM32��SPI����ģʽ ������ģʽ */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	/* ����λ���� �� 8λ */
	/* SPI_CPOL��SPI_CPHA���ʹ�þ���ʱ�Ӻ����ݲ��������λ��ϵ��
	   ��������: ���߿����Ǹߵ�ƽ,��2�����أ������ز�������)
	*/
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;			/* ʱ�������ز������� */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		/* ʱ�ӵĵ�2�����ز������� */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			/* Ƭѡ���Ʒ�ʽ��������� */

	/* ���ò�����Ԥ��Ƶϵ�� */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BAUD;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	/* ����λ������򣺸�λ�ȴ� */
	SPI_InitStructure.SPI_CRCPolynomial = 7;			/* CRC����ʽ�Ĵ�������λ��Ϊ7�������̲��� */
	SPI_Init(SPI_FLASH, &SPI_InitStructure);

	SPI_Cmd(SPI_FLASH, DISABLE);			/* �Ƚ�ֹSPI  */

	SPI_Cmd(SPI_FLASH, ENABLE);				/* ʹ��SPI  */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_EraseSector
*	����˵��: ����ָ��������
*	��    ��:  _uiSectorAddr : ������ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void sf_EraseSector(uint32_t _uiSectorAddr)
{
	sf_WriteEnable();								/* ����дʹ������ */

	/* ������������ */
	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_SE);								/* ���Ͳ������� */
	sf_SendByte((_uiSectorAddr & 0xFF0000) >> 16);	/* ����������ַ�ĸ�8bit */
	sf_SendByte((_uiSectorAddr & 0xFF00) >> 8);		/* ����������ַ�м�8bit */
	sf_SendByte(_uiSectorAddr & 0xFF);				/* ����������ַ��8bit */
	SF_CS_HIGH();									/* ����Ƭѡ */

	sf_WaitForWriteEnd();							/* �ȴ�����Flash�ڲ�д������� */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_EraseChip
*	����˵��: ��������оƬ
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void sf_EraseChip(void)
{
	sf_WriteEnable();								/* ����дʹ������ */

	/* ������������ */
	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_BE);							/* ������Ƭ�������� */
	SF_CS_HIGH();									/* ����Ƭѡ */

	sf_WaitForWriteEnd();							/* �ȴ�����Flash�ڲ�д������� */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_PageWrite
*	����˵��: ��һ��page��д�������ֽڡ��ֽڸ������ܳ���ҳ���С��4K)
*	��    ��:  	_pBuf : ����Դ��������
*				_uiWriteAddr ��Ŀ�������׵�ַ
*				_usSize �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void sf_PageWrite(uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize)
{
	uint32_t i, j;

	if (g_tSF.ChipID == SST25VF016B_ID)
	{
		/* AAIָ��Ҫ��������ݸ�����ż�� */
		if ((_usSize < 2) && (_usSize % 2))
		{
			return ;
		}

		sf_WriteEnable();								/* ����дʹ������ */

		SF_CS_LOW();									/* ʹ��Ƭѡ */
		sf_SendByte(CMD_AAI);							/* ����AAI����(��ַ�Զ����ӱ��) */
		sf_SendByte((_uiWriteAddr & 0xFF0000) >> 16);	/* ����������ַ�ĸ�8bit */
		sf_SendByte((_uiWriteAddr & 0xFF00) >> 8);		/* ����������ַ�м�8bit */
		sf_SendByte(_uiWriteAddr & 0xFF);				/* ����������ַ��8bit */
		sf_SendByte(*_pBuf++);							/* ���͵�1������ */
		sf_SendByte(*_pBuf++);							/* ���͵�2������ */
		SF_CS_HIGH();									/* ����Ƭѡ */

		sf_WaitForWriteEnd();							/* �ȴ�����Flash�ڲ�д������� */

		_usSize -= 2;									/* ����ʣ���ֽ��� */

		for (i = 0; i < _usSize / 2; i++)
		{
			SF_CS_LOW();								/* ʹ��Ƭѡ */
			sf_SendByte(CMD_AAI);						/* ����AAI����(��ַ�Զ����ӱ��) */
			sf_SendByte(*_pBuf++);						/* �������� */
			sf_SendByte(*_pBuf++);						/* �������� */
			SF_CS_HIGH();								/* ����Ƭѡ */
			sf_WaitForWriteEnd();						/* �ȴ�����Flash�ڲ�д������� */
		}

		/* ����д����״̬ */
		SF_CS_LOW();
		sf_SendByte(CMD_DISWR);
		SF_CS_HIGH();

		sf_WaitForWriteEnd();							/* �ȴ�����Flash�ڲ�д������� */
	}
	else	/* for MX25L1606E �� W25Q64BV */
	{
		for (j = 0; j < _usSize / 256; j++)
		{
			sf_WriteEnable();								/* ����дʹ������ */

			SF_CS_LOW();									/* ʹ��Ƭѡ */
			sf_SendByte(0x02);								/* ����AAI����(��ַ�Զ����ӱ��) */
			sf_SendByte((_uiWriteAddr & 0xFF0000) >> 16);	/* ����������ַ�ĸ�8bit */
			sf_SendByte((_uiWriteAddr & 0xFF00) >> 8);		/* ����������ַ�м�8bit */
			sf_SendByte(_uiWriteAddr & 0xFF);				/* ����������ַ��8bit */

			for (i = 0; i < 256; i++)
			{
				sf_SendByte(*_pBuf++);					/* �������� */
			}

			SF_CS_HIGH();								/* ��ֹƬѡ */

			sf_WaitForWriteEnd();						/* �ȴ�����Flash�ڲ�д������� */

			_uiWriteAddr += 256;
		}

		/* ����д����״̬ */
		SF_CS_LOW();
		sf_SendByte(CMD_DISWR);
		SF_CS_HIGH();

		sf_WaitForWriteEnd();							/* �ȴ�����Flash�ڲ�д������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: sf_ReadBuffer
*	����˵��: ������ȡ�����ֽڡ��ֽڸ������ܳ���оƬ������
*	��    ��:  	_pBuf : ����Դ��������
*				_uiReadAddr ���׵�ַ
*				_usSize �����ݸ���, ���Դ���PAGE_SIZE,���ǲ��ܳ���оƬ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void sf_ReadBuffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize)
{
	/* �����ȡ�����ݳ���Ϊ0���߳�������Flash��ַ�ռ䣬��ֱ�ӷ��� */
	if ((_uiSize == 0) ||(_uiReadAddr + _uiSize) > g_tSF.TotalSize)
	{
		return;
	}

	/* ������������ */
	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_READ);							/* ���Ͷ����� */
	sf_SendByte((_uiReadAddr & 0xFF0000) >> 16);	/* ����������ַ�ĸ�8bit */
	sf_SendByte((_uiReadAddr & 0xFF00) >> 8);		/* ����������ַ�м�8bit */
	sf_SendByte(_uiReadAddr & 0xFF);				/* ����������ַ��8bit */
	while (_uiSize--)
	{
		*_pBuf++ = sf_SendByte(DUMMY_BYTE);			/* ��һ���ֽڲ��洢��pBuf�������ָ���Լ�1 */
	}
	SF_CS_HIGH();									/* ����Ƭѡ */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_CmpData
*	����˵��: �Ƚ�Flash������.
*	��    ��:  	_ucpTar : ���ݻ�����
*				_uiSrcAddr ��Flash��ַ
*				_uiSize �����ݸ���, ���Դ���PAGE_SIZE,���ǲ��ܳ���оƬ������
*	�� �� ֵ: 0 = ���, 1 = ����
*********************************************************************************************************
*/
static uint8_t sf_CmpData(uint32_t _uiSrcAddr, uint8_t *_ucpTar, uint32_t _uiSize)
{
	uint8_t ucValue;

	/* �����ȡ�����ݳ���Ϊ0���߳�������Flash��ַ�ռ䣬��ֱ�ӷ��� */
	if ((_uiSrcAddr + _uiSize) > g_tSF.TotalSize)
	{
		return 1;
	}

	if (_uiSize == 0)
	{
		return 0;
	}

	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_READ);							/* ���Ͷ����� */
	sf_SendByte((_uiSrcAddr & 0xFF0000) >> 16);		/* ����������ַ�ĸ�8bit */
	sf_SendByte((_uiSrcAddr & 0xFF00) >> 8);		/* ����������ַ�м�8bit */
	sf_SendByte(_uiSrcAddr & 0xFF);					/* ����������ַ��8bit */
	while (_uiSize--)
	{
		/* ��һ���ֽ� */
		ucValue = sf_SendByte(DUMMY_BYTE);
		if (*_ucpTar++ != ucValue)
		{
			SF_CS_HIGH();
			return 1;
		}
	}
	SF_CS_HIGH();
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: sf_NeedErase
*	����˵��: �ж�дPAGEǰ�Ƿ���Ҫ�Ȳ�����
*	��    ��:   _ucpOldBuf �� ������
*			   _ucpNewBuf �� ������
*			   _uiLen �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 0 : ����Ҫ������ 1 ����Ҫ����
*********************************************************************************************************
*/
static uint8_t sf_NeedErase(uint8_t * _ucpOldBuf, uint8_t *_ucpNewBuf, uint16_t _usLen)
{
	uint16_t i;
	uint8_t ucOld;

	/*
	�㷨��1����old ��, new ����
	      old    new
		  1101   0101
	~     1111
		= 0010   0101

	�㷨��2��: old �󷴵Ľ���� new λ��
		  0010   old
	&	  0101   new
		 =0000

	�㷨��3��: ���Ϊ0,���ʾ�������. �����ʾ��Ҫ����
	*/

	for (i = 0; i < _usLen; i++)
	{
		ucOld = *_ucpOldBuf++;
		ucOld = ~ucOld;

		/* ע������д��: if (ucOld & (*_ucpNewBuf++) != 0) */
		if ((ucOld & (*_ucpNewBuf++)) != 0)
		{
			return 1;
		}
	}
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: sf_AutoWritePage
*	����˵��: д1��PAGE��У��,�������ȷ������д���Ρ��������Զ���ɲ���������
*	��    ��:  	_pBuf : ����Դ��������
*				_uiWriteAddr ��Ŀ�������׵�ַ
*				_usSize �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 0 : ���� 1 �� �ɹ�
*********************************************************************************************************
*/
static uint8_t sf_AutoWritePage(uint8_t *_ucpSrc, uint32_t _uiWrAddr, uint16_t _usWrLen)
{
	uint16_t i;
	uint16_t j;					/* ������ʱ */
	uint32_t uiFirstAddr;		/* ������ַ */
	uint8_t ucNeedErase;		/* 1��ʾ��Ҫ���� */
	uint8_t cRet;

	/* ����Ϊ0ʱ����������,ֱ����Ϊ�ɹ� */
	if (_usWrLen == 0)
	{
		return 1;
	}

	/* ���ƫ�Ƶ�ַ����оƬ�������˳� */
	if (_uiWrAddr >= g_tSF.TotalSize)
	{
		return 0;
	}

	/* ������ݳ��ȴ����������������˳� */
	if (_usWrLen > g_tSF.PageSize)
	{
		return 0;
	}

	/* ���FLASH�е�����û�б仯,��дFLASH */
	sf_ReadBuffer(s_spiBuf, _uiWrAddr, _usWrLen);
	if (memcmp(s_spiBuf, _ucpSrc, _usWrLen) == 0)
	{
		return 1;
	}

	/* �ж��Ƿ���Ҫ�Ȳ������� */
	/* ����������޸�Ϊ�����ݣ�����λ���� 1->0 ���� 0->0, ���������,���Flash���� */
	ucNeedErase = 0;
	if (sf_NeedErase(s_spiBuf, _ucpSrc, _usWrLen))
	{
		ucNeedErase = 1;
	}

	uiFirstAddr = _uiWrAddr & (~(g_tSF.PageSize - 1));

	if (_usWrLen == g_tSF.PageSize)		/* ������������д */
	{
		for	(i = 0; i < g_tSF.PageSize; i++)
		{
			s_spiBuf[i] = _ucpSrc[i];
		}
	}
	else						/* ��д�������� */
	{
		/* �Ƚ��������������ݶ��� */
		sf_ReadBuffer(s_spiBuf, uiFirstAddr, g_tSF.PageSize);

		/* ���������ݸ��� */
		i = _uiWrAddr & (g_tSF.PageSize - 1);
		memcpy(&s_spiBuf[i], _ucpSrc, _usWrLen);
	}

	/* д��֮�����У�飬�������ȷ����д�����3�� */
	cRet = 0;
	for (i = 0; i < 3; i++)
	{

		/* ����������޸�Ϊ�����ݣ�����λ���� 1->0 ���� 0->0, ���������,���Flash���� */
		if (ucNeedErase == 1)
		{
			sf_EraseSector(uiFirstAddr);		/* ����1������ */
		}

		/* ���һ��PAGE */
		sf_PageWrite(s_spiBuf, uiFirstAddr, g_tSF.PageSize);

		if (sf_CmpData(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
		{
			cRet = 1;
			break;
		}
		else
		{
			if (sf_CmpData(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
			{
				cRet = 1;
				break;
			}

			/* ʧ�ܺ��ӳ�һ��ʱ�������� */
			for (j = 0; j < 10000; j++);
		}
	}

	return cRet;
}

/*
*********************************************************************************************************
*	�� �� ��: sf_WriteBuffer
*	����˵��: д1��������У��,�������ȷ������д���Ρ��������Զ���ɲ���������
*	��    ��:  	_pBuf : ����Դ��������
*				_uiWrAddr ��Ŀ�������׵�ַ
*				_usSize �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 1 : �ɹ��� 0 �� ʧ��
*********************************************************************************************************
*/
uint8_t sf_WriteBuffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize)
{
	uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

	Addr = _uiWriteAddr % g_tSF.PageSize;
	count = g_tSF.PageSize - Addr;
	NumOfPage =  _usWriteSize / g_tSF.PageSize;
	NumOfSingle = _usWriteSize % g_tSF.PageSize;

	if (Addr == 0) /* ��ʼ��ַ��ҳ���׵�ַ  */
	{
		if (NumOfPage == 0) /* ���ݳ���С��ҳ���С */
		{
			if (sf_AutoWritePage(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
			{
				return 0;
			}
		}
		else 	/* ���ݳ��ȴ��ڵ���ҳ���С */
		{
			while (NumOfPage--)
			{
				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, g_tSF.PageSize) == 0)
				{
					return 0;
				}
				_uiWriteAddr +=  g_tSF.PageSize;
				_pBuf += g_tSF.PageSize;
			}
			if (sf_AutoWritePage(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
			{
				return 0;
			}
		}
	}
	else  /* ��ʼ��ַ����ҳ���׵�ַ  */
	{
		if (NumOfPage == 0) /* ���ݳ���С��ҳ���С */
		{
			if (NumOfSingle > count) /* (_usWriteSize + _uiWriteAddr) > SPI_FLASH_PAGESIZE */
			{
				temp = NumOfSingle - count;

				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, count) == 0)
				{
					return 0;
				}

				_uiWriteAddr +=  count;
				_pBuf += count;

				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, temp) == 0)
				{
					return 0;
				}
			}
			else
			{
				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
				{
					return 0;
				}
			}
		}
		else	/* ���ݳ��ȴ��ڵ���ҳ���С */
		{
			_usWriteSize -= count;
			NumOfPage =  _usWriteSize / g_tSF.PageSize;
			NumOfSingle = _usWriteSize % g_tSF.PageSize;

			if (sf_AutoWritePage(_pBuf, _uiWriteAddr, count) == 0)
			{
				return 0;
			}

			_uiWriteAddr +=  count;
			_pBuf += count;

			while (NumOfPage--)
			{
				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, g_tSF.PageSize) == 0)
				{
					return 0;
				}
				_uiWriteAddr +=  g_tSF.PageSize;
				_pBuf += g_tSF.PageSize;
			}

			if (NumOfSingle != 0)
			{
				if (sf_AutoWritePage(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
				{
					return 0;
				}
			}
		}
	}
	return 1;	/* �ɹ� */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_ReadID
*	����˵��: ��ȡ����ID
*	��    ��:  ��
*	�� �� ֵ: 32bit������ID (���8bit��0����ЧIDλ��Ϊ24bit��
*********************************************************************************************************
*/
uint32_t sf_ReadID(void)
{
	uint32_t uiID;
	uint8_t id1, id2, id3;

	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_RDID);								/* ���Ͷ�ID���� */
	id1 = sf_SendByte(DUMMY_BYTE);					/* ��ID�ĵ�1���ֽ� */
	id2 = sf_SendByte(DUMMY_BYTE);					/* ��ID�ĵ�2���ֽ� */
	id3 = sf_SendByte(DUMMY_BYTE);					/* ��ID�ĵ�3���ֽ� */
	SF_CS_HIGH();									/* ����Ƭѡ */

	uiID = ((uint32_t)id1 << 16) | ((uint32_t)id2 << 8) | id3;

	return uiID;
}

/*
*********************************************************************************************************
*	�� �� ��: sf_ReadInfo
*	����˵��: ��ȡ����ID,�������������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void sf_ReadInfo(void)
{
	/* �Զ�ʶ����Flash�ͺ� */
	{
		g_tSF.ChipID = sf_ReadID();	/* оƬID */

		switch (g_tSF.ChipID)
		{
			case SST25VF016B_ID:
				strcpy(g_tSF.ChipName, "SST25VF016B");
				g_tSF.TotalSize = 2 * 1024 * 1024;	/* ������ = 2M */
				g_tSF.PageSize = 4 * 1024;			/* ҳ���С = 4K */
				break;

			case MX25L1606E_ID:
				strcpy(g_tSF.ChipName, "MX25L1606E");
				g_tSF.TotalSize = 2 * 1024 * 1024;	/* ������ = 2M */
				g_tSF.PageSize = 4 * 1024;			/* ҳ���С = 4K */
				break;

			case W25Q64BV_ID:
				strcpy(g_tSF.ChipName, "W25Q64BV");
				g_tSF.TotalSize = 8 * 1024 * 1024;	/* ������ = 8M */
				g_tSF.PageSize = 4 * 1024;			/* ҳ���С = 4K */
				break;

			default:
				strcpy(g_tSF.ChipName, "Unknow Flash");
				g_tSF.TotalSize = 2 * 1024 * 1024;
				g_tSF.PageSize = 4 * 1024;
				break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: sf_SendByte
*	����˵��: ����������һ���ֽڣ�ͬʱ��MISO���߲����������ص�����
*	��    ��:  _ucByte : ���͵��ֽ�ֵ
*	�� �� ֵ: ��MISO���߲����������ص�����
*********************************************************************************************************
*/
static uint8_t sf_SendByte(uint8_t _ucValue)
{
	/* �ȴ��ϸ�����δ������� */
	while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_TXE) == RESET);

	/* ͨ��SPIӲ������1���ֽ� */
	SPI_I2S_SendData(SPI_FLASH, _ucValue);

	/* �ȴ�����һ���ֽ�������� */
	while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_RXNE) == RESET);

	/* ���ش�SPI���߶��������� */
	return SPI_I2S_ReceiveData(SPI_FLASH);
}

/*
*********************************************************************************************************
*	�� �� ��: sf_WriteEnable
*	����˵��: ����������дʹ������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void sf_WriteEnable(void)
{
	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_WREN);								/* �������� */
	SF_CS_HIGH();									/* ����Ƭѡ */
}

/*
*********************************************************************************************************
*	�� �� ��: sf_WriteStatus
*	����˵��: д״̬�Ĵ���
*	��    ��:  _ucValue : ״̬�Ĵ�����ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void sf_WriteStatus(uint8_t _ucValue)
{

	if (g_tSF.ChipID == SST25VF016B_ID)
	{
		/* ��1������ʹ��д״̬�Ĵ��� */
		SF_CS_LOW();									/* ʹ��Ƭѡ */
		sf_SendByte(CMD_EWRSR);							/* ������� ����д״̬�Ĵ��� */
		SF_CS_HIGH();									/* ����Ƭѡ */

		/* ��2������д״̬�Ĵ��� */
		SF_CS_LOW();									/* ʹ��Ƭѡ */
		sf_SendByte(CMD_WRSR);							/* ������� д״̬�Ĵ��� */
		sf_SendByte(_ucValue);							/* �������ݣ�״̬�Ĵ�����ֵ */
		SF_CS_HIGH();									/* ����Ƭѡ */
	}
	else
	{
		SF_CS_LOW();									/* ʹ��Ƭѡ */
		sf_SendByte(CMD_WRSR);							/* ������� д״̬�Ĵ��� */
		sf_SendByte(_ucValue);							/* �������ݣ�״̬�Ĵ�����ֵ */
		SF_CS_HIGH();									/* ����Ƭѡ */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: sf_WaitForWriteEnd
*	����˵��: ����ѭ����ѯ�ķ�ʽ�ȴ������ڲ�д�������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void sf_WaitForWriteEnd(void)
{
	SF_CS_LOW();									/* ʹ��Ƭѡ */
	sf_SendByte(CMD_RDSR);							/* ������� ��״̬�Ĵ��� */
	while((sf_SendByte(DUMMY_BYTE) & WIP_FLAG) == SET);	/* �ж�״̬�Ĵ�����æ��־λ */
	SF_CS_HIGH();									/* ����Ƭѡ */
}


//////////////////////////////////////////////////////////////////////////////////////////
uint8_t SPI_disk_status(void)
{

    return 0;
}

uint8_t SPI_disk_initialize(void)
{

	bsp_InitSPIBus();
	/* ����SPIӲ���������ڷ��ʴ���Flash */
	bsp_CfgSPIForSFlash();

	sf_ReadInfo();				/* �Զ�ʶ��оƬ�ͺ� */

	SF_CS_LOW();				/* �����ʽ��ʹ�ܴ���FlashƬѡ */
	sf_SendByte(CMD_DISWR);		/* ���ͽ�ֹд�������,��ʹ�����д���� */
	SF_CS_HIGH();				/* �����ʽ�����ܴ���FlashƬѡ */

	sf_WaitForWriteEnd();		/* �ȴ�����Flash�ڲ�������� */

	sf_WriteStatus(0);			/* �������BLOCK��д���� */
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
