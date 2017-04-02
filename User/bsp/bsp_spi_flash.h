/*
*********************************************************************************************************
*
*	ģ������ : SPI�ӿڴ���FLASH ��дģ��
*	�ļ����� : bsp_spi_flash.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_SPI_FLASH_H
#define _BSP_SPI_FLASH_H

#define SF_MAX_PAGE_SIZE	(4 * 1024)

/* ���崮��Flash ID */
enum
{
	SST25VF016B_ID = 0xBF2541,
	MX25L1606E_ID  = 0xC22015,
	W25Q64BV_ID    = 0xEF4017,
	MX25L25645G_ID  = 0xC22019
};

typedef struct
{
	uint32_t ChipID;		/* оƬID */
	char ChipName[16];		/* оƬ�ͺ��ַ�������Ҫ������ʾ */
	uint32_t TotalSize;		/* ������ */
	uint16_t PageSize;		/* ҳ���С */
    uint8_t  DiskInitFlag;  /* 1: �Ѿ���ʼ������ 0: ��û�г�ʼ�� */
}SFLASH_T;

void bsp_InitSFlash(void);
uint32_t sf_ReadID(void);
void sf_EraseChip(void);
void sf_EraseSector(uint32_t _uiSectorAddr);
void sf_Erase32KBlock(uint32_t _uiSectorAddr);
void sf_Erase64KBlock(uint32_t _uiSectorAddr);
void sf_PageWrite(uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);
uint8_t sf_WriteBuffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void sf_ReadBuffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);

extern SFLASH_T g_tSF;


//////////////////////////////////////////////////////////////////////////////////////////////
DSTATUS SPI_disk_status(void);
DSTATUS SPI_disk_initialize(void);

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
