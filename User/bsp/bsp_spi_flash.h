/*
*********************************************************************************************************
*
*	模块名称 : SPI接口串行FLASH 读写模块
*	文件名称 : bsp_spi_flash.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_SPI_FLASH_H
#define _BSP_SPI_FLASH_H

#define SF_MAX_PAGE_SIZE	(4 * 1024)

/* 定义串行Flash ID */
enum
{
	SST25VF016B_ID = 0xBF2541,
	MX25L1606E_ID  = 0xC22015,
	W25Q64BV_ID    = 0xEF4017,
	MX25L25645G_ID  = 0xC22019
};

typedef struct
{
	uint32_t ChipID;		/* 芯片ID */
	char ChipName[16];		/* 芯片型号字符串，主要用于显示 */
	uint32_t TotalSize;		/* 总容量 */
	uint16_t PageSize;		/* 页面大小 */
    uint8_t  DiskInitFlag;  /* 1: 已经初始化过， 0: 还没有初始化 */
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

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
