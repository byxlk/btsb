/*
*********************************************************************************************************
*
*	模块名称 : SPI Flash的FatFS 演示模块。
*	文件名称 : demo_spi_flash_fatfs.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _DEMO_SPI_FLASH_FATFS_H
#define _DEMO_SPI_FLASH_FATFS_H

/* 供外部调用的函数声明 */
void DemoFatFS(void);
void MountFS(FATFS *fs, uint8_t opt);


#endif


