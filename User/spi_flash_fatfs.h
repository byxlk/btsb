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

/* Exported macro ------------------------------------------------------------*/
#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER           (3)
#define PACKET_TRAILER          (2)
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_NAME_LENGTH        (256)
#define FILE_SIZE_LENGTH        (16)

#define MODEM_SOH                     (0x01)  /* start of 128-byte data packet */
#define MODEM_STX                     (0x02)  /* start of 1024-byte data packet */
#define MODEM_EOT                     (0x04)  /* end of transmission */
#define MODEM_ACK                     (0x06)  /* acknowledge */
#define MODEM_NAK                     (0x15)  /* negative acknowledge */
#define MODEM_CAN                     (0x18)  /* two of these in succession aborts transfer */
#define MODEM_C                       (0x43)  /* 'C' == 0x43, request 16-bit CRC */

#define USER_ABORT1                  (0x41)  /* 'A' == 0x41, abort by user */
#define USER_ABORT2                  (0x61)  /* 'a' == 0x61, abort by user */

#define NAK_TIMEOUT             (0x1000000)
#define MAX_ERRORS              (5)

#define DAT_TYPE_EOT                    (6)
#define DAT_TYPE_STX                    (5)
#define DAT_TYPE_SOH                    (4)
#define RECV_STATUS_TRANSFER_TERMINAT (3)
#define RECV_STATUS_TRANSFER_END       (2)
#define RECV_STATUS_STX_DAT_OK         (1)
#define RECV_STATUS_SOH_DAT_OK         (0)
#define RECV_STATUS_TIMEOUT             (-1)
#define RECV_STATUS_ABOART_BY_USER     (-2)
#define RECV_STATUS_CANCEL_BY_USER     (-3)
#define RECV_PACKET_STARTCODE_UNKNOW   (-4)
#define RECV_PACKET_SEQ_ERR             (-5)
#define RECV_PACKET_CRC_ERR             (-6)
#define RECV_PACKET_DAT_ERR             (-7)
#define RECV_UNKNOW_ERR                 (-8)


/* 供外部调用的函数声明 */
void DemoFatFS(void);
void MountFS(FATFS *fs, uint8_t opt);


#endif


