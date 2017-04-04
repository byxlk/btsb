/**
  ******************************************************************************
  * @file    STM32F2xx_IAP/inc/ymodem.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    02-May-2011
  * @brief   This file provides all the software function headers of the ymodem.c
  *          file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __YMODEM_H_
#define __YMODEM_H_

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
/* Exported constants --------------------------------------------------------*/
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


/* Exported types ------------------------------------------------------------*/
typedef struct{
    int           modemtype;
    int           crc_mode;
    int           nxt_num;          //下一数据块序号
    int           cur_num;          //当块序号
    int           len;
    int           rec_err;          //数据块接收状态
    unsigned char buf[PACKET_1K_SIZE];        //数据
    unsigned int  filelen;          //Ymodem可有带文件名称和长度
    unsigned char filename[FILE_NAME_LENGTH];
}modem_struct;


/* Exported functions ------------------------------------------------------- */
int8_t Ymodem_Receive (COM_PORT_E _ucPort, uint8_t *buf);
uint8_t Ymodem_Transmit (uint8_t *,const  uint8_t* , uint32_t );
uint32_t Send_Byte (COM_PORT_E _ucPort, uint8_t _pByte);
int32_t Recv_Byte (COM_PORT_E _ucPort, uint8_t *_pByte, uint32_t timeout);

#endif  /* __YMODEM_H_ */


/*******************(C)COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
