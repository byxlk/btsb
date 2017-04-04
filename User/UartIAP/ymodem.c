 /**
  ******************************************************************************
  * @file    STM32F2xx_IAP/src/ymodem.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    02-May-2011
  * @brief   This file provides all the software functions related to the ymodem
  *          protocol.
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

/** @addtogroup STM32F2xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "common.h"
#include "ymodem.h"
#include "string.h"
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t FileName[];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
********************************************************************************
  * @函数名称    UpdateCRC16
  * @函数说明   更新输入数据的CRC校验
  * @输入参数   crcIn
                byte
  * @输出参数   无
  * @返回参数   CRC校验值
********************************************************************************
**/
uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
    uint32_t crc = crcIn;
    uint32_t in = byte | 0x100;

    do
    {
        crc <<= 1;
        in <<= 1;
        if(in & 0x100)
            ++crc;
        if(crc & 0x10000)
            crc ^= 0x1021;
    }

    while(!(in & 0x10000));

    return crc & 0xffffu;
}


/*
********************************************************************************
  * @函数名称    UpdateCRC16
  * @函数说明   更新输入数据的CRC校验
  * @输入参数   data ：数据
                size ：长度
  * @输出参数   无
  * @返回参数   CRC校验值
********************************************************************************
*/
uint16_t Cal_CRC16(const uint8_t* data, uint32_t size)
{
    uint32_t crc = 0;
    const uint8_t* dataEnd = data + size;

    while(data < dataEnd)
    crc = UpdateCRC16(crc, *data++);

    crc = UpdateCRC16(crc, 0);
    crc = UpdateCRC16(crc, 0);

    return crc & 0xffffu;
}

/*
********************************************************************************
  * @函数名称    CalChecksum
  * @函数说明   计算YModem数据包的总大小
  * @输入参数   data ：数据
                size ：长度
  * @输出参数   无
  * @返回参数   数据包的总大小
********************************************************************************
*/
uint8_t CalChecksum(const uint8_t* data, uint32_t size)
{
    uint32_t sum = 0;
    const uint8_t* dataEnd = data + size;

    while(data < dataEnd )
        sum += *data++;

    return (sum & 0xffu);
}

/*
********************************************************************************
  * @brief  Receive byte from sender
  * @param  c: Character
  * @param  timeout: Timeout
  * @retval 0: Byte received
  *        -1: Timeout
********************************************************************************
*/
int32_t Recv_Byte (COM_PORT_E _ucPort, uint8_t *_pByte, uint32_t timeout)
{
    while (timeout-- > 0)
    {
        if(comGetChar(_ucPort, _pByte) == 1)
        {
            return 0;
        }
    }

    return -1;
}

/*
********************************************************************************
  * @brief  Send a byte
  * @param  c: Character
  * @retval 0: Byte sent
********************************************************************************
*/
uint32_t Send_Byte (COM_PORT_E _ucPort, uint8_t _pByte)
{
    comSendChar(_ucPort, _pByte);

    return 0;
}

/******************************************************************************
*
  * @函数名称    YmodemRecvData
  * @函数说明   使用Ymodem协议接收数据
  * @输入参数   data ：数据
                size ：长度
  * @输出参数   无
  * @返回参数   数据包的总大小
*******************************************************************************
*/
int Ymodem_RecvData(void)
{
    int i=0;
    char ErrorNum=0            //错误计数
    char YmodemState=0;
    unsigned char TempChar=0   //接收数据
    int crcvalue=0;
    int len=0;
    int count=0;

    while(1)
    {
        switch(YmodemState)
        {
            case 0: //通信起始阶段
                SerialPutChar(CRC16);                //发起始信号
                //每次等待0.2s钟，发生超时重发“C”
                if(GetKeyC(1000000,&TempChar)==0)
                {
                    if(TempChar==SOH )
                    {
                        GetKey(&TempChar);
                        if(TempChar!=0x00)return ;          //不是00序号。
                        GetKey(&TempChar);
                        if ( TempChar != 0xFF )return ;     //不是00序号补码。
                        for ( i=0; i<128; i++ )
                        {                                //接收数据包0，共128字节
                            GetKey((UserBuf+i));
                        }
                        GetKey(&TempChar);
                        GetKey(&TempChar);  //丢弃CRC校验，暂时不想实现。

                        SerialPutChar(ACK);             //发送确认信号。
                        YmodemState=1;  //状态切换到数据传输状态
                        for ( i=0; i<128; i++ )
                        {                        //接收数据包0，共128字节
                            file_name[i]=0;
                        }
                        for ( i=0; (i<128)&&(UserBuf[i]); i++ )
                        {                          //接收数据包0，共128字节
                            file_name[i]=UserBuf[i];
                        }
                        Str2Int((UserBuf+i+1),&PackLen);

                        SerialPutChar (CRC16);   //再发一个C，正式启动数据传输
                    }
                }
                break;
            case 1:        //数据传输阶段
                GetKey(&TempChar);
                switch(TempChar)
                {
                    case EOT: YmodemState = EOT;
                        SerialPutChar ( ACK );
                        continue;
                    case SOH:
                        len = 128;
                        break;
                    case STX:
                        len = 1024;
                        break;
                    case ABORT1:
                        Serial_PutString("用户取消文件传输!\r\n");
                        return PackLen;
                    case ABORT2:
                        Serial_PutString("用户取消文件传输!\r\n");
                        return PackLen;
                    default :return PackLen
;
                }  //end of switch (StartChar)
                GetKey(&TempChar);
                GetKey(&TempChar);
                for(i=0;i<len;i++)
                {      //接收整个数据包
                    GetKey((UserBuf+i));
                }
                //CRC桥验
                GetKey(&TempChar);
                crcvalue=TempChar;
                crcvalue<<=8;
                GetKey(&TempChar);
                crcvalue|=TempChar;

                if(Cal_CRC16(UserBuf,len)!=crcvalue)
                {
                    ErrorNum+=1;
                }

                if(ErrorNum==0)
                {
                    SerialPutChar(ACK);
                    count+=len;
                }
                else SerialPutChar(NAK);//接收发现错误，要求重发。
                    break;
            case EOT:   //结束传输阶段
                SerialPutChar(CRC16);
                GetKey(&TempChar);//接收起始字符。
                if(TempChar==SOH)
                {
                    GetKey(&TempChar);
                    if(TempChar!=0x00)return PackLen;            //不是00序号。
                    GetKey(&TempChar);
                    if(TempChar!=0xFF)return PackLen;    //不是00序号补码。
                    for(i=0;i<128;i++)
                    {
                        GetKey((UserBuf+i));
                    }
                    //CRC桥验
                    GetKey(&TempChar);
                    crcvalue=TempChar;
                    crcvalue<<=8;
                    GetKey(&TempChar);
                    crcvalue|=TempChar;

					if(Cal_CRC16(UserBuf,128)!=crcvalue)
                    {
                        ErrorNum+=1;
                    }
                    if(ErrorNum==0)
                    {
                        SerialPutChar(ACK);
                        return PackLen;
                    }
                    else SerialPutChar(NAK);//接收发现错误，要求重发。
                    break;
                }
            default:
                return PackLen;
        }
    }
    return PackLen;
}


/*
********************************************************************************
  * @brief  Receive a packet from sender
  * @param  data
  * @param  length
  * @param  timeout
  *     0: end of transmission
  *    -1: abort by sender
  *    >0: packet length
  * @retval 0: normally return
  *        -1: timeout or packet error
  *         1: abort by user
********************************************************************************
*/
static int8_t Receive_Packet (COM_PORT_E _ucPort, uint8_t *rbuf, int32_t *length, uint32_t timeout)
{
    uint16_t i;
    uint16_t packet_size;
    uint8_t _pByte;
    uint8_t _pSeqNo;
    uint16_t _pCrcDat = 0;
    *length = 0;

    /* Recive first byte data */
    if (Recv_Byte(_ucPort, &_pByte, timeout) != 0)
    {
        printf("COM%d recv byte timeout.\r\n",_ucPort);
        return RECV_STATUS_TIMEOUT;
    }

    /* sparse the first byte data */
    switch (_pByte)
    {
        case MODEM_SOH:
        case MODEM_STX:
        {
            packet_size = (_pByte == MODEM_SOH)? PACKET_SIZE : PACKET_1K_SIZE;
            //rbuf[0] = _pByte;

            /* check sequence number and it complement */
            if (Recv_Byte(_ucPort, &_pByte, timeout) == 0)
            {
                //rbuf[PACKET_SEQNO_INDEX] = _pByte;
                _pSeqNo = _pByte;
            }
            else return RECV_STATUS_TIMEOUT;

            if (Recv_Byte(_ucPort, &_pByte, timeout) == 0)
            {
                //if(rbuf[PACKET_SEQNO_INDEX] != ((_pByte ^ 0xff) & 0xff))
                if(_pSeqNo != ((_pByte ^ 0xff) & 0xff))
                    return RECV_PACKET_SEQ_ERR;
                //rbuf[PACKET_SEQNO_COMP_INDEX] = _pByte;
            }
            else return RECV_STATUS_TIMEOUT;

            /* recv data and save it to buffer */
            //for (i = 3; i < (packet_size + PACKET_OVERHEAD); i ++)
            for (i = 0; i < packet_size; i ++)
            {
                if (Recv_Byte(_ucPort, &_pByte, timeout) == 0)
                    rbuf[i] = _pByte;
                else
                    return RECV_STATUS_TIMEOUT;
            }

            /* data CRC verfiy */
            if (Recv_Byte(_ucPort, &_pByte, timeout) == 0)
                _pCrcDat |= _pByte << 8;
            else return RECV_STATUS_TIMEOUT;
            if (Recv_Byte(_ucPort, &_pByte, timeout) == 0)
                _pCrcDat |= _pByte;
            else return RECV_STATUS_TIMEOUT;

            *length = packet_size;
            break;
        }
        case MODEM_EOT:
            return RECV_STATUS_TRANSFER_END;
        case MODEM_CAN:
        {
            if ((Recv_Byte(_ucPort, &_pByte, timeout) == 0) && (_pByte == MODEM_CAN))
            {
                *length = -1;
                return RECV_STATUS_CANCEL_BY_USER;
            }
            else
            {
                return RECV_UNKNOW_ERR;
            }
        }
        case USER_ABORT1:
        case USER_ABORT2:
            return RECV_STATUS_ABOART_BY_USER;
        default:
            return RECV_PACKET_STARTCODE_UNKNOW;
    }

    return (_pByte == MODEM_SOH)? RECV_STATUS_SOH_DAT_OK : RECV_STATUS_STX_DAT_OK;
}
#if 1
/**
  * @brief  Receive a file using the ymodem protocol.
  * @param  buf: Address of the first byte.
  * @retval The size of the file.
  */
int8_t Ymodem_Receive (COM_PORT_E _ucPort, uint8_t *buf)
{
    //uint16_t i = 0;
    //uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
    int8_t rp_result = 0;
    int32_t packet_length = 0;

    /* Step1 Recvier send C */
    Send_Byte(_ucPort, MODEM_C);

    /* Get recive data and save it to buffer */
    rp_result = Receive_Packet(_ucPort, //data recive and send serial port
                                buf,    //buffer
                                &packet_length,
                                NAK_TIMEOUT); // recive data timeout

    /* Sparse recive data result */
    switch (rp_result)
    {
        case RECV_STATUS_SOH_DAT_OK:
        case RECV_STATUS_STX_DAT_OK:
            Send_Byte(_ucPort, MODEM_ACK);
            break;
        case RECV_STATUS_TRANSFER_END:
            Send_Byte(_ucPort, MODEM_ACK);
            break;
        case RECV_STATUS_TIMEOUT:
        case RECV_PACKET_STARTCODE_UNKNOW:
        case RECV_PACKET_SEQ_ERR:
        case RECV_PACKET_DAT_ERR:
        case RECV_PACKET_CRC_ERR:
            Send_Byte(_ucPort, MODEM_NAK);
            break;
        case RECV_STATUS_ABOART_BY_USER:
        case RECV_STATUS_CANCEL_BY_USER:
            Send_Byte(_ucPort, MODEM_CAN);
            Send_Byte(_ucPort, MODEM_CAN);
            break;
        default:
            break;
    }
    return rp_result;
}
#else
/**
  * @brief  Receive a file using the ymodem protocol.
  * @param  buf: Address of the first byte.
  * @retval The size of the file.
  */
int32_t Ymodem_Receive (COM_PORT_E _ucPort, uint8_t *buf)
{
    uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
    uint8_t file_size[FILE_SIZE_LENGTH];
    uint8_t *file_ptr;
    uint8_t *buf_ptr;

    int32_t i;
    int32_t packet_length;
    int32_t session_done;
    int32_t file_done;
    int32_t packets_received;
    int32_t errors;
    int32_t session_begin;
    int32_t size = 0;
    uint32_t flashdestination;
    uint32_t ramsource;

    /* Initialize flashdestination variable */
    flashdestination = APPLICATION_ADDRESS;

    for (session_done = 0, errors = 0, session_begin = 0; ;)
    {
        for (packets_received = 0, file_done = 0, buf_ptr = buf; ;)
        {
            switch (Receive_Packet(_ucPort, packet_data, &packet_length, NAK_TIMEOUT))
            {
                case 0:
                    errors = 0;
                    switch (packet_length)
                    {
                        case - 1:/* Abort by sender */
                            Send_Byte(_ucPort, MODEM_ACK);
                            return 0;
                        case 0:/* End of transmission */
                            Send_Byte(_ucPort, MODEM_ACK);
                            file_done = 1;
                            break;
                        default:/* Normal packet */
                            if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff))
                            {
                                Send_Byte(_ucPort, MODEM_NAK);
                            }
                            else
                            {
                                if (packets_received == 0)
                                {
                                    /* Filename packet */
                                    if (packet_data[PACKET_HEADER] != 0)
                                    {
                                        /* Filename packet has valid data */
                                        for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < FILE_NAME_LENGTH);)
                                        {
                                            FileName[i++] = *file_ptr++;
                                        }
                                        FileName[i++] = '\0';
                                        for (i = 0, file_ptr ++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);)
                                        {
                                            file_size[i++] = *file_ptr++;
                                        }
                                        file_size[i++] = '\0';
                                        Str2Int(file_size, &size);

                                        /* Test the size of the image to be sent */
                                        /* Image size is greater than Flash size */
                                        if (size > (USER_FLASH_SIZE + 1))
                                        {
                                            /* End session */
                                            Send_Byte(CA);
                                            Send_Byte(CA);
                                            return -1;
                                        }
                                        /* erase user application area */
                                        FLASH_If_Erase(APPLICATION_ADDRESS);
                                        Send_Byte(ACK);
                                        Send_Byte(CRC16);
                                    }
                                    /* Filename packet is empty, end session */
                                    else
                                    {
                                        Send_Byte(ACK);
                                        file_done = 1;
                                        session_done = 1;
                                        break;
                                    }
                                }
                                /* Data packet */
                                else
                                {
                                    memcpy(buf_ptr, packet_data + PACKET_HEADER, packet_length);
                                    ramsource = (uint32_t)buf;

                                    /* Write received data in Flash */
                                    if (FLASH_If_Write(&flashdestination, (uint32_t*) ramsource, (uint16_t) packet_length/4)  == 0)
                                    {
                                        Send_Byte(ACK);
                                    }
                                    else /* An error occurred while writing to Flash memory */
                                    {
                                        /* End session */
                                        Send_Byte(CA);
                                        Send_Byte(CA);
                                        return -2;
                                    }
                                }
                                packets_received ++;
                                session_begin = 1;
                            }
                    }
                    break;
                case 1:
                    Send_Byte(CA);
                    Send_Byte(CA);
                    return -3;
                default:
                    if (session_begin > 0)
                    {
                        errors ++;
                    }
                    if (errors > MAX_ERRORS)
                    {
                        Send_Byte(CA);
                        Send_Byte(CA);
                        return 0;
                    }
                    Send_Byte(CRC16);
                    break;
            }
            if (file_done != 0)
            {
                break;
            }
        }
        if (session_done != 0)
        {
            break;
        }
    }
    return (int32_t)size;
}
#endif
/**
  * @brief  check response using the ymodem protocol
  * @param  buf: Address of the first byte
  * @retval The size of the file
  */
int32_t Ymodem_CheckResponse(uint8_t c)
{
  return 0;
}
#if 0
/**
  * @brief  Prepare the first block
  * @param  timeout
  *     0: end of transmission
  * @retval None
  */
void Ymodem_PrepareIntialPacket(uint8_t *data, const uint8_t* fileName, uint32_t *length)
{
  uint16_t i, j;
  uint8_t file_ptr[10];

  /* Make first three packet */
  data[0] = MODEM_SOH;
  data[1] = 0x00;
  data[2] = 0xff;

  /* Filename packet has valid data */
  for (i = 0; (fileName[i] != '\0') && (i < FILE_NAME_LENGTH);i++)
  {
     data[i + PACKET_HEADER] = fileName[i];
  }

  data[i + PACKET_HEADER] = 0x00;

  Int2Str (file_ptr, *length);
  for (j =0, i = i + PACKET_HEADER + 1; file_ptr[j] != '\0' ; )
  {
     data[i++] = file_ptr[j++];
  }

  for (j = i; j < PACKET_SIZE + PACKET_HEADER; j++)
  {
    data[j] = 0;
  }
}

/**
  * @brief  Prepare the data packet
  * @param  timeout
  *     0: end of transmission
  * @retval None
  */
void Ymodem_PreparePacket(uint8_t *SourceBuf, uint8_t *data, uint8_t pktNo, uint32_t sizeBlk)
{
  uint16_t i, size, packetSize;
  uint8_t* file_ptr;

  /* Make first three packet */
  packetSize = sizeBlk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
  size = sizeBlk < packetSize ? sizeBlk :packetSize;
  if (packetSize == PACKET_1K_SIZE)
  {
     data[0] = MODEM_STX;
  }
  else
  {
     data[0] = MODEM_SOH;
  }
  data[1] = pktNo;
  data[2] = (~pktNo);
  file_ptr = SourceBuf;

  /* Filename packet has valid data */
  for (i = PACKET_HEADER; i < size + PACKET_HEADER;i++)
  {
     data[i] = *file_ptr++;
  }
  if ( size  <= packetSize)
  {
    for (i = size + PACKET_HEADER; i < packetSize + PACKET_HEADER; i++)
    {
      data[i] = 0x1A; /* EOF (0x1A) or 0x00 */
    }
  }
}

/**
  * @brief  Transmit a data packet using the ymodem protocol
  * @param  data
  * @param  length
  * @retval None
  */
void Ymodem_SendPacket(uint8_t *data, uint16_t length)
{
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Send_Byte(data[i]);
    i++;
  }
}

/**
  * @brief  Transmit a file using the ymodem protocol
  * @param  buf: Address of the first byte
  * @retval The size of the file
  */
uint8_t Ymodem_Transmit (uint8_t *buf, const uint8_t* sendFileName, uint32_t sizeFile)
{

  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint8_t filename[FILE_NAME_LENGTH];
  uint8_t *buf_ptr, tempCheckSum;
  uint16_t tempCRC;
  uint16_t blkNumber;
  uint8_t receivedC[2], CRC16_F = 0, i;
  uint32_t errors, ackReceived, size = 0, pktSize;

  errors = 0;
  ackReceived = 0;
  for (i = 0; i < (FILE_NAME_LENGTH - 1); i++)
  {
    filename[i] = sendFileName[i];
  }
  CRC16_F = 1;

  /* Prepare first block */
  Ymodem_PrepareIntialPacket(&packet_data[0], filename, &sizeFile);

  do
  {
    /* Send Packet */
    Ymodem_SendPacket(packet_data, PACKET_SIZE + PACKET_HEADER);

    /* Send CRC or Check Sum based on CRC16_F */
    if (CRC16_F)
    {
       tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
       Send_Byte(tempCRC >> 8);
       Send_Byte(tempCRC & 0xFF);
    }
    else
    {
       tempCheckSum = CalChecksum (&packet_data[3], PACKET_SIZE);
       Send_Byte(tempCheckSum);
    }

    /* Wait for Ack and 'C' */
    if (Receive_Byte(&receivedC[0], 10000) == 0)
    {
      if (receivedC[0] == ACK)
      {
        /* Packet transferred correctly */
        ackReceived = 1;
      }
    }
    else
    {
        errors++;
    }
  }while (!ackReceived && (errors < 0x0A));

  if (errors >=  0x0A)
  {
    return errors;
  }
  buf_ptr = buf;
  size = sizeFile;
  blkNumber = 0x01;
  /* Here 1024 bytes package is used to send the packets */


  /* Resend packet if NAK  for a count of 10 else end of communication */
  while (size)
  {
    /* Prepare next packet */
    Ymodem_PreparePacket(buf_ptr, &packet_data[0], blkNumber, size);
    ackReceived = 0;
    receivedC[0]= 0;
    errors = 0;
    do
    {
      /* Send next packet */
      if (size >= PACKET_1K_SIZE)
      {
        pktSize = PACKET_1K_SIZE;

      }
      else
      {
        pktSize = PACKET_SIZE;
      }
      Ymodem_SendPacket(packet_data, pktSize + PACKET_HEADER);
      /* Send CRC or Check Sum based on CRC16_F */
      /* Send CRC or Check Sum based on CRC16_F */
      if (CRC16_F)
      {
         tempCRC = Cal_CRC16(&packet_data[3], pktSize);
         Send_Byte(tempCRC >> 8);
         Send_Byte(tempCRC & 0xFF);
      }
      else
      {
        tempCheckSum = CalChecksum (&packet_data[3], pktSize);
        Send_Byte(tempCheckSum);
      }

      /* Wait for Ack */
      if ((Receive_Byte(&receivedC[0], 100000) == 0)  && (receivedC[0] == ACK))
      {
        ackReceived = 1;
        if (size > pktSize)
        {
           buf_ptr += pktSize;
           size -= pktSize;
           if (blkNumber == (USER_FLASH_SIZE/1024))
           {
             return 0xFF; /*  error */
           }
           else
           {
              blkNumber++;
           }
        }
        else
        {
          buf_ptr += pktSize;
          size = 0;
        }
      }
      else
      {
        errors++;
      }
    }while(!ackReceived && (errors < 0x0A));
    /* Resend packet if NAK  for a count of 10 else end of communication */

    if (errors >=  0x0A)
    {
      return errors;
    }

  }
  ackReceived = 0;
  receivedC[0] = 0x00;
  errors = 0;
  do
  {
    Send_Byte(EOT);
    /* Send (EOT); */
    /* Wait for Ack */
    if ((Receive_Byte(&receivedC[0], 10000) == 0)  && receivedC[0] == ACK)
    {
      ackReceived = 1;
    }
    else
    {
      errors++;
    }
  }while (!ackReceived && (errors < 0x0A));

  if (errors >=  0x0A)
  {
    return errors;
  }

  /* Last packet preparation */
  ackReceived = 0;
  receivedC[0] = 0x00;
  errors = 0;

  packet_data[0] = SOH;
  packet_data[1] = 0;
  packet_data [2] = 0xFF;

  for (i = PACKET_HEADER; i < (PACKET_SIZE + PACKET_HEADER); i++)
  {
     packet_data [i] = 0x00;
  }

  do
  {
    /* Send Packet */
    Ymodem_SendPacket(packet_data, PACKET_SIZE + PACKET_HEADER);

    /* Send CRC or Check Sum based on CRC16_F */
    tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
    Send_Byte(tempCRC >> 8);
    Send_Byte(tempCRC & 0xFF);

    /* Wait for Ack and 'C' */
    if (Receive_Byte(&receivedC[0], 10000) == 0)
    {
      if (receivedC[0] == ACK)
      {
        /* Packet transferred correctly */
        ackReceived = 1;
      }
    }
    else
    {
        errors++;
    }
  }while (!ackReceived && (errors < 0x0A));

  /* Resend packet if NAK  for a count of 10  else end of communication */
  if (errors >=  0x0A)
  {
    return errors;
  }

  do
  {
    Send_Byte(EOT);
    /* Send (EOT); */
    /* Wait for Ack */
    if ((Receive_Byte(&receivedC[0], 10000) == 0)  && receivedC[0] == ACK)
    {
      ackReceived = 1;
    }
    else
    {
      errors++;
    }
  }while (!ackReceived && (errors < 0x0A));

  if (errors >=  0x0A)
  {
    return errors;
  }
  return 0; /* file transmitted successfully */
}
#endif
/**
  * @}
  */

/*******************(C)COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
