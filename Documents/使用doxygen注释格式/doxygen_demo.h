/**
  ******************************************************************************
  * @file    doxygen_demo.h
  * @author  mousie
  * @version V1.0.0
  * @date    2010.3.24
  * @brief   ʹ��doxygen��ʽ��ͷ�ļ�ע��ʾ��, ����Ϊ���ע�͵ı�׼ģ��
  *          ʹ��doxygen���Կ������ɹ淶�����˵���ĵ�.
  *          �������doxygen����Ϣ��������������.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DOXYGEN_DEMO_H
#define __DOXYGEN_DEMO_H

/* Include -------------------------------------------------------------------*/
#include "somefile.h"

/** @addtogroup Drivers
  * @{
  */
/** @addtogroup Doxygen_Demo
  * @{
  */



/**
  ******************************************************************************
  * @defgroup Doxygen_Demo_Configure
  * @brief    �û�����
  ******************************************************************************
  * @{
  */
#define __EZOS_TASK_NUM                 16                                      ///< �������������
/**
  * @}
  */



/** @defgroup Doxygen_Demo_Public_TypeDefine
  * @brief    �������Ͷ���
  * @{
  */
/// �������������״̬
typedef enum
{
  EZTASK_OK                             = (0x00),                               ///< ������ز����ɹ�
  EZTASK_ERROR                          = (0x01),                               ///< ������ز���ʧ��
  EZTASK_EXIST                          = (0x02),                               ///< �����Ѵ���
} ezTask_Status;
/**
  * @}
  */

/** @defgroup Doxygen_Demo_Public_MacroDefine
  * @brief    ���к궨��
  * @{
  */
#define __TASK_PROC                     void                                    ///< ������ʾ
/**
  * @}
  */

/** @defgroup Doxygen_Demo_Public_Variable
  * @brief    ��������ȫ�ֱ���
  * @{
  */

/**
  * @}
  */

/** @defgroup Doxygen_Demo_Public_Function
  * @brief    ���幫�к���
  * @{
  */
void demoInit(void);                                                            ///< ��ʼ������
/**
  * @}
  */



/**
  * @}
  */
/**
  * @}
  */

#endif
/* END OF FILE ---------------------------------------------------------------*/
