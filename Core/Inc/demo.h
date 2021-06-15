/**
 *****************************************************************************
 * @file    demo.h
 * @brief   Header for polling tag detect
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0094, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0094
 *
  ******************************************************************************
  */
/*! \file
 *
 *  \author 
 *
 *  \brief Demo functionality header file
 *
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEMO_H
#define DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "st_errno.h"
#include "rfal_nfc.h"
/** @addtogroup X-CUBE-NFC6_Applications
 *  @{
 */

/** @addtogroup PollingTagDetect
 *  @{
 */

/** @defgroup PTD_Demo
 *  @brief Demo functions containing the example code
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @defgroup PTD_Demo_Exported_Functions
 *  @{
 */
bool demoIni( void );
extern void demoCycle(void);
extern bool get_data(uint8_t * id, uint8_t * data, uint8_t * id_len, uint8_t * data_len, uint16_t * rssi);
/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* DEMO_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
