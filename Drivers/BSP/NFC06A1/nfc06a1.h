/**
  ******************************************************************************
 * @file    nfc06a1.h
 * @author  MMY Application Team
 * @brief   This file contains definitions for the nfc06a1.c
 *          board specific functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NFC06A1_H
#define NFC06A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if (defined (USE_STM32L4XX_NUCLEO))
#include "stm32l4xx_nucleo.h"
#elif (defined (USE_STM32F4XX_NUCLEO))
#include "stm32f4xx_nucleo.h"
#elif (defined (USE_STM32L0XX_NUCLEO))
#include "stm32l0xx_nucleo.h"
#endif
#include "nfc06a1_conf.h"

#if 0 /* Code not compiled, reserved for Doxygen */
/** @addtogroup BSP
  * @{
  */

/** @defgroup NFC06A1
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

/* Exported types ------------------------------------------------------------*/
#if 0 /* Code not compiled, reserved for Doxygen */
/** @defgroup NFC06A1_Exported_Types
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */
/**
 * @brief  NFC06A1 Led enumerator definition
 */
typedef enum 
{
  TF_LED = 0,         /*!< Led Type F   */
  TB_LED,             /*!< Led Type B   */
  TA_LED,             /*!< Led Type A   */
  TV_LED,             /*!< Led Type V   */
  AP2P_LED,           /*!< Led Type P2P */
  TX_LED              /*!< Led Field    */
}NFC06A1_Led_E;

/**
 * @brief  NFC06A1 Led structure definition
 */
typedef struct
{
  uint16_t          NFC06A1_LED_PIN;
  GPIO_TypeDef *    NFC06A1_LED_PIN_PORT;
}NFC06A1_Led_TypeDef;

#if 0 /* Code not compiled, reserved for Doxygen */
/**
  * @}
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#if 0 /* Code not compiled, reserved for Doxygen */
/** @defgroup NFC06A1_Exported_Functions
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

void NFC06A1_LED_Init(void);
void NFC06A1_LED_DeInit(const NFC06A1_Led_E led);
void NFC06A1_LED_ON(const NFC06A1_Led_E led);
void NFC06A1_LED_OFF(const NFC06A1_Led_E led);
void NFC06A1_LED_Toggle(const NFC06A1_Led_E led);

#if 0 /* Code not compiled, reserved for Doxygen */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

#ifdef __cplusplus
  }
#endif
#endif /* NFC06A1_H */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
