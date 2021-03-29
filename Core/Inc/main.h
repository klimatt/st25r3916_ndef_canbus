/**
  ******************************************************************************
 * @file    main.h
 * @author  MMY Application Team
 * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "nfc06a1.h"
#include "nucleo_l476rg_bus.h"
#include "stm32l4xx_hal_can.h"

/** @defgroup X-CUBE-NFC6_Applications
 *  @brief Sample applications for X-NUCLEO-NFC06A1 STM32 expansion boards.
 *  @{
 */

/** @defgroup PollingTagDetect
 *  @brief This demo shows how to poll for several types of NFC cards/devices and how 
 *  to exchange data with these devices, using the RFAL library.
 *  @{
 */

/** @defgroup PTD_Main
 *  @brief Main application program
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PTD_Main_Exported_Constants
 *  @{
 */
#define LED_FIELD_Pin 1         /*!< Enable usage of led field pin on the platform      */
#define LED_FIELD_GPIO_Port 1   /*!< Enable usage of led field port on the platform     */

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PTD_Main_Exported_Constants
 *  @{
 */
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup PTD_Main_Exported_Functions
 *  @{
 */
   
void _Error_Handler(char * file, int line);

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

#endif /* MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
