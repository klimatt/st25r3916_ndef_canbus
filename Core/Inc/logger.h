/**
 ******************************************************************************
 * @file    logger.h
 * @brief   Header for logger module
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
#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

#if (USE_LOGGER == LOGGER_OFF && !defined(HAL_UART_MODULE_ENABLED))
  #define UART_HandleTypeDef void
#endif
/** @addtogroup X-CUBE-NFC6_Applications
 *  @{
 */

/** @addtogroup PollingTagDetect
 *  @{
 */

/** @defgroup PTD_Logger
 *  @brief Driver providing printf-like functions to output log messages.
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PTD_Logger_Exported_Constants
 *  @{
 */
#define LOGGER_ON   1                    /*!< Allows activating logger    */
#define LOGGER_OFF  0                    /*!< Allows deactivating logger  */
/**
  * @}
  */
   
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup PTD_Logger_Exported_Functions
 *  @{
 */
/*!
 *****************************************************************************
 *  \brief  Writes out a formated string via UART interface
 *
 *  This function is used to write a formated string via the UART interface.
 *
 *****************************************************************************
 */
extern void logUsartInit(UART_HandleTypeDef *husart);

/*!
 *****************************************************************************
 *  \brief  Writes out a formated string via UART interface
 *
 *  This function is used to write a formated string via the UART interface.
 *
 *****************************************************************************
 */
extern int logUsart(const char* format, ...);

/*!
 *****************************************************************************
 *  \brief  Writes out a formated string via ITM interface
 *
 *  This function is used to write a formated string via the ITM interface.
 *
 *****************************************************************************
 */
extern int logITM(const char* format, ...);
/*!
 *****************************************************************************
 *  \brief  helper to convert hex data into formated string
 *
 *  \param[in] data : pointer to buffer to be dumped.
 *
 *  \param[in] dataLen : buffer length
 *
 *  \return hex formated string
 *
 *****************************************************************************
 */
extern char* hex2Str(unsigned char * data, size_t dataLen);


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

#endif /* LOGGER_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
