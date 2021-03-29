/**
  ******************************************************************************
  * @file    nucleo_l476rg_bus.c
  * @author  MMY Application Team
  * @brief   header file for the BSP BUS IO driver
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
#ifndef NUCLEO_L476RG_BUS_H
#define NUCLEO_L476RG_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nucleo_l476rg_conf.h"
#include "nucleo_l476rg_errno.h"

#if 0 /* Code not compiled, reserved for Doxygen */
/** @addtogroup BSP
  * @{
  */

/** @addtogroup NUCLEO_L476RG
  * @{
  */

/** @defgroup NUCLEO_L476RG_BUS NUCLEO_L476RG BUS
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

/* Exported types ------------------------------------------------------------*/
#if 0 /* Code not compiled, reserved for Doxygen */
/** @defgroup NUCLEO_L476RG_BUS_Private_Types NUCLEO_L476RG BUS Private types
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
typedef struct
{
  pSPI_CallbackTypeDef  pMspSpiInitCb;
  pSPI_CallbackTypeDef  pMspSpiDeInitCb;
}BSP_SPI_Cb_t;
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1) */

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
typedef struct
{
  pI2C_CallbackTypeDef  pMspI2cInitCb;
  pI2C_CallbackTypeDef  pMspI2cDeInitCb;
}BSP_I2C_Cb_t;
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1) */
/* Exported constants --------------------------------------------------------*/
#if 0 /* Code not compiled, reserved for Doxygen */
/** @defgroup NUCLEO_L476RG_BUS_Exported_Constants NUCLEO_L476RG BUS Exported Constants
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

#ifndef RFAL_USE_I2C
#define BUS_SPI1                        SPI1
#define BUS_SPI1_CLK_ENABLE()           __HAL_RCC_SPI1_CLK_ENABLE()
#define BUS_SPI1_CLK_DISABLE()          __HAL_RCC_SPI1_CLK_DISABLE()
#define BUS_SPI1_NSS_GPIO_PIN           GPIO_PIN_6
#define BUS_SPI1_IRQ_GPIO_PIN           GPIO_PIN_0
#define BUS_SPI1_SCK_GPIO_PIN           GPIO_PIN_5
#define BUS_SPI1_MISO_GPIO_PIN          GPIO_PIN_6
#define BUS_SPI1_MOSI_GPIO_PIN          GPIO_PIN_7
#define BUS_SPI1_NSS_GPIO_PORT          GPIOB
#define BUS_SPI1_IRQ_GPIO_PORT          GPIOA
#define BUS_SPI1_SCK_GPIO_PORT          GPIOA
#define BUS_SPI1_MISO_GPIO_PORT         GPIOA
#define BUS_SPI1_MOSI_GPIO_PORT         GPIOA
#define BUS_SPI1_NSS_MODE               GPIO_MODE_OUTPUT_PP
#define BUS_SPI1_IRQ_MODE               GPIO_MODE_IT_RISING
#define BUS_SPI1_SCK_MODE               GPIO_MODE_AF_PP
#define BUS_SPI1_MISO_MODE              GPIO_MODE_AF_PP
#define BUS_SPI1_MOSI_MODE              GPIO_MODE_AF_PP
#define BUS_SPI1_NSS_GPIO_AF            GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_AF            GPIO_AF5_SPI1
#define BUS_SPI1_MISO_GPIO_AF           GPIO_AF5_SPI1
#define BUS_SPI1_MOSI_GPIO_AF           GPIO_AF5_SPI1
#define BUS_SPI1_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI1_IRQ_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_SCK_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MISO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MOSI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define BSP_SPI1_IRQ_PRIO               ((uint32_t) 0x0F)
#define BSP_SPI1_IRQ_EXTI_IRQn          EXTI0_IRQn
#define BSP_SPI1_IRQ_EXTI_LINE          EXTI_LINE_0

#ifndef BUS_SPI1_BAUDRATE
#define BUS_SPI1_BAUDRATE       5000000U /* baud rate of SPI1 = 5 Mbps*/
#endif

#ifndef BUS_SPI1_TIMEOUT
#define BUS_SPI1_TIMEOUT        2000U /* baud rate of SPI1 = 5 Mbps*/
#endif
#else
#define BUS_I2C1                        I2C1
#define BUS_I2C1_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define BUS_I2C1_CLK_DISABLE()          __HAL_RCC_I2C1_CLK_DISABLE()
#define BUS_I2C1_IRQ_GPIO_PIN           GPIO_PIN_0
#define BUS_I2C1_SCL_GPIO_PIN           GPIO_PIN_8
#define BUS_I2C1_SDA_GPIO_PIN           GPIO_PIN_9
#define BUS_I2C1_IRQ_GPIO_PORT          GPIOA
#define BUS_I2C1_SCL_GPIO_PORT          GPIOB
#define BUS_I2C1_SDA_GPIO_PORT          GPIOB
#define BUS_I2C1_IRQ_MODE               GPIO_MODE_IT_RISING
#define BUS_I2C1_SCL_MODE               GPIO_MODE_AF_OD
#define BUS_I2C1_SDA_MODE               GPIO_MODE_AF_OD
#define BUS_I2C1_SCK_GPIO_AF            GPIO_AF4_I2C1
#define BUS_I2C1_SDA_GPIO_AF            GPIO_AF4_I2C1
#define BUS_I2C1_IRQ_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_I2C1_SCL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C1_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define BSP_I2C1_IRQ_PRIO               ((uint32_t) 0x0F)
#define BSP_I2C1_IRQ_EXTI_IRQn          EXTI0_IRQn
#define BSP_I2C1_IRQ_EXTI_LINE          EXTI_LINE_0

#ifndef BUS_I2C1_POLL_TIMEOUT
#define BUS_I2C1_POLL_TIMEOUT   0x1000U
#endif
/*  Frequeny in Hz  */
#ifndef BUS_FREQUENCY
#define BUS_FREQUENCY           4000000U /* Frequency of I2C1 = 400 KHz*/
#endif
#endif /* RFAL_USE_I2C */
#if 0 /* Code not compiled, reserved for Doxygen */
/**
  * @}
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
#if 0 /* Code not compiled, reserved for Doxygen */
/** @defgroup NUCLEO_L476RG_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

#ifndef RFAL_USE_I2C
extern SPI_HandleTypeDef Handle_Spi1;
#else
extern I2C_HandleTypeDef Handle_I2c1;
#endif /* RFAL_USE_I2C */

#if 0 /* Code not compiled, reserved for Doxygen */
/**
  * @}
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

/* Exported functions ------------------------------------------------------- */
#if 0 /* Code not compiled, reserved for Doxygen */
/** @addtogroup NUCLEO_L476RG_BUS_Exported_Functions
  * @{
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

int32_t BSP_GetTick(void);
#ifndef RFAL_USE_I2C
/* BUS IO driver over SPI Peripheral */
HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef * const p_SpiHandle, const uint32_t Baudrate_Presc);
int32_t BSP_SPI1_Init(void);
int32_t BSP_SPI1_DeInit(void);
int32_t BSP_SPI1_SendRecv(const uint8_t * const pTxData, uint8_t * const pRxData, uint16_t Length);
int32_t BSP_SPI1_Error(void);
void BSP_SPI1_IRQ_Callback(void);
void BSP_SPI1_IRQHandler(void);
#else
/* BUS IO driver over I2C Peripheral */
HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef * const hi2c, const uint32_t timing);
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(const uint16_t DevAddr, const uint32_t Trials);
int32_t BSP_I2C1_WriteReg(const uint16_t Addr, const uint16_t Reg, const uint8_t * const pData, const uint16_t Length);
int32_t BSP_I2C1_ReadReg(const uint16_t Addr, const uint16_t Reg, uint8_t * const pData, uint16_t Length);
int32_t BSP_I2C1_WriteReg16(const uint16_t Addr, const uint16_t Reg, const uint8_t * const pData, const uint16_t Length);
int32_t BSP_I2C1_ReadReg16(const uint16_t Addr, const uint16_t Reg, uint8_t * const pData, uint16_t Length);
int32_t BSP_I2C1_Send(const uint16_t DevAddr, const uint8_t * const pData, const uint16_t Length, const uint8_t txOnly);
int32_t BSP_I2C1_Recv(const uint16_t DevAddr, uint8_t * const pData, uint16_t Length);
int32_t BSP_I2C1_SequencialSend(const uint16_t DevAddr, const uint8_t * const pData, const uint16_t Length, const uint8_t last, const uint8_t txOnly);
int32_t BSP_I2C1_SequencialRecv(const uint16_t DevAddr, uint8_t * const pData, uint16_t Length);
void BSP_I2C1_IRQ_Callback(void);
void BSP_I2C1_IRQHandler(void);
void BSP_I2C1_EV_IRQHandler(void);
void BSP_I2C1_ER_IRQHandler(void);
#endif /* RFAL_USE_I2C */

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

/**
  * @}
  */
#endif /* MISRAC2012 4.4 : Avoid a section of code seems to be commented out */

#ifdef __cplusplus
}
#endif

#endif /* NUCLEO_L476RG_BUS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
