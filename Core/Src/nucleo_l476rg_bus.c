/**
  ******************************************************************************
  * @file    nucleo_l476rg_bus.c
  * @author  MMY Application Team
  * @brief   source file for the BSP BUS IO driver
  ******************************************************************************
  *@attention
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

/* Includes ------------------------------------------------------------------*/
#include "nucleo_l476rg_bus.h"
#include "st25R3916_irq.h"
#include <string.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup NUCLEO_L476RG
  * @{
  */

/** @defgroup NUCLEO_L476RG_BUS NUCLEO_L476RG BUS
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define I2C_ANALOG_FILTER_ENABLE        1
#define I2C_ANALOG_FILTER_DELAY_MIN     50 /* ns */
#define I2C_ANALOG_FILTER_DELAY_MAX     260 /* ns */
#define I2C_ANALOG_FILTER_DELAY_DEFAULT 2 /* ns */

#define VALID_PRESC_NBR                 100
#define PRESC_MAX                       16
#define SCLDEL_MAX                      16
#define SDADEL_MAX                      16
#define SCLH_MAX                        256
#define SCLL_MAX                        256
#define I2C_DNF_MAX                     16

#define NSEC_PER_SEC                    1000000000L

/* Private macros ------------------------------------------------------------*/
#define DIV_ROUND_CLOSEST(x, d)  (((x) + ((d) / 2)) / (d))

/* Private variables ---------------------------------------------------------*/
/** @defgroup NUCLEO_L476RG_Private_Variables BUS Private Variables
  * @{
  */
/**
 * @struct i2c_specs
 * @brief private i2c specification timing
 */
struct i2c_specs {
  uint32_t rate;        /*!< I2C bus speed (Hz). */
  uint32_t rate_min;    /*!< 80% of I2C bus speed (Hz). */
  uint32_t rate_max;    /*!< 100% of I2C bus speed (Hz). */
  uint32_t fall_max;    /*!< Max fall time of both SDA and SCL signals (ns). */
  uint32_t rise_max;    /*!< Max rise time of both SDA and SCL signals (ns). */
  uint32_t hddat_min;   /*!< Min data hold time (ns). */
  uint32_t vddat_max;   /*!< Max data valid time (ns). */
  uint32_t sudat_min;   /*!< Min data setup time (ns). */
  uint32_t l_min;       /*!< Min low period of the SCL clock (ns). */
  uint32_t h_min;       /*!< Min high period of the SCL clock (ns). */
};

/**
 * @enum i2c_speed
 * @brief private i2c speed choice
 */
enum i2c_speed {
  I2C_SPEED_STANDARD,   /*!< Standard mode 100 kHz */
  I2C_SPEED_FAST,       /*!< Fast mode 400 kHz */
  I2C_SPEED_FAST_PLUS,  /*!< Fast mode plus 1 MHz */
};

/**
 * @struct i2c_setup
 * @brief private I2C timing setup parameters
 */
struct i2c_setup {
  uint32_t rise_time;     /*!< Rise time (ns). */
  uint32_t fall_time;     /*!< Fall time (ns). */
  uint8_t dnf;            /*!< filter coefficient (0-16). */
  uint8_t analog_filter;  /*!< Analog filter delay (On/Off). */
};

/**
 * @struct i2c_timings
 * @brief private I2C output parameters
 */
struct i2c_timings {
  uint8_t presc;    /*!< Prescaler value. */
  uint8_t scldel;   /*!< Data setup time. */
  uint8_t sdadel;   /*!< Data hold time. */
  uint8_t sclh;     /*!< SCL high period (master mode). */
  uint8_t scll;     /*!< SCL low period (master mode). */
};

const struct i2c_specs i2c_specs[] = {
  [I2C_SPEED_STANDARD] = {
    .rate = 100000,
    .rate_min = 100000,
    .rate_max = 120000,
    .fall_max = 300,
    .rise_max = 1000,
    .hddat_min = 0,
    .vddat_max = 3450,
    .sudat_min = 250,
    .l_min = 4700,
    .h_min = 4000,
  },
  [I2C_SPEED_FAST] = {
    .rate = 400000,
    .rate_min = 320000,
    .rate_max = 400000,
    .fall_max = 300,
    .rise_max = 300,
    .hddat_min = 0,
    .vddat_max = 900,
    .sudat_min = 100,
    .l_min = 1300,
    .h_min = 600,
  },
  [I2C_SPEED_FAST_PLUS] = {
    .rate = 1000000,
    .rate_min = 800000,
    .rate_max = 1000000,
    .fall_max = 120,
    .rise_max = 120,
    .hddat_min = 0,
    .vddat_max = 450,
    .sudat_min = 50,
    .l_min = 500,
    .h_min = 260,
  },
};

const struct i2c_setup i2c_user_setup[] = {
  [I2C_SPEED_STANDARD] = {
  .rise_time = 400,
  .fall_time = 100,
  .dnf = I2C_ANALOG_FILTER_DELAY_DEFAULT,
  .analog_filter = 1,
 },
 [I2C_SPEED_FAST] = {
  .rise_time = 250,
  .fall_time = 100,
  .dnf = I2C_ANALOG_FILTER_DELAY_DEFAULT,
  .analog_filter = 1,
 },
 [I2C_SPEED_FAST_PLUS] = {
  .rise_time = 60,
  .fall_time = 100,
  .dnf = I2C_ANALOG_FILTER_DELAY_DEFAULT,
  .analog_filter = 1,
 }
};

#ifndef RFAL_USE_I2C
SPI_HandleTypeDef Handle_Spi1;
EXTI_HandleTypeDef hExti0 = {.Line=BSP_SPI1_IRQ_EXTI_LINE};
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
static uint32_t IsSPI1MspCbValid = 0;										
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#else
I2C_HandleTypeDef Handle_I2c1;
EXTI_HandleTypeDef hExti0 = {.Line=BSP_I2C1_IRQ_EXTI_LINE};
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
static uint32_t IsI2C1MspCbValid = 0;										
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
#endif /* RFAL_USE_I2C */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup NUCLEO_L476RG_Private_FunctionPrototypes  Private Function Prototypes
  * @{
  */
#ifndef RFAL_USE_I2C
static uint32_t SPI_GetPrescaler(const uint32_t clock_src_hz, const uint32_t baudrate_mbps );
static void SPI1_MspInit(SPI_HandleTypeDef * const spiHandle);
static void SPI1_MspDeInit(SPI_HandleTypeDef * const spiHandle);
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
int32_t BSP_SPI1_RegisterMspCallbacks (BSP_SPI_Cb_t * const Callback);
int32_t BSP_SPI1_RegisterDefaultMspCallbacks(void);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#else
static uint32_t I2C_GetTiming(const uint32_t clock_src_hz, const uint32_t i2cfreq_hz);
static void I2C1_MspInit(I2C_HandleTypeDef * const hI2c);
static void I2C1_MspDeInit(I2C_HandleTypeDef * const hI2c);
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t * const Callback);
int32_t BSP_I2C1_RegisterDefaultMspCallbacks(void);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
#endif /* RFAL_USE_I2C */

/**
  * @}
  */

/* Functions Definition ------------------------------------------------------*/
/** @defgroup NUCLEO_L476RG_BUS_Exported_Functions NUCLEO_L476RG_BUS Exported Functions
  * @{
  */
/**
  * @brief  Return system tick in ms
  * @param  None
  * @return Current HAL time base time stamp
  */
int32_t BSP_GetTick(void)
{
  return HAL_GetTick();
}

#ifndef RFAL_USE_I2C
/* BUS IO driver over SPI Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER SPI
*******************************************************************************/
/**
  * @brief  Initialize the SPI1 bus
  * @param  None
  * @return BSP status
  */
int32_t BSP_SPI1_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  Handle_Spi1.Instance  = BUS_SPI1;

  if (HAL_SPI_GetState(&Handle_Spi1) == HAL_SPI_STATE_RESET)
  {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
    /* Init the SPI Msp */
    SPI1_MspInit(&Handle_Spi1);
#else
    if(IsSPI1MspCbValid == 0U)
    {
      if(BSP_SPI1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    /* Init the SPI */
    if ((MX_SPI1_Init(&Handle_Spi1, SPI_GetPrescaler(HAL_RCC_GetPCLK2Freq(), BUS_SPI1_BAUDRATE) )) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

/**
  * @brief  DeInitialize SPI bus
  * @param  None
  * @retval BSP status
  */
int32_t BSP_SPI1_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  /* DeInit the SPI */
  if (HAL_SPI_DeInit(&Handle_Spi1) != HAL_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }
  
  /* DeInit the SPI Msp */
  SPI1_MspDeInit(&Handle_Spi1);

  return ret;
}

/**
  * @brief      SPI Write byte(s) to device
  * @param[in]  pTxData : Pointer to data buffer to write
  * @param[in]  Length : number of bytes to write
  * @return     BSP status
  */
int32_t BSP_SPI1_Send(const uint8_t * const pData, const uint16_t Length)
{
  HAL_StatusTypeDef status;
  uint8_t   tx[256];
  int32_t ret = BSP_ERROR_NONE;
  
  if (pData != NULL)
  {
    (void)memcpy(tx, pData, Length);
  }
  
  status = HAL_SPI_Transmit(&Handle_Spi1, tx, Length, BUS_SPI1_TIMEOUT);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    ret = BSP_SPI1_Init();
  }
  
  return ret;
}

/**
  * @brief      SPI Read and Write byte(s) to device
  * @param[in]  pTxData : Pointer to data buffer to write
  * @param[out] pRxData : Pointer to data buffer for read data
  * @param[in]  Length : number of bytes to write
  * @return     BSP status
  */
int32_t BSP_SPI1_Recv(uint8_t * const pData, const uint16_t Length)
{
  HAL_StatusTypeDef status;
  uint8_t   rx[256];
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  
  status = HAL_SPI_Receive(&Handle_Spi1, (pData != NULL) ? (uint8_t *)pData : rx, Length, BUS_SPI1_TIMEOUT);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    ret = BSP_SPI1_Init();
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

/**
  * @brief      SPI Read and Write byte(s) to device
  * @param[in]  pTxData : Pointer to data buffer to write
  * @param[out] pRxData : Pointer to data buffer for read data
  * @param[in]  Length : number of bytes to write
  * @return     BSP status
  */
int32_t BSP_SPI1_SendRecv(const uint8_t * const pTxData, uint8_t * const pRxData, uint16_t Length)
{
  HAL_StatusTypeDef status;
  uint8_t   tx[256];
  uint8_t   rx[256];
  int32_t ret = BSP_ERROR_NONE;
  
  if (pTxData != NULL)
  {
    (void)memcpy(tx, pTxData, Length);
  }
  
  status = HAL_SPI_TransmitReceive(&Handle_Spi1, tx, (pRxData != NULL) ? (uint8_t *)pRxData : rx, Length, BUS_SPI1_TIMEOUT);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    ret = BSP_SPI1_Init();
  }
  
  return ret;
}

/**
  * @brief  SPI error treatment function
  * @param  None
  * @return BSP status
  */
int32_t BSP_SPI1_Error(void)
{
  /* De-initialize the SPI communication BUS */
  (void)HAL_SPI_DeInit(&Handle_Spi1);

  /* Re-Initiaize the SPI communication BUS */
  return BSP_SPI1_Init();
}

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)  
/**
  * @brief     BSP SPI1 Bus Msp Callback registering
  * @param[in] Callback: pointer to SPI1 MspInit/MspDeInit callback functions
  * @return    BSP status
  */
int32_t BSP_SPI1_RegisterMspCallbacks (BSP_SPI_Cb_t * const Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  __HAL_SPI_RESET_HANDLE_STATE(&Handle_Spi1);

  /* Register MspInit/MspDeInit Callbacks */
  if (HAL_SPI_RegisterCallback(&Handle_Spi1, HAL_SPI_MSPINIT_CB_ID, Callback->pMspSpiInitCb) != HAL_OK)
  { 
    ret = BSP_ERROR_PERIPH_FAILURE;
  } 
  else if (HAL_SPI_RegisterCallback(&Handle_Spi1, HAL_SPI_MSPDEINIT_CB_ID, Callback->pMspSpiDeInitCb) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    IsSPI1MspCbValid = 1U;
  }

  /* BSP status */
  return ret;
}

/**
  * @brief  Register Default BSP SPI1 Bus Msp Callbacks
  * @param  None
  * @return BSP status
  */
int32_t BSP_SPI1_RegisterDefaultMspCallbacks(void)
{
  int32_t ret = BSP_ERROR_NONE;
  
  __HAL_SPI_RESET_HANDLE_STATE (&Handle_Spi1);

  /* Register default MspInit/MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&Handle_Spi1, HAL_SPI_MSPINIT_CB_ID, SPI1_MspInit) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else if (HAL_SPI_RegisterCallback(&Handle_Spi1, HAL_SPI_MSPDEINIT_CB_ID, SPI1_MspDeInit) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    IsSPI1MspCbValid = 1U;
  }

  /* BSP status */
  return ret;
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @brief         Configure the SPI1 peripheral
  * @param[in,out] p_SpiHandle: Handle to the SPI peripheral
  * @param[in]     Baudrate_Presc: Prescaler value, to achieve the baudrate, with respect to sysclock
  * @return        HAL status
  */
__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef* const p_SpiHandle, const uint32_t Baudrate_Presc)
{
  HAL_StatusTypeDef ret = HAL_OK;
  
  p_SpiHandle->Init.Mode              = SPI_MODE_MASTER;
  p_SpiHandle->Init.Direction         = SPI_DIRECTION_2LINES;
  p_SpiHandle->Init.DataSize          = SPI_DATASIZE_8BIT;
  p_SpiHandle->Init.CLKPolarity       = SPI_POLARITY_LOW;
  p_SpiHandle->Init.CLKPhase          = SPI_PHASE_2EDGE;
  p_SpiHandle->Init.NSS               = SPI_NSS_SOFT;
  p_SpiHandle->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  p_SpiHandle->Init.BaudRatePrescaler = Baudrate_Presc;
  p_SpiHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
  p_SpiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
  p_SpiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  p_SpiHandle->Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  p_SpiHandle->Init.CRCPolynomial     = 7;

  if (HAL_SPI_Init(p_SpiHandle) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

/**
  * @brief     Retrieve prescaler value to achieve baudrate with respect to sysclock
  * @param[in] Clock_Src_Hz: spi source clock
  * @param[in] Baudrate_Mbps: baudrate value to achieve in mbps
  * @return    Prescaler value
  */
static uint32_t SPI_GetPrescaler(const uint32_t Clock_Src_Hz, const uint32_t Baudrate_Mbps )
{
  uint32_t divisor = 0;
  uint32_t spi_clk = Clock_Src_Hz;
  uint32_t presc = 0;
  static const uint32_t baudrate[]= { 
                                      SPI_BAUDRATEPRESCALER_2,
                                      SPI_BAUDRATEPRESCALER_4,
                                      SPI_BAUDRATEPRESCALER_8,
                                      SPI_BAUDRATEPRESCALER_16,
                                      SPI_BAUDRATEPRESCALER_32,
                                      SPI_BAUDRATEPRESCALER_64,
                                      SPI_BAUDRATEPRESCALER_128,
                                      SPI_BAUDRATEPRESCALER_256,
                                    };

  while (spi_clk > Baudrate_Mbps)
  {
    presc = baudrate[divisor];
    if (++divisor > 7)
    {
      break;
    }
    
    spi_clk = (spi_clk >> 1);
  }

  return presc;
}

/**
  * @brief         Initialize SPI1 MSP.
  * @param[in,out] p_SpiHandle: Handle to the SPI1 peripheral
  * @return        None
  */
static void SPI1_MspInit(SPI_HandleTypeDef* const p_SpiHandle)
{
  GPIO_InitTypeDef  gpioinitstruct;
  
  /* Enable GPIO clock */
  BUS_SPI1_NSS_GPIO_CLK_ENABLE();
  BUS_SPI1_IRQ_GPIO_CLK_ENABLE();
  BUS_SPI1_SCK_GPIO_CLK_ENABLE();
  BUS_SPI1_MISO_GPIO_CLK_ENABLE();
  BUS_SPI1_MOSI_GPIO_CLK_ENABLE();
  
  /*** Configure GPIOs ***/
  /* Configure SPI NSS */
  /* Configure SPI NSS as GPIO function */
  gpioinitstruct.Pin        = BUS_SPI1_NSS_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_SPI1_NSS_MODE;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_MEDIUM;
  gpioinitstruct.Alternate  = BUS_SPI1_NSS_GPIO_AF;
  HAL_GPIO_Init(BUS_SPI1_NSS_GPIO_PORT, &gpioinitstruct);
  
  /* Configure SPI NSS pin Output Level */
  HAL_GPIO_WritePin(BUS_SPI1_NSS_GPIO_PORT, BUS_SPI1_NSS_GPIO_PIN, GPIO_PIN_SET);

  /* Configure SPI IRQ as GPIO interrupt function */
  gpioinitstruct.Pin        = BUS_SPI1_IRQ_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_SPI1_IRQ_MODE;
  HAL_GPIO_Init(BUS_SPI1_IRQ_GPIO_PORT, &gpioinitstruct);
  
  /* Configure SPI SCK, MISO and MOSI */
  /* Configure SPI SCK as alternate function */
  gpioinitstruct.Pin        = BUS_SPI1_SCK_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_SPI1_SCK_MODE;
  gpioinitstruct.Alternate  = BUS_SPI1_SCK_GPIO_AF;
  HAL_GPIO_Init(BUS_SPI1_SCK_GPIO_PORT, &gpioinitstruct);
  
  /* Configure SPI MISO as alternate function */
  gpioinitstruct.Pin        = BUS_SPI1_MISO_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_SPI1_MISO_MODE;
  gpioinitstruct.Alternate  = BUS_SPI1_MISO_GPIO_AF;
  HAL_GPIO_Init(BUS_SPI1_MISO_GPIO_PORT, &gpioinitstruct);
  
  /* Configure SPI MOSI as alternate function */
  gpioinitstruct.Pin        = BUS_SPI1_MOSI_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_SPI1_MOSI_MODE;
  gpioinitstruct.Alternate  = BUS_SPI1_MOSI_GPIO_AF;
  HAL_GPIO_Init(BUS_SPI1_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/
  /* Enable SPI clock */
  BUS_SPI1_CLK_ENABLE();
  
  /* Configure interrupt callback */
  (void)HAL_EXTI_GetHandle(&hExti0, hExti0.Line);  
  (void)HAL_EXTI_RegisterCallback(&hExti0, HAL_EXTI_COMMON_CB_ID, BSP_SPI1_IRQ_Callback);
    
  /* Enable interruption */
  HAL_NVIC_SetPriority(BSP_SPI1_IRQ_EXTI_IRQn, BSP_SPI1_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(BSP_SPI1_IRQ_EXTI_IRQn);
  
}

/**
  * @brief         DeInitialize SPI1 MSP.
  * @param[in,out] spiHandle: Handle to the SPI1 peripheral
  * @return        None
  */
static void SPI1_MspDeInit(SPI_HandleTypeDef* const spiHandle)
{
  /* Disable IRQ interrupts */
  HAL_NVIC_DisableIRQ((IRQn_Type)(BSP_SPI1_IRQ_EXTI_IRQn));
  
  /* Restore Gpio configuration */
  HAL_GPIO_DeInit(BUS_SPI1_NSS_GPIO_PORT, BUS_SPI1_NSS_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_IRQ_GPIO_PORT, BUS_SPI1_IRQ_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_SCK_GPIO_PORT, BUS_SPI1_SCK_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_MISO_GPIO_PORT, BUS_SPI1_MISO_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_MOSI_GPIO_PORT, BUS_SPI1_MOSI_GPIO_PIN);
  
  /* Peripheral clock disable */
  __HAL_RCC_SPI1_CLK_DISABLE();
}

/**
  * @brief  BSP SPI1 callback
  * @param  None
  * @return None
  */
__weak void BSP_SPI1_IRQ_Callback(void)
{
  /* Prevent unused argument(s) compilation warning */
  
  /* This function should be implemented by the user application.
   * It is called into this driver when an event from ST25R3916 is triggered. 
   */
  st25r3916Isr();
}

/**
  * @brief  BSP SPI1 interrupt Handler
  * @param  None
  * @return None
  */
void BSP_SPI1_IRQHandler(void)
{
  HAL_EXTI_IRQHandler(&hExti0);
}

#else
/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize the I2C1 bus
  * @param  None
  * @return BSP status
  */
int32_t BSP_I2C1_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  Handle_I2c1.Instance  = BUS_I2C1;

  if (HAL_I2C_GetState(&Handle_I2c1) == HAL_I2C_STATE_RESET)
  {
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    /* Init the SPI Msp */
    I2C1_MspInit(&Handle_I2c1);
#else
    if(IsI2C1MspCbValid == 0U)
    {
      if(BSP_I2C1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

    /* Init the I2C */
    if (MX_I2C1_Init(&Handle_I2c1, I2C_GetTiming(SystemCoreClock , BUS_I2C1_FREQUENCY)) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
    else if (HAL_I2CEx_ConfigAnalogFilter(&Handle_I2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

/**
  * @brief  DeInitialize the I2C1 bus
  * @param  None
  * @retval BSP status
  */
int32_t BSP_I2C1_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    /* DeInit the I2C */
    I2C1_MspDeInit(&Handle_I2c1);
  #endif
  /* DeInit the I2C */
  if (HAL_I2C_DeInit(&Handle_I2c1) != HAL_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }

  return ret;
}

/**
  * @brief     Return the status of the I2C1 Bus
  * @param[in] DevAddr: Device address on Bus
  * @param[in] Trials: Number of tentatives
  * @return    BSP status
  */
int32_t BSP_I2C1_IsReady(const uint16_t DevAddr, const uint32_t Trials)
{
  int32_t ret;
  if (HAL_I2C_IsDeviceReady(&Handle_I2c1, DevAddr, Trials, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }
  return ret;
}

/**
  * @brief     Write a value in a register of the device through bus.
  * @param[in] DevAddr: Device address on Bus
  * @param[in] Reg: The target register address to write
  * @param[in] pData: Pointer to data buffer to write
  * @param[in] Length: Data Length
  * @return    BSP status
  */

int32_t BSP_I2C1_WriteReg(const uint16_t DevAddr, const uint16_t Reg, const uint8_t * const pData, const uint16_t Length)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  uint32_t hal_error = HAL_OK;

  if (HAL_I2C_Mem_Write(&Handle_I2c1, (uint8_t)DevAddr,
                       (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)pData, Length, BUS_I2C1_POLL_TIMEOUT) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    hal_error = HAL_I2C_GetError(&Handle_I2c1);
    if (hal_error == HAL_I2C_ERROR_AF)
    {
      return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief      Reads a register of the device through bus
  * @param[in]  DevAddr: Device address on Bus
  * @param[in]  Reg: The target register address to read
  * @param[out] pData: Pointer to data buffer to read
  * @param[in]  Length: Data Length
  * @return     BSP status
  */
int32_t  BSP_I2C1_ReadReg(const uint16_t DevAddr, const uint16_t Reg, uint8_t * const pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  uint32_t hal_error = HAL_OK;

  if (HAL_I2C_Mem_Read(&Handle_I2c1, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_8BIT, pData,
                       Length, 0x1000) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    hal_error = HAL_I2C_GetError(&Handle_I2c1);
    if (hal_error == HAL_I2C_ERROR_AF)
    {
      return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**

  * @brief     Writes a value in a register of the device through bus
  * @param[in] DevAddr: Device address on Bus
  * @param[in] Reg: The target register address to write
  * @param[in] pData: Pointer to data buffer to write
  * @param[in] Length: Data Length
  * @return    BSP statu
  */
int32_t BSP_I2C1_WriteReg16(const uint16_t DevAddr, const uint16_t Reg, const uint8_t * const pData, const uint16_t Length)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  uint32_t hal_error = HAL_OK;

  if (HAL_I2C_Mem_Write(&Handle_I2c1, (uint8_t)DevAddr,
                       (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                       (uint8_t *)pData, Length, 0x1000) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    hal_error = HAL_I2C_GetError(&Handle_I2c1);
    if (hal_error == HAL_I2C_ERROR_AF)
    {
      return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief      Read registers through a bus (16 bits)
  * @param[in]  DevAddr: Device address on BUS
  * @param[in]  Reg: The target register address to read
  * @param[out] pData: Pointer to data buffer to read
  * @param[in]  Length: Data Length
  * @return     BSP status
  */
int32_t  BSP_I2C1_ReadReg16(const uint16_t DevAddr, const uint16_t Reg, uint8_t * const pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  uint32_t hal_error = HAL_OK;

  if (HAL_I2C_Mem_Read(&Handle_I2c1, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_16BIT, pData,
                       Length, 0x1000) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    hal_error = HAL_I2C_GetError(&Handle_I2c1);
    if (hal_error == HAL_I2C_ERROR_AF)
    {
      return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief     This method transmits sequentially in master I2C mode an amount of data in blocking mode with Interrupt.
  * @param[in] address : Target device address: The device 7 bits address value
  *                      in datasheet must be shifted to the left before calling the interface
  * @param[in] txBuf   : Pointer to data buffer
  * @param[in] txLen   : Amount of data to be sent
  * @param[in] last    : true if last data to be transmitted
  * @param[in] txOnly  : true if no reception is to be performed after (STOP)
  *                      false if a reception should happen afterwards with repeated START
  * @return    HAL Error code
  */
int32_t BSP_I2C1_Send(const uint16_t DevAddr, const uint8_t * const pData, const uint16_t Length, const uint8_t txOnly)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if (HAL_I2C_Master_Transmit(&Handle_I2c1, DevAddr, (uint8_t *)pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK )
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }
  
  return ret;
}

/**
  * @brief      This method receives sequentially in master I2C mode an amount of data in blocking mode with Interrupt.
  * @param[in]  address : Target device address: The device 7 bits address value
  *                       in datasheet must be shifted to the left before calling the interface
  * @param[out] rxBuf   : Pointer to data buffer
  * @param[in]  rxLen   : Amount of data to read
  * @return     HAL Error code
  */
int32_t BSP_I2C1_Recv(const uint16_t DevAddr, uint8_t * const pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if (HAL_I2C_Master_Receive(&Handle_I2c1, DevAddr, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK )
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }
  
  return ret;
}

/**
  * @brief     This method transmits sequentially in master I2C mode an amount of data in blocking mode with Interrupt.
  * @param[in] address : Target device address: The device 7 bits address value
  *                      in datasheet must be shifted to the left before calling the interface
  * @param[in] txBuf   : Pointer to data buffer
  * @param[in] txLen   : Amount of data to be sent
  * @param[in] last    : true if last data to be transmitted
  * @param[in] txOnly  : true if no reception is to be performed after (STOP)
  *                      false if a reception should happen afterwards with repeated START
  * @return    HAL Error code
  */
int32_t BSP_I2C1_SequencialSend(const uint16_t DevAddr, const uint8_t * const pData, const uint16_t Length, const uint8_t last, const uint8_t txOnly)
{
  HAL_StatusTypeDef ret;
  uint32_t txFlag;

  if ((last != 0) && (txOnly != 0))
  {
    txFlag = I2C_LAST_FRAME;
  }
  else
  {
    txFlag = (last ? /*I2C_LAST_FRAME_NO_STOP*/ I2C_FIRST_FRAME : I2C_FIRST_AND_NEXT_FRAME);
  }
  
  
  ret = HAL_I2C_Master_Sequential_Transmit_IT(&Handle_I2c1, DevAddr, (uint8_t*)pData, Length, txFlag);
  if (ret != HAL_OK)
  {
    return ret;
  }
  
  while (HAL_I2C_GetState(&Handle_I2c1) != HAL_I2C_STATE_READY);
  
  if (HAL_I2C_GetError(&Handle_I2c1) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * @brief      This method receives sequentially in master I2C mode an amount of data in blocking mode with Interrupt.
  * @param[in]  address : Target device address: The device 7 bits address value
  *                       in datasheet must be shifted to the left before calling the interface
  * @param[out] rxBuf   : Pointer to data buffer
  * @param[in]  rxLen   : Amount of data to read
  * @return     HAL Error code
  */
int32_t BSP_I2C1_SequencialRecv(const uint16_t DevAddr, uint8_t * const pData, uint16_t Length)
{
  HAL_StatusTypeDef ret;
  
  ret = HAL_I2C_Master_Sequential_Receive_IT(&Handle_I2c1, DevAddr, (uint8_t*)pData, Length, I2C_LAST_FRAME);
  if (ret != HAL_OK)
  {
    return ret;
  }
  
  while (HAL_I2C_GetState(&Handle_I2c1) != HAL_I2C_STATE_READY);
  
  if (HAL_I2C_GetError(&Handle_I2c1) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief         Configure the I2C1 bus
  * @param[in,out] p_I2cHandle: Handle to the SPI peripheral
  * @param[in]     Timing: Timing value, to achieve the baudrate, with respect to sysclock
  * @return        HAL status
  */
__weak HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef * const p_I2cHandle, const uint32_t Timing)
{
  HAL_StatusTypeDef ret = HAL_OK;
  
  p_I2cHandle->Instance = BUS_I2C1;
  p_I2cHandle->Init.Timing = Timing;
  p_I2cHandle->Init.OwnAddress1 = 0;
  p_I2cHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  p_I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  p_I2cHandle->Init.OwnAddress2 = 0;
  p_I2cHandle->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  p_I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  p_I2cHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(p_I2cHandle) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(p_I2cHandle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(p_I2cHandle, 2) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 5);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

  return ret;
}

/**
  * @brief     Retrieve timing value to achieve baudrate with respect to sysclock
  * @param[in] Clock_Src_Hz: I2C source clock
  * @param[in] I2CFreq_hz: frequency value to achieve in hz
  * @return    STM32L476 I2C timing value
  */
static uint32_t I2C_GetTiming(const uint32_t Clock_Src_Hz, const uint32_t I2CFreq_hz)
{
  uint32_t ret = 0;
  uint32_t speed = 0;
  uint32_t is_valid_speed = 0;
  uint32_t p_prev = PRESC_MAX;
  uint32_t i2cclk;
  uint32_t i2cspeed;
  uint32_t clk_error_prev;
  uint32_t tsync;
  uint32_t af_delay_min, af_delay_max;
  uint32_t dnf_delay;
  uint32_t clk_min, clk_max;
  int32_t sdadel_min, sdadel_max;
  int32_t scldel_min;
  struct i2c_timings *s;
  struct i2c_timings valid_timing[VALID_PRESC_NBR];
  uint32_t p, l, a, h;
  uint32_t valid_timing_nbr = 0;
 
  if ((Clock_Src_Hz == 0U) || (I2CFreq_hz == 0U))
  {
    ret = 0;
  }
  else
  {
    for (uint32_t i =0 ; i <= (uint32_t)I2C_SPEED_FAST_PLUS ; i++)
    {
      if ((I2CFreq_hz >= i2c_specs[i].rate_min) && (I2CFreq_hz <= i2c_specs[i].rate_max))
      {
        is_valid_speed = 1;
        speed = i;
        break;
      }
    }

    if (is_valid_speed != 0U)
    {
      i2cclk = DIV_ROUND_CLOSEST(NSEC_PER_SEC, Clock_Src_Hz);
      i2cspeed = DIV_ROUND_CLOSEST(NSEC_PER_SEC, I2CFreq_hz);
      clk_error_prev = i2cspeed;
      /* Analog and Digital Filters */
      af_delay_min =((i2c_user_setup[speed].analog_filter == 1U)? I2C_ANALOG_FILTER_DELAY_MIN : 0U);
      af_delay_max =((i2c_user_setup[speed].analog_filter == 1U)? I2C_ANALOG_FILTER_DELAY_MAX : 0U);
      dnf_delay = i2c_user_setup[speed].dnf * i2cclk;
      sdadel_min = (int32_t)i2c_user_setup[speed].fall_time - (int32_t)i2c_specs[speed].hddat_min 
                    - (int32_t)af_delay_min - (int32_t)(((int32_t)i2c_user_setup[speed].dnf + 3) * (int32_t)i2cclk);
      sdadel_max = (int32_t)i2c_specs[speed].vddat_max - (int32_t)i2c_user_setup[speed].rise_time 
                    - (int32_t)af_delay_max - (int32_t)(((int32_t)i2c_user_setup[speed].dnf + 4) * (int32_t)i2cclk);
      scldel_min = (int32_t)i2c_user_setup[speed].rise_time + (int32_t)i2c_specs[speed].sudat_min;

      if (sdadel_min <= 0)
      {
        sdadel_min = 0;
      }
      
      if (sdadel_max <= 0)
      { 
        sdadel_max = 0;
      }

      /* Compute possible values for PRESC, SCLDEL and SDADEL */
      for (p = 0; p < PRESC_MAX; p++)
      { 
        for (l = 0; l < SCLDEL_MAX; l++)
        { 
          uint32_t scldel = (l + 1U) * (p + 1U) * i2cclk;
          if (scldel < (uint32_t)scldel_min)
          { 
            continue;
          }

          for (a = 0; a < SDADEL_MAX; a++)
          {
            uint32_t sdadel = ((a * (p + 1U)) + 1U) * i2cclk;

            if (((sdadel >= (uint32_t)sdadel_min) && (sdadel <= (uint32_t)sdadel_max))&& (p != p_prev))
            {
              valid_timing[valid_timing_nbr].presc = p;
              valid_timing[valid_timing_nbr].scldel = l;
              valid_timing[valid_timing_nbr].sdadel = a;
              p_prev = p;
              valid_timing_nbr ++;

              if (valid_timing_nbr >= VALID_PRESC_NBR)
              {
                /* max valid timing buffer is full, use only these values*/
                goto Compute_scll_sclh;
              }
            }
          }
        }
      }
      
      if (valid_timing_nbr == 0U)
      {
        return 0;
      }
Compute_scll_sclh:
      tsync = af_delay_min + dnf_delay + (2U * i2cclk);
      s = NULL;
      clk_max = NSEC_PER_SEC / i2c_specs[speed].rate_min; clk_min = NSEC_PER_SEC / i2c_specs[speed].rate_max;
      /* * Among Prescaler possibilities discovered above figures out SCL Low 
         * and High Period. Provided: 
         * - SCL Low Period has to be higher than Low Period of tehs SCL Clock 
         * defined by I2C Specification. I2C Clock has to be lower than 
         * (SCL Low Period - Analog/Digital filters) / 4. 
         * - SCL High Period has to be lower than High Period of the SCL Clock 
         * defined by I2C Specification 
         * - I2C Clock has to be lower than SCL High Period */
      for (uint32_t count = 0; count < valid_timing_nbr; count++)
      {
        uint32_t prescaler = (valid_timing[count].presc + 1U) * i2cclk;
        for (l = 0; l < SCLL_MAX; l++)
        {
          uint32_t tscl_l = ((l + 1U) * prescaler) + tsync;
          if ((tscl_l < i2c_specs[speed].l_min) || (i2cclk >= ((tscl_l - af_delay_min - dnf_delay) / 4U)))
          {
            continue;
          }

          for (h = 0; h < SCLH_MAX; h++)
          {
            uint32_t tscl_h = ((h + 1U) * prescaler) + tsync;
            uint32_t tscl = tscl_l + tscl_h + i2c_user_setup[speed].rise_time + i2c_user_setup[speed].fall_time;

            if ((tscl >= clk_min) && (tscl <= clk_max) && (tscl_h >= i2c_specs[speed].h_min) && (i2cclk < tscl_h))
            {
              int32_t clk_error = (int32_t)tscl - (int32_t)i2cspeed;

              if (clk_error < 0)
              {
                clk_error = -clk_error;
              }
              /* save the solution with the lowest clock error */
              if ((uint32_t)clk_error < clk_error_prev)
              {
                clk_error_prev = (uint32_t)clk_error;
                valid_timing[count].scll = l;
                valid_timing[count].sclh = h;
                s = &valid_timing[count];
              }
            }
          }
        }
      }
      if (s == NULL)
      {
        return 0;
      }

      ret = ((s->presc & 0x0FU) << 28) | ((s->scldel & 0x0FU) << 20) | ((s->sdadel & 0x0FU) << 16) 
            | ((s->sclh & 0xFFU) << 8) | ((s->scll & 0xFFU) << 0);
    }
  }
  return ret;
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)  
/**
  * @brief     BSP I2C1 Bus Msp Callback registering
  * @param[in] Callback: pointer to I2C1 MspInit/MspDeInit callback functions
  * @return    BSP status
  */
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t * const Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  __HAL_I2C_RESET_HANDLE_STATE(&Handle_I2c1);

  /* Register MspInit/MspDeInit Callbacks */
  if (HAL_I2C_RegisterCallback(&Handle_I2c1, HAL_I2C_MSPINIT_CB_ID, Callback->pMspI2cInitCb) != HAL_OK)
  { 
    ret = BSP_ERROR_PERIPH_FAILURE;
  } 
  else if (HAL_I2C_RegisterCallback(&Handle_I2c1, HAL_I2C_MSPDEINIT_CB_ID, Callback->pMspI2cDeInitCb) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    IsI2C1MspCbValid = 1U;
  }

  /* BSP status */
  return ret;
}

/**
  * @brief  Register Default BSP I2C1 Bus Msp Callbacks
  * @param  None
  * @return BSP status
  */
int32_t BSP_I2C1_RegisterDefaultMspCallbacks(void)
{
  int32_t ret = BSP_ERROR_NONE;
  
  __HAL_I2C_RESET_HANDLE_STATE (&Handle_I2c1);

  /* Register default MspInit/MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&Handle_I2c1, HAL_I2C_MSPINIT_CB_ID, I2C1_MspInit) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else if (HAL_I2C_RegisterCallback(&Handle_I2c1, HAL_I2C_MSPDEINIT_CB_ID, I2C1_MspDeInit) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    IsI2C1MspCbValid = 1U;
  }

  /* BSP status */
  return ret;
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief         Initialize I2c MSP.
  * @param[in,out] i2cHandle: Handle to the I2C1 peripheral
  * @return        None
  */
static void I2C1_MspInit(I2C_HandleTypeDef * const i2cHandle)
{
  GPIO_InitTypeDef gpioinitstruct;

  BUS_I2C1_IRQ_GPIO_CLK_ENABLE();
  BUS_I2C1_SCL_GPIO_CLK_ENABLE();
  BUS_I2C1_SDA_GPIO_CLK_ENABLE();
  
  /* I2C1 GPIO Configuration */
  gpioinitstruct.Pin = BUS_I2C1_SCL_GPIO_PIN;
  gpioinitstruct.Mode = BUS_I2C1_SCL_MODE;
  gpioinitstruct.Pull = GPIO_PULLUP;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = BUS_I2C1_SCK_GPIO_AF;
  HAL_GPIO_Init(BUS_I2C1_SCL_GPIO_PORT, &gpioinitstruct);
  
  gpioinitstruct.Pin = BUS_I2C1_SDA_GPIO_PIN;
  gpioinitstruct.Mode = BUS_I2C1_SDA_MODE;
  gpioinitstruct.Pull = GPIO_PULLUP;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = BUS_I2C1_SDA_GPIO_AF;
  HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &gpioinitstruct);

  /* Configure ST25R3916 IRQ */
  gpioinitstruct.Pin        = BUS_I2C1_IRQ_GPIO_PIN;
  gpioinitstruct.Mode       = BUS_I2C1_IRQ_MODE;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUS_I2C1_IRQ_GPIO_PORT, &gpioinitstruct);

  /* Peripheral clock enable */
  BUS_I2C1_CLK_ENABLE();
  
  /* Configure interrupt callback */
  (void)HAL_EXTI_GetHandle(&hExti0, hExti0.Line);  
  (void)HAL_EXTI_RegisterCallback(&hExti0, HAL_EXTI_COMMON_CB_ID, BSP_I2C1_IRQ_Callback);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(BSP_I2C1_IRQ_EXTI_IRQn, BSP_I2C1_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(BSP_I2C1_IRQ_EXTI_IRQn);
    
}

/**
  * @brief         DeInitialize I2C MSP.
  * @param[in,out] i2cHandle: Handle to the I2C1 peripheral
  * @return        None
  */
static void I2C1_MspDeInit(I2C_HandleTypeDef * const i2cHandle)
{
  /* Peripheral clock disable */
  BUS_I2C1_CLK_DISABLE();

  /* I2C1 GPIO Configuration */
  HAL_GPIO_DeInit(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN);

  /* Configure ST25R3916 IRQ */
  HAL_GPIO_DeInit(BUS_I2C1_IRQ_GPIO_PORT, BUS_I2C1_IRQ_GPIO_PIN);
}

/**
  * @brief  BSP SPI1 callback
  * @param  None
  * @return None
  */
__weak void BSP_I2C1_IRQ_Callback(void)
{
  /* Prevent unused argument(s) compilation warning */
  
  /* This function should be implemented by the user application.
   * It is called into this driver when an event from ST25R3916 is triggered. 
   */
  st25r3916Isr();
}

/**
  * @brief  BSP SPI1 interrupt Handler
  * @param  None
  * @return None
  */
void BSP_I2C1_IRQHandler(void)
{
  HAL_EXTI_IRQHandler(&hExti0);
}

/**
  * @brief  BSP SPI1 interrupt Handler
  * @param  None
  * @return None
  */
void BSP_I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&Handle_I2c1);
}

/**
  * @brief  BSP SPI1 interrupt Handler
  * @param  None
  * @return None
  */
void BSP_I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&Handle_I2c1);
}
#endif /* RFAL_USE_I2C */

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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
