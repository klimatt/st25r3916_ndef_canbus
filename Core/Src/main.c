/**
  ******************************************************************************
 * @file    main.c 
 * @author  MMY Application Team
 * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "demo.h"
#include "platform.h"
#include "logger.h"
#include "st_errno.h"
#include "rfal_rf.h"
#include "rfal_analogConfig.h"


#define RTT_ON;

#ifdef RTT_ON
  #include "SEGGER_RTT.h"
#endif

//#include "SEGGER_RTT.h"

//#include "stm32l4xx_hal_can.h"


/** @addtogroup X-CUBE-NFC6_Applications
 *  @{
 */

/** @addtogroup PollingTagDetect
 *  @{
 */

/** @addtogroup PTD_Main 
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup PTD_Main_Private_Variables 
 * @{
 */
uint8_t globalCommProtectCnt = 0;   /*!< Global Protection counter     */
UART_HandleTypeDef hlogger;         /*!< Handler to the UART HW logger */
CAN_HandleTypeDef hcan1;
IWDG_HandleTypeDef hiwdg;
/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_CAN1_Init(void);
static void gpio_can_init(void);
static void MX_IWDG_Init(void);


void can_irq(CAN_HandleTypeDef *pcan) {
  CAN_RxHeaderTypeDef msg;
  uint8_t data[8];
  HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &msg, data);
  // do something
}

HAL_StatusTypeDef can_init(CAN_HandleTypeDef *hcan)
{
  uint32_t tickstart;

  /* Check CAN handle */
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CAN_ALL_INSTANCE(hcan->Instance));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.TimeTriggeredMode));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.AutoBusOff));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.AutoWakeUp));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.AutoRetransmission));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.ReceiveFifoLocked));
  assert_param(IS_FUNCTIONAL_STATE(hcan->Init.TransmitFifoPriority));
  assert_param(IS_CAN_MODE(hcan->Init.Mode));
  assert_param(IS_CAN_SJW(hcan->Init.SyncJumpWidth));
  assert_param(IS_CAN_BS1(hcan->Init.TimeSeg1));
  assert_param(IS_CAN_BS2(hcan->Init.TimeSeg2));
  assert_param(IS_CAN_PRESCALER(hcan->Init.Prescaler));

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
  if (hcan->State == HAL_CAN_STATE_RESET)
  {
    /* Reset callbacks to legacy functions */
    hcan->RxFifo0MsgPendingCallback  =  HAL_CAN_RxFifo0MsgPendingCallback;  /* Legacy weak RxFifo0MsgPendingCallback  */
    hcan->RxFifo0FullCallback        =  HAL_CAN_RxFifo0FullCallback;        /* Legacy weak RxFifo0FullCallback        */
    hcan->RxFifo1MsgPendingCallback  =  HAL_CAN_RxFifo1MsgPendingCallback;  /* Legacy weak RxFifo1MsgPendingCallback  */
    hcan->RxFifo1FullCallback        =  HAL_CAN_RxFifo1FullCallback;        /* Legacy weak RxFifo1FullCallback        */
    hcan->TxMailbox0CompleteCallback =  HAL_CAN_TxMailbox0CompleteCallback; /* Legacy weak TxMailbox0CompleteCallback */
    hcan->TxMailbox1CompleteCallback =  HAL_CAN_TxMailbox1CompleteCallback; /* Legacy weak TxMailbox1CompleteCallback */
    hcan->TxMailbox2CompleteCallback =  HAL_CAN_TxMailbox2CompleteCallback; /* Legacy weak TxMailbox2CompleteCallback */
    hcan->TxMailbox0AbortCallback    =  HAL_CAN_TxMailbox0AbortCallback;    /* Legacy weak TxMailbox0AbortCallback    */
    hcan->TxMailbox1AbortCallback    =  HAL_CAN_TxMailbox1AbortCallback;    /* Legacy weak TxMailbox1AbortCallback    */
    hcan->TxMailbox2AbortCallback    =  HAL_CAN_TxMailbox2AbortCallback;    /* Legacy weak TxMailbox2AbortCallback    */
    hcan->SleepCallback              =  HAL_CAN_SleepCallback;              /* Legacy weak SleepCallback              */
    hcan->WakeUpFromRxMsgCallback    =  HAL_CAN_WakeUpFromRxMsgCallback;    /* Legacy weak WakeUpFromRxMsgCallback    */
    hcan->ErrorCallback              =  HAL_CAN_ErrorCallback;              /* Legacy weak ErrorCallback              */

    if (hcan->MspInitCallback == NULL)
    {
      hcan->MspInitCallback = HAL_CAN_MspInit; /* Legacy weak MspInit */
    }

    /* Init the low level hardware: CLOCK, NVIC */
    hcan->MspInitCallback(hcan);
  }

#else
  if (hcan->State == HAL_CAN_STATE_RESET)
  {
    /* Init the low level hardware: CLOCK, NVIC */
    gpio_can_init();
  }
#endif /* (USE_HAL_CAN_REGISTER_CALLBACKS) */

  /* Exit from sleep mode */
  CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_SLEEP);

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check Sleep mode leave acknowledge */
  while ((hcan->Instance->MSR & CAN_MSR_SLAK) != 0U)
  {
    if ((HAL_GetTick() - tickstart) > 10)
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

      /* Change CAN state */
      hcan->State = HAL_CAN_STATE_ERROR;

      return HAL_ERROR;
    }
  }

  /* Request initialisation */
  SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait initialisation acknowledge */
  while ((hcan->Instance->MSR & CAN_MSR_INAK) == 0U)
  {
    if ((HAL_GetTick() - tickstart) > 10)
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

      /* Change CAN state */
      hcan->State = HAL_CAN_STATE_ERROR;

      return HAL_ERROR;
    }
  }

  /* Set the time triggered communication mode */
  if (hcan->Init.TimeTriggeredMode == ENABLE)
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_TTCM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_TTCM);
  }

  /* Set the automatic bus-off management */
  if (hcan->Init.AutoBusOff == ENABLE)
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
  }

  /* Set the automatic wake-up mode */
  if (hcan->Init.AutoWakeUp == ENABLE)
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_AWUM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_AWUM);
  }

  /* Set the automatic retransmission */
  if (hcan->Init.AutoRetransmission == ENABLE)
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_NART);
  }
  else
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_NART);
  }

  /* Set the receive FIFO locked mode */
  if (hcan->Init.ReceiveFifoLocked == ENABLE)
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_RFLM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_RFLM);
  }

  /* Set the transmit FIFO priority */
  if (hcan->Init.TransmitFifoPriority == ENABLE)
  {
    SET_BIT(hcan->Instance->MCR, CAN_MCR_TXFP);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_TXFP);
  }

  /* Set the bit timing register */
  /*WRITE_REG(hcan->Instance->BTR, (uint32_t)(hcan->Init.Mode           |
                                            hcan->Init.SyncJumpWidth  |
                                            hcan->Init.TimeSeg1       |
                                            hcan->Init.TimeSeg2       |
                                            (hcan->Init.Prescaler - 1U)));
*/
  WRITE_REG(hcan->Instance->BTR, (uint32_t)(
    hcan->Init.Mode | 
    hcan->Init.SyncJumpWidth << 24 |
    hcan->Init.TimeSeg2 << 20 |
    hcan->Init.TimeSeg1 << 16 |
    hcan->Init.Prescaler
  )
  
  );

  //WRITE_REG(hcan->Instance->BTR, (uint32_t)0x41320009);
  /* Initialize the error code */
  hcan->ErrorCode = HAL_CAN_ERROR_NONE;

  /* Initialize the CAN state */
  hcan->State = HAL_CAN_STATE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Deinitializes the CAN peripheral registers to their default
  *         reset values.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_DeInit(CAN_HandleTypeDef *hcan)
{
  /* Check CAN handle */
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CAN_ALL_INSTANCE(hcan->Instance));

  /* Stop the CAN module */
  (void)HAL_CAN_Stop(hcan);

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
  if (hcan->MspDeInitCallback == NULL)
  {
    hcan->MspDeInitCallback = HAL_CAN_MspDeInit; /* Legacy weak MspDeInit */
  }

  /* DeInit the low level hardware: CLOCK, NVIC */
  hcan->MspDeInitCallback(hcan);

#else
  /* DeInit the low level hardware: CLOCK, NVIC */
  HAL_CAN_MspDeInit(hcan);
#endif /* (USE_HAL_CAN_REGISTER_CALLBACKS) */

  /* Reset the CAN peripheral */
  SET_BIT(hcan->Instance->MCR, CAN_MCR_RESET);

  /* Reset the CAN ErrorCode */
  hcan->ErrorCode = HAL_CAN_ERROR_NONE;

  /* Change CAN state */
  hcan->State = HAL_CAN_STATE_RESET;

  /* Return function status */
  return HAL_OK;
}

HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);

  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(pHeader->IDE));
  assert_param(IS_CAN_RTR(pHeader->RTR));
  assert_param(IS_CAN_DLC(pHeader->DLC));
  if (pHeader->IDE == CAN_ID_STD)
  {
    assert_param(IS_CAN_STDID(pHeader->StdId));
  }
  else
  {
    assert_param(IS_CAN_EXTID(pHeader->ExtId));
  }
  assert_param(IS_FUNCTIONAL_STATE(pHeader->TransmitGlobalTime));

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INTERNAL;

        return HAL_ERROR;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
      if (pHeader->IDE == CAN_ID_STD)
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) |
                                                           pHeader->RTR);
      }
      else
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->ExtId << CAN_TI0R_EXID_Pos) |
                                                           pHeader->IDE |
                                                           pHeader->RTR);
      }

      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TDTR = (pHeader->DLC);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR,
                ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR,
                ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);
      /* Return function status */
      return HAL_OK;
    }
    else
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

      return HAL_ERROR;
    }
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}

/* Private functions ---------------------------------------------------------*/
/** @defgroup PTD_Main_Private_Functions
 * @{
 */
/**
  *****************************************************************************
  * @brief  Main program
  *
  * @param  None
  *
  * @return None
  *****************************************************************************
  */


//#define NFC_TL
//#define NFC_TR 
//#define NFC_BL 
#define NFC_BR

#ifdef NFC_TL
  uint32_t CAN_ID = 0x1000a00a;
  uint32_t HB_ID = 0x1002bc0a;
#endif

#ifdef NFC_TR
  uint32_t CAN_ID = 0x1000a10a;
  uint32_t HB_ID = 0x1002bd0a;
#endif

#ifdef NFC_BL
  uint32_t CAN_ID = 0x1000a20a;
  uint32_t HB_ID = 0x1002be0a;
#endif

#ifdef NFC_BR
  uint32_t CAN_ID = 0x1000a30a;
  uint32_t HB_ID = 0x1002bf0a;
#endif

#define ERROR_LED GPIO_PIN_3
#define STATE_LED GPIO_PIN_4
#define LEDS_PORT GPIOA
const uint32_t CNT_LIMIT = 40000;

int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  #ifdef RTT_ON
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  #endif
  HAL_Init();

  /* Configure the System clock to have a frequency of 80 MHz */
  SystemClock_Config();
  MX_IWDG_Init();
  HAL_IWDG_Refresh(&hiwdg);

   //gpio_can_init();
   //logUsartInit(&hlogger);
  MX_CAN1_Init();

  //NFC06A1_LED_Init();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = ERROR_LED | STATE_LED;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEDS_PORT, &GPIO_InitStruct);
  
  /* Configure Led pin Output Level as off */
  HAL_GPIO_WritePin(LEDS_PORT, ERROR_LED, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_RESET);
  HAL_IWDG_Refresh(&hiwdg);

  //BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
#ifdef RFAL_USE_I2C
  BSP_I2C1_Init();
#else
  BSP_SPI1_Init();
#endif /* RFAL_USE_I2C */  
  /* Initialize log module */
  
   #ifdef RTT_ON
    SEGGER_RTT_printf(0,"Welcome to X-NUCLEO-NFC06A1 + CAN-Bus\r\n");
   #endif
  
  //MX_GPIO_Init();
 
  //HAL_CAN_MspInit(&hcan1);

  /* Initalize RFAL */
  if( !demoIni() )
  {
    /*
    * in case the rfal initalization failed signal it by flashing all LED
    * and stoping all operations
    */
    #ifdef RTT_ON
      SEGGER_RTT_printf(0,"Initialization failed..\r\n");
    #endif
    Error_Handler();
  } 
  else
  {
    #ifdef RTT_ON
      SEGGER_RTT_printf(0,"Initialization succeeded..\r\n");
    #endif
    HAL_GPIO_TogglePin(LEDS_PORT, ERROR_LED);
    HAL_GPIO_TogglePin(LEDS_PORT, STATE_LED);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LEDS_PORT, ERROR_LED, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_RESET);
  }

  CAN_FilterTypeDef sf;
  sf.FilterMaskIdHigh = (uint16_t)(0);
  sf.FilterMaskIdLow = (uint16_t)(0);
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK) {
    #ifdef RTT_ON
      SEGGER_RTT_printf(0,"CAN_Filter config Fail: 0x%08x \r\n", hcan1.ErrorCode);
    #endif
    Error_Handler();
  }
  HAL_IWDG_Refresh(&hiwdg);

  /*if (HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq)) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }*/

  uint32_t mb;
  CAN_TxHeaderTypeDef msg;
  

  
  msg.IDE = CAN_ID_EXT;
  msg.RTR = CAN_RTR_DATA;
  //msg.DLC = 7;
  msg.TransmitGlobalTime = DISABLE;

  uint32_t counter = 0;

  
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    #ifdef RTT_ON
      SEGGER_RTT_printf(0,"CAN_Start Fail: 0x%08x \r\n", hcan1.ErrorCode);
    #endif
     Error_Handler();
  }
  if (HAL_CAN_WakeUp (&hcan1) != HAL_OK) {
    #ifdef RTT_ON
      SEGGER_RTT_printf(0,"CAN_Wakeup Fail: 0x%08x \r\n", hcan1.ErrorCode);
    #endif
     Error_Handler();
  }
  rfalNfcDevice nfcDevice;
  /* Infinite loop */

  uint8_t data[128] = { 0 };
  uint8_t id[128] = { 0 };
  uint8_t data_len = 0, id_len = 0;
  uint16_t rssi = 0;
  while (1)
  { 
   HAL_IWDG_Refresh(&hiwdg);
   if (get_data(&id, &data, &id_len, &data_len, &rssi))
    {
      counter = 0;
      HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_SET);
      HAL_Delay(10);
      HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_RESET);
      #ifdef RTT_ON
        SEGGER_RTT_printf(0,"=============== \r\n");
        SEGGER_RTT_printf(0,"id_len: %d \r\n", id_len);
        for(uint8_t i = 0; i < id_len; i++){
          SEGGER_RTT_printf(0,"id[%d]: 0x%2x \r\n", i, id[i]);
        }
        SEGGER_RTT_printf(0,"data_len: %d \r\n", data_len);
        for(uint8_t i = 0; i < data_len; i++){
          SEGGER_RTT_printf(0,"data[%d]: 0x%2x \r\n", i, data[i]);
        }
        SEGGER_RTT_printf(0,"=============== \r\n");
      #endif
      #ifdef RTT_ON
        SEGGER_RTT_printf(0,"CAN_State: 0x%08x \r\n", HAL_CAN_GetState(&hcan1));
      #endif


      int16_t value_from_ascii = atoi((char*)&data[0]);
      uint8_t buff_to_send[4];
      memcpy((uint8_t*)&buff_to_send[0], (uint8_t*)&value_from_ascii, 2);
      memcpy((uint8_t*)&buff_to_send[2], (uint8_t*)&rssi, 2);

      #ifdef RTT_ON
        SEGGER_RTT_printf(0,"int val: %d :: 0x%2x\r\n", value_from_ascii, value_from_ascii);
        SEGGER_RTT_printf(0,"RSSI: %d\r\n", rssi);
      #endif
       
      for(uint8_t i = 0; i < 128; i++) {
        data[i] = 0;
        id[i] = 0;
      }
      data_len = 0;
      id_len = 0;
      rssi = 0;

      msg.ExtId = CAN_ID;
      msg.DLC = 4;  

      if (can_send(&hcan1, &msg, buff_to_send, &mb) != HAL_OK) {
        #ifdef RTT_ON
          SEGGER_RTT_printf(0,"CAN_SEND Fail: 0x%08x \r\n", hcan1.ErrorCode);
        #endif
        //Error_Handler();
      }
    }
    if (counter == CNT_LIMIT)
    {
      HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_SET);
      HAL_Delay(10);
      msg.ExtId = HB_ID;
      msg.DLC = 0;  
      if (can_send(&hcan1, &msg, NULL, &mb) != HAL_OK) {
        #ifdef RTT_ON
          SEGGER_RTT_printf(0,"CAN_SEND Fail: 0x%08x \r\n", hcan1.ErrorCode);
        #endif
        //Error_Handler();
      }
      counter = 0;
    }
    HAL_GPIO_WritePin(LEDS_PORT, STATE_LED, GPIO_PIN_RESET);
    counter++;
  }
}

/**
  *****************************************************************************
  * @brief  This function is executed in case of error occurrence.
  *
  * @param  None
  *
  * @return None
  *****************************************************************************
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    HAL_GPIO_TogglePin(LEDS_PORT, ERROR_LED);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */ 
}

static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = 1;
  hcan1.Init.TimeSeg1 = 2;
  hcan1.Init.TimeSeg2 = 3;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (can_init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}


static void gpio_can_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    //__HAL_RCC_CAN1_CLK_ENABLE();

     __IO uint32_t tmpreg; 
     SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_CAN1EN); 
     //Delay after an RCC peripheral clock enabling  
     tmpreg = READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_CAN1EN); 
     UNUSED(tmpreg); 
                                         



    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* CAN1 interrupt Init */
    /*HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);*/
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */

}

static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

#ifdef USE_FULL_ASSERT

/**
  *****************************************************************************
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  *
  * @param[in]  file: pointer to the source file name
  * @param[in]  line: assert_param error line source number
  *
  * @return None
  *****************************************************************************
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

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
