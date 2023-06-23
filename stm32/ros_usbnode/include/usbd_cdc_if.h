/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(const void* Buf, uint32_t Len);
uint8_t CDC_TransmitTimed_FS(const void* Buf, uint32_t Len, uint32_t TimeoutMs);

void CDC_ResumeTransmit_FS(void);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
uint8_t CDC_IsBusy();
uint32_t CDC_RXQueue_Dequeue(void* Dst, uint32_t MaxLen);
uint32_t CDC_TXQueue_GetReadAvailable();
uint32_t CDC_TXQueue_GetWriteAvailable();
uint32_t CDC_RXQueue_GetReadAvailable();
uint32_t CDC_RXQueue_GetWriteAvailable();
uint32_t CDC_GetDroppedTxPackets();
uint32_t CDC_GetDroppedRxPackets();
void CDC_ResetDroppedTxPackets();
void CDC_ResetDroppedRxPackets();
uint8_t CDC_DataReceivedHandler(const uint8_t *Data, uint32_t len);
uint32_t CDC_GetLastTransmitStartTick();
uint32_t CDC_GetLastTransmitCompleteTick();
uint8_t CDC_IsComportOpen();

/**
 * @brief  CDC_TransmitString
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *
 *
 * @param  string: 0-terminated C-string to send
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
static inline uint8_t CDC_TransmitString_FS(const char *string)
{
    return CDC_Transmit_FS(string, strlen(string));
}

/* USER CODE END EXPORTED_FUNCTIONS */

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

#endif /* __USBD_CDC_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

