
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos.c
  * @author  MCD Application Team
  * @brief   azure_rtos application implementation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "app_azure_rtos.h"
#include "stm32g0xx.h"
#include "usbpd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN TX_Pool_Buffer */
/* USER CODE END TX_Pool_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE] __ALIGN_END;
static TX_BYTE_POOL tx_app_byte_pool;

/* USER CODE BEGIN UX_HOST_Pool_Buffer */
/* USER CODE END UX_HOST_Pool_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static UCHAR ux_host_byte_pool_buffer[UX_HOST_APP_MEM_POOL_SIZE] __ALIGN_END;
static TX_BYTE_POOL ux_host_app_byte_pool;

/* USER CODE BEGIN USBPD_Pool_Buffer */

/* USER CODE END USBPD_Pool_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
static UCHAR usbpd_byte_pool_buffer[USBPD_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL usbpd_app_byte_pool;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */

  /* USER CODE END  tx_application_define */

  VOID *memory_ptr;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */
    Error_Handler();
    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&tx_app_byte_pool;

    if (App_ThreadX_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  App_ThreadX_Init_Error */
      Error_Handler();
      /* USER CODE END  App_ThreadX_Init_Error */
    }

    /* USER CODE BEGIN  App_ThreadX_Init_Success */

    /* USER CODE END  App_ThreadX_Init_Success */

  }

  if (tx_byte_pool_create(&ux_host_app_byte_pool, "Ux App memory pool", ux_host_byte_pool_buffer, UX_HOST_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */
    Error_Handler();
    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN UX_HOST_Byte_Pool_Success */

    /* USER CODE END UX_HOST_Byte_Pool_Success */

    memory_ptr = (VOID *)&ux_host_app_byte_pool;

    if (MX_USBX_Host_Init(memory_ptr) != UX_SUCCESS)
    {
      /* USER CODE BEGIN MX_USBX_Host_Init_Error */
      Error_Handler();
      /* USER CODE END MX_USBX_Host_Init_Error */
    }

    /* USER CODE BEGIN MX_USBX_Host_Init_Success */

    /* USER CODE END MX_USBX_Host_Init_Success */
  }

  if (tx_byte_pool_create(&usbpd_app_byte_pool, "USBPD App memory pool", usbpd_byte_pool_buffer, USBPD_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN USBPD_Byte_Pool_Error */
      Error_Handler();
    /* USER CODE END USBPD_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN USBPD_Byte_Pool_Success */

    /* USER CODE END USBPD_Byte_Pool_Success */

    memory_ptr = (VOID *)&usbpd_app_byte_pool;
    if (MX_USBPD_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  MX_USBPD_Init_Error */

      Error_Handler();
      /* USER CODE END  MX_USBPD_Init_Error */
    }
    /* USER CODE BEGIN  MX_USBPD_Init_Success */

    /* USER CODE END  MX_USBPD_Init_Success */
  }
}

/* USER CODE BEGIN  0 */

/* USER CODE END  0 */
