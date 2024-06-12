/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_10
#define KEY0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// 按键扫描状态机
#define KEY_STATUS_DOWN	0
#define KEY_STATUS_UP		1
#define KEY_STATUS_IDLE	2

// 按键防抖计数器
#define DEBOUND_COUNT_MAX	65535

// 按键抬起后延迟一段时间，用于触发CW解码
#define DECODE_COUNT_MAX	65535

// 队列消息
#define QUEUE_MSG_KEYDOWN	0
#define QUEUE_MSG_KEYUP		1
#define QUEUE_MSG_DECODE	2

// 队列大小
#define QUEUE_LENGTH	20
#define QUEUE_ITEM_SIZE	sizeof(uint8_t)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
