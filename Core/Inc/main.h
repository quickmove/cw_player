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
#define BEEP_Pin GPIO_PIN_4
#define BEEP_GPIO_Port GPIOB
#define CWKEY_Pin GPIO_PIN_5
#define CWKEY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//
#define KEY_STATUS_IDLE	0
#define KEY_STATUS_DOWN	1
#define KEY_STATUS_UP		2

//
#define SHOW_STATUS_IDLE		0
#define SHOW_STATUS_DOWN		1
#define SHOW_STATUS_UP			2
#define SHOW_STATUS_DECODE	3


//
#define DEBOUND_DELAY_MAX	3

// when key is down to up,delay count time to decode cw
#define DECODE_DELAY_MAX	150

// delay x count for show dot or blank
#define CW_LINE_SHOW_DELAY_MAX	30

// delay x count for check di or da (_ or .)
#define CW_LINE_CHECK_DIDA_DELAY_MAX	3

//
#define QUEUE_MSG_KEYDOWN	0
#define QUEUE_MSG_KEYUP		1
#define QUEUE_MSG_DECODE	2


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
