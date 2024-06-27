/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "queue.h"
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CW_LINE_DOT		0x18
#define CW_LINE_BLANK	0x00

#define CW_LINE_ACTION_DECODE		1
#define CW_LINE_ACTION_DOT			2
#define CW_LINE_ACTION_BLANK		3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for keyTask */
osThreadId_t keyTaskHandle;
const osThreadAttr_t keyTask_attributes = {
  .name = "keyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for beepTask */
osThreadId_t beepTaskHandle;
const osThreadAttr_t beepTask_attributes = {
  .name = "beepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for keyEvent_Queue */
osMessageQueueId_t keyEvent_QueueHandle;
const osMessageQueueAttr_t keyEvent_Queue_attributes = {
  .name = "keyEvent_Queue"
};
/* USER CODE BEGIN PV */

//
uint8_t fontMap[][16] = {
		{0x00,0xC0,0x40,0x40,0x40,0x40,0xF0,0x40,0x40,0x40,0x40,0x40,0xC0,0x00,0x00,0x00},
		{0x00,0x07,0x02,0x02,0x02,0x02,0x1F,0x02,0x02,0x02,0x02,0x02,0x07,0x00,0x00,0x00},/*,0 */

		{0x20,0x20,0x20,0xE0,0x20,0x20,0x28,0x30,0x20,0x20,0xE0,0x20,0x20,0x20,0x00,0x00},
		{0x10,0x10,0x10,0x08,0x09,0x0A,0x04,0x04,0x0A,0x09,0x08,0x10,0x10,0x10,0x00,0x00},/*,1 */
};

// 128point/1 line
uint8_t cwLineBuff[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTaskKey(void *argument);
void StartTaskBeep(void *argument);
void StartTaskLed(void *argument);

/* USER CODE BEGIN PFP */

void oledShowDemo(void);
void oledShowCWLine(uint8_t action);
void oledCwLineBuffInit(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void oledShowDemo (void)
{
	// posD: page=b0-b7, col-lower=0x00-0x0f, col-upper=0x10-0x1f
  uint8_t posD[] = { 0xb0, 0x00, 0x10 };
  ssd1306_show (posD, 3, &fontMap[0][0], 16);
  posD[0] = 0xb1;	// page1
  posD[1] = 0x00;
  ssd1306_show (posD, 3, &fontMap[1][0], 16);
  posD[0] = 0xb0;
  posD[2] = 0x11;
  ssd1306_show (posD, 3, &fontMap[2][0], 16);
  posD[0] = 0xb1;
  posD[2] = 0x11;
  ssd1306_show (posD, 3, &fontMap[3][0], 16);
}

void oledShowCWLine(uint8_t action) {
	// process cwLineBuff by action
	if(action == CW_LINE_ACTION_DOT) {
		// array move left 2 index, last 2 byte set CW_LINE_DOT
		for(uint8_t i = 0; i < 128-2; i++) {
			cwLineBuff[i] = cwLineBuff[i+2];
		}
		cwLineBuff[126] = CW_LINE_DOT;
		cwLineBuff[127] = CW_LINE_DOT;
	} else if(action == CW_LINE_ACTION_BLANK) {
		for(uint8_t i = 0; i < 128-2; i++) {
			cwLineBuff[i] = cwLineBuff[i+2];
		}
		cwLineBuff[126] = CW_LINE_BLANK;
		cwLineBuff[127] = CW_LINE_BLANK;
	} else if(action == CW_LINE_ACTION_DECODE) {
		for(uint8_t i = 0; i < 128-6; i++) {
			cwLineBuff[i] = cwLineBuff[i+6];
		}
		cwLineBuff[122] = CW_LINE_BLANK;
		cwLineBuff[123] = CW_LINE_BLANK;
		cwLineBuff[124] = CW_LINE_BLANK;
		cwLineBuff[125] = CW_LINE_BLANK;
		cwLineBuff[126] = CW_LINE_BLANK;
		cwLineBuff[127] = CW_LINE_BLANK;
	} else {
		//TODO out of action
	}

	// page0 info h
	// page1 info l
	// page2 graph h
	// page3 graph l
	// page4 ------
	//*page5 cwline
	// page6 decode h
	// page7 decode l
	uint8_t posD[] = { 0xb5, 0x00, 0x10 };
	ssd1306_show (posD, 3, cwLineBuff, 128);
}

void oledCwLineBuffInit(void) {
	for(uint8_t i = 0; i < 128; i++) {
		cwLineBuff[i] = 0x00;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of keyEvent_Queue */
  keyEvent_QueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &keyEvent_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of keyTask */
  keyTaskHandle = osThreadNew(StartTaskKey, NULL, &keyTask_attributes);

  /* creation of beepTask */
  beepTaskHandle = osThreadNew(StartTaskBeep, NULL, &beepTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartTaskLed, NULL, &ledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CWKEY_Pin */
  GPIO_InitStruct.Pin = CWKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CWKEY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t keyEventMsg = 0;
	osStatus_t status = 0;
	uint8_t cwLineStatus = 0;
	uint8_t cwLineShowDelay = 0;

	ssd1306_init(&hi2c1);

  /* Infinite loop */
  for(;;)
  {
  	status = osMessageQueueGet(keyEvent_QueueHandle, &keyEventMsg, NULL, 0);
  	if(status == osOK) { // received, switch show_status of cw_line.
  		if(keyEventMsg == QUEUE_MSG_KEYDOWN) {
  			cwLineStatus = SHOW_STATUS_DOWN;
  		} else if(keyEventMsg == QUEUE_MSG_KEYUP) {
  			cwLineStatus = SHOW_STATUS_UP;
  		} else if(keyEventMsg == QUEUE_MSG_DECODE) {
  			cwLineStatus = SHOW_STATUS_DECODE;
  		}
  		cwLineShowDelay = 0;
  	}

		// process cw_line
		if(cwLineStatus == SHOW_STATUS_DOWN) {
			cwLineShowDelay++;
			if(cwLineShowDelay > CW_LINE_SHOW_DELAY_COUNT_MAX) {
				oledShowCWLine(CW_LINE_ACTION_DOT);
				cwLineShowDelay = 0;
			}
			osStatus_t beepThreadStatus = osThreadResume(beepTaskHandle);
			if(beepThreadStatus == osOK) {
				//TODO open beep
			}
		} else  if(cwLineStatus == SHOW_STATUS_UP) {
			cwLineShowDelay++;
			if(cwLineShowDelay > CW_LINE_SHOW_DELAY_COUNT_MAX) {
				oledShowCWLine(CW_LINE_ACTION_BLANK);
				cwLineShowDelay = 0;
			}
			osStatus_t beepThreadStatus = osThreadSuspend(beepTaskHandle);
			if(beepThreadStatus == osOK) {
				//TODO stop beep
			}
		} else if(cwLineStatus == SHOW_STATUS_DECODE) {
			oledShowCWLine(CW_LINE_ACTION_DECODE);
			cwLineStatus = SHOW_STATUS_IDLE;
		} else { // cw_line is IDLE
			osStatus_t beepThreadStatus = osThreadSuspend(beepTaskHandle);
			if(beepThreadStatus == osOK) {
				//TODO stop beep
			}
		}

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskKey */
/**
* @brief Function implementing the keyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskKey */
void StartTaskKey(void *argument)
{
  /* USER CODE BEGIN StartTaskKey */
	uint8_t keyStatus = KEY_STATUS_IDLE;
	uint8_t keyQueueMsg = 0;
	uint16_t deboundCount = 0;
	uint16_t decodeWaitCount = 0;
  /* Infinite loop */
  for(;;)
  {
  	GPIO_PinState keyPinState = HAL_GPIO_ReadPin(CWKEY_GPIO_Port, CWKEY_Pin);

  	if(keyStatus == KEY_STATUS_DOWN) {
  		if(keyPinState == GPIO_PIN_SET) {	// key is UP
  			deboundCount++;
  			if(deboundCount > DEBOUND_COUNT_MAX) {
  				keyQueueMsg = QUEUE_MSG_KEYUP;
  				osStatus_t status = osMessageQueuePut(keyEvent_QueueHandle, &keyQueueMsg, 0, osWaitForever);
  				if (status != osOK) {
  				    //TODO 错误处理
  				} else {
  					deboundCount = 0;
  					keyStatus = KEY_STATUS_UP;
  				}
  			}
  		} else {	// keyPinState == GPIO_PIN_RESET // key is DOWNing
				deboundCount = 0;
			}

  	} else if(keyStatus == KEY_STATUS_UP) {
  		if(keyPinState == GPIO_PIN_SET) {
  			decodeWaitCount++;
  			if(decodeWaitCount > DECODE_COUNT_MAX) {
  				keyQueueMsg = QUEUE_MSG_DECODE;
  				osStatus_t status = osMessageQueuePut(keyEvent_QueueHandle, &keyQueueMsg, 0, osWaitForever);
					if (status != osOK) {
							//TODO 错误处理
					} else {
						decodeWaitCount = 0;
	  				keyStatus = KEY_STATUS_IDLE;
					}
  			}
  		} else {	//keyPinState == GPIO_PIN_RESET	// key is DOWN
  			decodeWaitCount = 0;
  			keyQueueMsg = QUEUE_MSG_KEYDOWN;
  			osStatus_t status = osMessageQueuePut(keyEvent_QueueHandle, &keyQueueMsg, 0, osWaitForever);
				if (status != osOK) {
						//TODO 错误处理
				} else {
	  			keyStatus = KEY_STATUS_DOWN;
				}
  		}

  	} else { //keyStatus == KEY_STATUS_IDLE
  		if(keyPinState == GPIO_PIN_RESET) {	// key is DOWN
  			keyQueueMsg = QUEUE_MSG_KEYDOWN;
  			osStatus_t status = osMessageQueuePut(keyEvent_QueueHandle, &keyQueueMsg, 0, osWaitForever);
				if (status != osOK) {
						//TODO 错误处理
				} else {
					keyStatus = KEY_STATUS_DOWN;
				}
  		}
  	}

    osDelay(1);
  }
  /* USER CODE END StartTaskKey */
}

/* USER CODE BEGIN Header_StartTaskBeep */
/**
* @brief Function implementing the beepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBeep */
void StartTaskBeep(void *argument)
{
  /* USER CODE BEGIN StartTaskBeep */
  /* Infinite loop */
  for(;;)
  {
  	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
  	osDelay(1);
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		osDelay(1);
  }
  /* USER CODE END StartTaskBeep */
}

/* USER CODE BEGIN Header_StartTaskLed */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed */
void StartTaskLed(void *argument)
{
  /* USER CODE BEGIN StartTaskLed */
  /* Infinite loop */
  for(;;)
  {
  	osDelay(1);
  }
  /* USER CODE END StartTaskLed */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
