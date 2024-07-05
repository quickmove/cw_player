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
#include "delay.h"

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

TIM_HandleTypeDef htim2;

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
		{0x00,0x00,0xE0,0x38,0x78,0xC0,0x00,0x00,0x00,0x0E,0x03,0x02,0x02,0x03,0x0C,0x00},/*"A",0*/
		{0x00,0xF8,0x88,0x88,0x88,0x58,0x30,0x00,0x00,0x0F,0x08,0x08,0x08,0x0D,0x07,0x00},/*"B",1*/
		{0x00,0xF0,0x18,0x08,0x08,0x18,0x30,0x00,0x00,0x07,0x0C,0x08,0x08,0x0C,0x06,0x00},/*"C",2*/
		{0x00,0xF8,0x08,0x08,0x08,0xF8,0xE0,0x00,0x00,0x0F,0x08,0x08,0x08,0x0F,0x03,0x00},/*"D",3*/
		{0x00,0xF8,0x88,0x88,0x88,0x88,0x08,0x00,0x00,0x0F,0x08,0x08,0x08,0x08,0x08,0x00},/*"E",4*/
		{0x00,0xF8,0x88,0x88,0x88,0x88,0x08,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00},/*"F",5*/
		{0x00,0xF0,0x18,0x08,0x88,0x98,0x90,0x00,0x00,0x07,0x0C,0x08,0x08,0x0C,0x03,0x00},/*"G",6*/
		{0x00,0xF8,0x80,0x80,0x80,0xF8,0xF8,0x00,0x00,0x0F,0x00,0x00,0x00,0x0F,0x0F,0x00},/*"H",7*/
		{0x00,0x08,0x08,0xF8,0xF8,0x08,0x00,0x00,0x00,0x08,0x08,0x0F,0x0F,0x08,0x00,0x00},/*"I",8*/
		{0x00,0x00,0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x06,0x08,0x08,0x08,0x07,0x00,0x00},/*"J",9*/
		{0x00,0xF8,0x80,0x80,0xC0,0x30,0x08,0x00,0x00,0x0F,0x00,0x00,0x01,0x06,0x08,0x00},/*"K",10*/
		{0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x08,0x08,0x08,0x08,0x00},/*"L",11*/
		{0x00,0xF8,0x38,0xC0,0xE0,0x18,0xF8,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x0F,0x00},/*"M",12*/
		{0x00,0xF8,0x18,0xE0,0x00,0xF8,0xF8,0x00,0x00,0x0F,0x00,0x00,0x03,0x0C,0x0F,0x00},/*"N",13*/
		{0x00,0xF0,0x18,0x08,0x08,0xF8,0xE0,0x00,0x00,0x07,0x0C,0x08,0x08,0x0F,0x03,0x00},/*"O",14*/
		{0x00,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00},/*"P",15*/
		{0x00,0xF0,0x18,0x08,0x08,0x18,0xF0,0x00,0x00,0x07,0x0C,0x08,0x18,0x3C,0x27,0x00},/*"Q",16*/
		{0x00,0xF8,0x88,0x88,0x88,0xD8,0x70,0x00,0x00,0x0F,0x00,0x00,0x01,0x0E,0x08,0x00},/*"R",17*/
		{0x00,0x30,0x58,0x88,0x88,0x98,0x10,0x00,0x00,0x04,0x0C,0x08,0x08,0x0D,0x07,0x00},/*"S",18*/
		{0x00,0x08,0x08,0xF8,0x08,0x08,0x08,0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x00},/*"T",19*/
		{0x00,0xF8,0x00,0x00,0x00,0xF8,0xF8,0x00,0x00,0x07,0x0C,0x08,0x08,0x0F,0x03,0x00},/*"U",20*/
		{0x00,0x38,0xE0,0x00,0x00,0xE0,0x18,0x00,0x00,0x00,0x03,0x0E,0x0E,0x01,0x00,0x00},/*"V",21*/
		{0x08,0xF0,0x00,0xF8,0xF8,0x00,0xF8,0x00,0x00,0x0F,0x0F,0x00,0x03,0x0C,0x07,0x00},/*"W",22*/
		{0x00,0x08,0x30,0xC0,0xE0,0x30,0x08,0x00,0x00,0x0C,0x06,0x01,0x01,0x06,0x08,0x00},/*"X",23*/
		{0x08,0x18,0x60,0x80,0xC0,0x70,0x18,0x00,0x00,0x00,0x00,0x0F,0x01,0x00,0x00,0x00},/*"Y",24*/
		{0x00,0x08,0x08,0x88,0x68,0x38,0x08,0x00,0x00,0x0C,0x0E,0x09,0x08,0x08,0x08,0x00},/*"Z",25*/
		{0x00,0x00,0x08,0x88,0x88,0x70,0x00,0x00,0x00,0x00,0x08,0x0B,0x00,0x00,0x00,0x00},/*"?",26*/
		{0x00,0x30,0x10,0x08,0xF8,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x0F,0x08,0x08,0x00},/*"1",27*/
		{0x00,0x30,0x18,0x08,0x08,0xD8,0x20,0x00,0x00,0x08,0x0C,0x0A,0x09,0x08,0x08,0x00},/*"2",28*/
		{0x00,0x08,0x08,0x48,0x68,0x98,0x00,0x00,0x00,0x06,0x08,0x08,0x08,0x0F,0x03,0x00},/*"3",29*/
		{0x00,0x80,0xC0,0x30,0x18,0x88,0x00,0x00,0x00,0x03,0x02,0x02,0x02,0x0F,0x00,0x00},/*"4",30*/
		{0x00,0xF8,0x88,0x48,0x48,0xC8,0x80,0x00,0x00,0x04,0x0C,0x08,0x08,0x0C,0x07,0x00},/*"5",31*/
		{0x00,0x80,0xE0,0x50,0x48,0xC0,0x80,0x00,0x00,0x07,0x0C,0x08,0x08,0x0C,0x07,0x00},/*"6",32*/
		{0x00,0x18,0x08,0x08,0x88,0xE8,0x18,0x00,0x00,0x00,0x08,0x0C,0x03,0x00,0x00,0x00},/*"7",33*/
		{0x00,0x30,0xD8,0x88,0x88,0x58,0x30,0x00,0x00,0x07,0x08,0x08,0x08,0x0D,0x07,0x00},/*"8",34*/
		{0x00,0xF0,0x98,0x08,0x08,0x98,0xF0,0x00,0x00,0x00,0x01,0x09,0x07,0x01,0x00,0x00},/*"9",35*/
		{0x00,0xF0,0x18,0x88,0x88,0x18,0xF0,0x00,0x00,0x07,0x0C,0x08,0x08,0x0C,0x07,0x00},/*"0",36*/
};

// 0b[left pad 0 for 7bit]1[dida]1
uint8_t didaMap[] = {
		0b0001011,			/* A, ._ */
		0b0110001, 		/* B, _... */
		0b0110101, 		/* C, _._. */
		0b0011001,		/* D, _.. */
		0b0000101,		/* E, . */
		0b0100101,		/* F, .._. */
		0b0011101,		/* G, __. */
		0b0100001,		/* H, .... */
		0b0001001,		/* I, .. */
		0b0101111,		/* J, .___ */
		0b0011011,		/* K, _._ */
		0b0101001,		/* L, ._.. */
		0b0001111,		/* M, __ */
		0b0001101,		/* N, _. */
		0b0011111,		/* O, ___ */
		0b0101101,		/* P, .__. */
		0b0111011,		/* Q, __._ */
		0b0010101,		/* R, ._. */
		0b0010001,		/* S, ... */
		0b0000111,		/* T, _ */
		0b0010011,		/* U, .._ */
		0b0100011,		/* V, ..._ */
		0b0010111,		/* W, .__ */
		0b0110011,		/* X, _.._ */
		0b0110111,		/* Y, _.__ */
		0b0111001,		/* Z, __.. */
		0b0000000,		/* ?, */
		0b1011111,		/* 1, .____ */
		0b1001111,		/* 2, ..___ */
		0b1000111,		/* 3, ...__ */
		0b1000011,		/* 4, ...._ */
		0b1000001,		/* 5, ..... */
		0b1100001,		/* 6, _.... */
		0b1110001,		/* 7, __... */
		0b1111001,		/* 8, ___.. */
		0b1111101,		/* 9, ____. */
		0b1111111,		/* 0, _____ */
};

//
uint8_t cwLineBuff[128];

//
uint8_t decodeLineHisBuff[16];

//
uint8_t decodeLineCurrentPos = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartTaskKey(void *argument);
void StartTaskLed(void *argument);

/* USER CODE BEGIN PFP */

void oledShowDemo(void);
void oledShowCWLine(uint8_t action);
void oledClear(void);
void oledShowCWDecode(uint8_t didaBuff);
void oledClearCWDecode(void);
void oledShowHisCWDecode(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void oledShowDemo (void)
{
	// posD: page=b0-b7, col-lower=0x00-0x0f, col-upper=0x10-0x1f
  uint8_t posD[] = { 0xb6, 0x00, 0x10 };
  ssd1306_show (posD, 3, &fontMap[0][0], 8);
  posD[0] = 0xb7;	// page1
  ssd1306_show (posD, 3, &fontMap[0][8], 8);

  posD[0] = 0xb6;
  posD[1] = 0x09;
  ssd1306_show (posD, 3, &fontMap[1][0], 8);
  posD[0] = 0xb7;
  posD[1] = 0x09;
  ssd1306_show (posD, 3, &fontMap[1][8], 8);
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
		for(uint8_t i = 0; i < 128-4; i++) {
			cwLineBuff[i] = cwLineBuff[i+4];
		}
		cwLineBuff[124] = CW_LINE_BLANK;
		cwLineBuff[125] = CW_LINE_BLANK;
		cwLineBuff[126] = CW_LINE_BLANK;
		cwLineBuff[127] = CW_LINE_BLANK;
	} else {
		// out of action
	}

	// 0xb3 is cwline page.
	uint8_t posD[] = { 0xb3, 0x00, 0x10 };
	ssd1306_show (posD, 3, cwLineBuff, 128);
}

void oledClear(void) {
	uint8_t oledBuff[128];
	for(uint8_t i = 0; i < 128; i++) {
				oledBuff[i] = 0x00;
	}
	for(uint8_t i = 0; i < 8; i++) {
		uint8_t posD[] = {0xb0 + i, 0x00, 0x10 };
		ssd1306_show (posD, 3, oledBuff, 128);
	}

}


void oledShowCWDecode(uint8_t didaBuff) {
  //TODO return line
	if(decodeLineCurrentPos > 15) {
		decodeLineCurrentPos = 0;
		oledClearCWDecode();
		oledShowHisCWDecode();
	}

	// 26 is "?"
	uint8_t didaDecodeFontIndex = 26;

	// set 1 for pad right
	didaBuff = didaBuff << 1 | 0x01;

	// decode didaBuff
	for(uint8_t i = 0;i < sizeof(didaMap); i++) {
		if(didaMap[i] == didaBuff) {
			didaDecodeFontIndex = i;
		}
	}

	// get curPos
	uint8_t curPos, curPosH, curPosL;
	curPos = decodeLineCurrentPos * 8;
	curPosH = (curPos & 0b11110000) >> 4;
	curPosL = curPos & 0b1111;

	// show decode letter
	uint8_t posD[3];
	posD[0] = 0xb6;
	posD[1] = 0x00 + curPosL;
	posD[2] = 0x10 + curPosH;
  ssd1306_show (posD, 3, &fontMap[didaDecodeFontIndex][0], 8);
  posD[0] = 0xb7;
  ssd1306_show (posD, 3, &fontMap[didaDecodeFontIndex][8], 8);

  // set history letter
  decodeLineHisBuff[decodeLineCurrentPos] = didaDecodeFontIndex;

  decodeLineCurrentPos++;
}

void oledClearCWDecode(void) {
	uint8_t cwDecodeBuff[128];
	for(uint8_t i = 0; i < 128; i++) {
		cwDecodeBuff[i] = 0x00;
	}
	uint8_t posD[] = {0xb6, 0x00, 0x10 };
	ssd1306_show(posD, 3, &cwDecodeBuff[0], 128);
	posD[0] = 0xb7;
	ssd1306_show(posD, 3, &cwDecodeBuff[0], 128);
}

void oledShowHisCWDecode(void) {
	uint8_t posD[3];
	for(uint8_t i = 0; i < 16; i++) {
		posD[0] = 0xb4;
		posD[1] = 0x00 + ((i * 8) & 0b1111);
		posD[2] = 0x10 + (((i * 8) & 0b11110000) >> 4);
		ssd1306_show(posD, 3, &fontMap[decodeLineHisBuff[i]][0], 8);
		posD[0] = 0xb5;
		ssd1306_show(posD, 3, &fontMap[decodeLineHisBuff[i]][8], 8);
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
  MX_TIM2_Init();
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

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
	uint8_t checkDiDaDelay = 0;
	uint8_t didaBuff = 0b1;	// init 1 for pad left

	ssd1306_init(&hi2c1);

	oledClear();

//	oledShowDemo();

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
			if(cwLineShowDelay > CW_LINE_SHOW_DELAY_MAX) {
				oledShowCWLine(CW_LINE_ACTION_DOT);
				checkDiDaDelay++;
				cwLineShowDelay = 0;
			}
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		} else  if(cwLineStatus == SHOW_STATUS_UP) {
			cwLineShowDelay++;
			if(cwLineShowDelay > CW_LINE_SHOW_DELAY_MAX) {
				oledShowCWLine(CW_LINE_ACTION_BLANK);
				if(checkDiDaDelay > 0) {
					if(checkDiDaDelay > CW_LINE_CHECK_DIDA_DELAY_MAX) {
						// is da _, set didaBuff 1
						didaBuff = didaBuff << 1 | 0x01;
					} else {
						// is di .,set didaBuff 0
						didaBuff = didaBuff << 1;
					}
					checkDiDaDelay = 0;
				}
				cwLineShowDelay = 0;
			}
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		} else if(cwLineStatus == SHOW_STATUS_DECODE) {
			oledShowCWLine(CW_LINE_ACTION_DECODE);
			oledShowCWDecode(didaBuff);
			didaBuff = 0b1;
			cwLineStatus = SHOW_STATUS_IDLE;
		} else { // cw_line is IDLE
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
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
  			if(deboundCount > DEBOUND_DELAY_MAX) {
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
  			if(decodeWaitCount > DECODE_DELAY_MAX) {
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
