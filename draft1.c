/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APB2_BUS_FREQUENCY 8000000 //8MHz
#define ECONOMIZER_PWM_FREQUENCY 20000 //20kHz
#define ECONOMIZER_PWM_PERIOD (APB2_BUS_FREQUENCY / ECONOMIZER_PWM_FREQUENCY) //400
#define ECONOMIZER_PWM_DUTY_CYCLE_50 ((ECONOMIZER_PWM_PERIOD * 50) / 100) //200

#define VOLTAGE_SENSING_RESISTOR_R1 10000.0f //10k ohm
#define VOLTAGE_SENSING_RESISTOR_R2 1000.0f //1k ohm
#define DC_DC_OP_VOLTAGE_THRESHOLD 11.0f //12V

#define MPPT_STATE_ALL_ON 0
#define MPPT_STATE_TWO_ON 1
#define MPPT_STATE_ALL_OFF 2

#define MPPT_PORT GPIOB
#define MPPT_PRECHARGE_DELAY 2000
#define MPPT_MOSFET_THRESHOLD_TEMPERATURE 60.0f //60 degrees Celsius
#define MPPT_TEMPERATURE_HYSTERESIS 5.0f //5 degrees Celsius

#define BATTERY_SOC_THRESHOLD_HIGH 99.0f
#define BATTERY_SOC_THRESHOLD_MID 98.0f
#define BATTERY_SOC_THRESHOLD_LOW 97.0f
#define BATTERY_SOC_HYSTERESIS 0.5f //1 percent

#define CONTACTOR_PULLIN_TIME 10  // 10 * 10ms = 100ms
#define CONTACTOR_STATE_OFF 0
#define CONTACTOR_STATE_PULLIN 1
#define CONTACTOR_STATE_ECONOMIZER 2

#define CONTACTOR_MPPT1_IN 0
#define CONTACTOR_MPPT2_IN 1
#define CONTACTOR_MPPT3_IN 2
#define CONTACTOR_MPPT_OUT 3

#define INDI_PORT GPIOD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

// Contactor economizer states
typedef struct {
  uint8_t state;
  uint8_t counter;
} ContactorState;

ContactorState contactorStates[4] = {
  {CONTACTOR_STATE_OFF, 0},
  {CONTACTOR_STATE_OFF, 0},
  {CONTACTOR_STATE_OFF, 0},
  {CONTACTOR_STATE_OFF, 0}
};

//BMS variable

//Indicator Variables

volatile uint8_t blinking_enabled = 0;
volatile uint8_t blink_counter;


volatile uint8_t BMS_MODE = 0;
volatile float BATTERY_SOC_PERCENT = 0.0f;

volatile uint16_t DC_DC_OP_ADC = 0;
volatile float DC_DC_OP_VOLTAGE = 0.0f;
volatile uint16_t adc_buffer[5];


volatile float MPPT1_MOSFET_TEMPERATURE = 0.0f;
volatile float MPPT2_MOSFET_TEMPERATURE = 0.0f;
volatile float MPPT3_MOSFET_TEMPERATURE = 0.0f;
volatile uint8_t SOC_CONTROL_STATE = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void CAN_FilterConfig(void);
void HAL_Fifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Read_All_ADC_Channels(void);
void MPPT_IN_Start(void);
void MPPT_Precharge(void);
void MPPT_12V_Supply(void);
void MPPT_ALL_ON(void);
void MPPT_ALL_OFF(void);
void MPPT_COOLEST_TWO_ON(void);
void MPPT_SOC_CONTROL(void);
void MPPT_TEMPERATURE_SHUTDOWN(void);

// Contactor control functions
void Contactor_On(uint8_t contactor);
void Contactor_Off(uint8_t contactor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  CAN_FilterConfig();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,10);

  HAL_TIM_Base_Start_IT(&htim4);
  
  // Start PWM for all channels
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  while(BMS_MODE!= 0x05 || DC_DC_OP_VOLTAGE < DC_DC_OP_VOLTAGE_THRESHOLD)
  {
  	 Read_All_ADC_Channels();
  	DC_DC_OP_VOLTAGE = ((VOLTAGE_SENSING_RESISTOR_R1 + VOLTAGE_SENSING_RESISTOR_R2)*(float)DC_DC_OP_ADC * 3.3) / (4095*VOLTAGE_SENSING_RESISTOR_R2);
  }

  MPPT_IN_Start();
  MPPT_Precharge();
  HAL_GPIO_WritePin(MPPT_PORT, MPPT_12V_SUPPLY, GPIO_PIN_SET);


  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
  AnalogWDGConfig.HighThreshold = 63;
  AnalogWDGConfig.LowThreshold = 32;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 39 ;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, INDI_RIGHT_OUT_PIN_Pin|INDI_LEFT_OUT_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MPPT_PRECHARGE_PIN_Pin|MPPT_12V_SUPPLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JUNCTION_BOX_FAN_GPIO_Port, JUNCTION_BOX_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INDI_RIGHT_OUT_PIN_Pin INDI_LEFT_OUT_PIN_Pin */
  GPIO_InitStruct.Pin = INDI_RIGHT_OUT_PIN_Pin|INDI_LEFT_OUT_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INDI_LEFT_IN_PIN_Pin INDI_RIGHT_IN_PIN_Pin INDI_HAZARD_PIN_Pin */
  GPIO_InitStruct.Pin = INDI_LEFT_IN_PIN_Pin|INDI_RIGHT_IN_PIN_Pin|INDI_HAZARD_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MPPT_PRECHARGE_PIN_Pin MPPT_12V_SUPPLY_Pin */
  GPIO_InitStruct.Pin = MPPT_PRECHARGE_PIN_Pin|MPPT_12V_SUPPLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : JUNCTION_BOX_FAN_Pin */
  GPIO_InitStruct.Pin = JUNCTION_BOX_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JUNCTION_BOX_FAN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CAN_FilterConfig(void)
{
	CAN_FilterTypeDef BMS_Filter;

	BMS_Filter.FilterActivation = ENABLE;
	BMS_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	BMS_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	BMS_Filter.FilterIdHigh = 0x600 << 5; // 0x600
	BMS_Filter.FilterIdLow = 0x0000;
	BMS_Filter.FilterMaskIdHigh = 0x700 << 5; // 0x7FF
	BMS_Filter.FilterMaskIdLow = 0x0000;
	BMS_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	BMS_Filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &BMS_Filter);


	CAN_FilterTypeDef MPPT_Filter;
	MPPT_Filter.FilterActivation = ENABLE;
	MPPT_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	MPPT_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	MPPT_Filter.FilterIdHigh = 0x690 << 5;
	MPPT_Filter.FilterIdLow = 0x0000;
	MPPT_Filter.FilterMaskIdHigh = 0x700 << 5;
	MPPT_Filter.FilterMaskIdLow = 0x0000;
	MPPT_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	MPPT_Filter.FilterBank = 1;
	HAL_CAN_ConfigFilter(&hcan1, &MPPT_Filter);
}


void HAL_Fifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
			}

	switch(RxHeader.StdId)
	{
	case 0x6F7:
		memcpy(&BMS_MODE, &RxData[1], sizeof(BMS_MODE));
		break;

	case 0x6F4:
		memcpy(&BATTERY_SOC_PERCENT, &RxData[4], sizeof(Battery_SOC_Percent));
		break;

	case 0x692:
		memcpy(&MPPT1_MOSFET_TEMPERATURE, &RxData[0], sizeof(MPPT1_MOSFET_TEMPERATURE));
		break;

	case 0x6A2:
		memcpy(&MPPT2_MOSFET_TEMPERATURE, &RxData[0], sizeof(MPPT2_MOSFET_TEMPERATURE));
		break;

	case 0x6B2:
		memcpy(&MPPT3_MOSFET_TEMPERATURE, &RxData[0], sizeof(MPPT3_MOSFET_TEMPERATURE));
		break;
	}
}


void Read_All_ADC_Channels(void)
{
	HAL_ADC_Start(&hadc1);
	for(int i= 0; i < 5; i++)
	{
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
		DC_DC_OP_ADC = adc_buffer[0];
	}
	HAL_ADC_Stop(&hadc1);
}

// Contactor control functions
void Contactor_On(uint8_t contactor)
{
  if (contactor >= 4) return;
  
  // Set PWM to 100% (pulse = period+1)
  switch(contactor) {
    case CONTACTOR_MPPT1_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 10);
      break;
    case CONTACTOR_MPPT2_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 10);
      break;
    case CONTACTOR_MPPT3_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 10);
      break;
    case CONTACTOR_MPPT_OUT:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10);
      break;
  }
  
  contactorStates[contactor].state = CONTACTOR_STATE_PULLIN;
  contactorStates[contactor].counter = 0;
}

void Contactor_Off(uint8_t contactor)
{
  if (contactor >= 4) return;
  
  // Set PWM to 0%
  switch(contactor) {
    case CONTACTOR_MPPT1_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      break;
    case CONTACTOR_MPPT2_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
      break;
    case CONTACTOR_MPPT3_IN:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      break;
    case CONTACTOR_MPPT_OUT:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      break;
  }
  
  contactorStates[contactor].state = CONTACTOR_STATE_OFF;
}

// MPPT functions
void MPPT_IN_Start(void)
{
	Contactor_On(CONTACTOR_MPPT1_IN);
	Contactor_On(CONTACTOR_MPPT2_IN);
	Contactor_On(CONTACTOR_MPPT3_IN);
}

void MPPT_Precharge(void)
{
	HAL_GPIO_WritePin(MPPT_PORT, MPPT_PRECHARGE_PIN, GPIO_PIN_SET);
	HAL_Delay(MPPT_PRECHARGE_DELAY);
	Contactor_On(CONTACTOR_MPPT_OUT);
	HAL_GPIO_WritePin(MPPT_PORT, MPPT_PRECHARGE_PIN, GPIO_PIN_RESET);
}

void MPPT_ALL_ON(void)
{
	Contactor_On(CONTACTOR_MPPT1_IN);
	Contactor_On(CONTACTOR_MPPT2_IN);
	Contactor_On(CONTACTOR_MPPT3_IN);
}

void MPPT_ALL_OFF(void)
{
	Contactor_Off(CONTACTOR_MPPT1_IN);
	Contactor_Off(CONTACTOR_MPPT2_IN);
	Contactor_Off(CONTACTOR_MPPT3_IN);
}

void MPPT_COOLEST_TWO_ON(void)
{
	int mppt_on_count = 0;
	if(contactorStates[CONTACTOR_MPPT1_IN].state != CONTACTOR_STATE_OFF)
	{
		mppt_on_count++;
	}
	if(contactorStates[CONTACTOR_MPPT2_IN].state != CONTACTOR_STATE_OFF)
	{
		mppt_on_count++;
	}
	if(contactorStates[CONTACTOR_MPPT3_IN].state != CONTACTOR_STATE_OFF)
	{
		mppt_on_count++;
	}

	struct{
		uint8_t contactor;
		float temperature;

	}mppts[3] = {
		{CONTACTOR_MPPT1_IN, MPPT1_MOSFET_TEMPERATURE},
		{CONTACTOR_MPPT2_IN, MPPT2_MOSFET_TEMPERATURE},
		{CONTACTOR_MPPT3_IN, MPPT3_MOSFET_TEMPERATURE}
	};

	for(int i = 0; i < 2; i++){
		for(int j = i+1; j < 3; j++){
			if(mppts[i].temperature > mppts[j].temperature)
			{
			const uint8_t temporaryContactor = mppts[i].contactor;
			const float temporaryTemperature = mppts[i].temperature;
			mppts[i].contactor = mppts[j].contactor;
			mppts[i].temperature = mppts[j].temperature;
			mppts[j].contactor = temporaryContactor;
			mppts[j].temperature = temporaryTemperature;
			}
		}
	}
	switch(mppt_on_count)
	{

	case 3:
		int enabledCount  = 0;
		for(int i = 0; i < 3 && enabledCount < 2; i++)
		{
			if(mppts[i].temperature < MPPT_MOSFET_THRESHOLD_TEMPERATURE - MPPT_TEMPERATURE_HYSTERESIS)
			{
				Contactor_On(mppts[i].contactor);
				enabledCount++;
			}
		}
		break;

	case 2:
		if(mppts[2].temperature - mppts[1].temperature > MPPT_TEMPERATURE_HYSTERESIS){
			int enabledCount = 0;
		for(int i = 0; i < 3 && enabledCount < 2; i++)
		{
			if(mppts[i].temperature < MPPT_MOSFET_THRESHOLD_TEMPERATURE - MPPT_TEMPERATURE_HYSTERESIS)
			{
				Contactor_On(mppts[i].contactor);
				enabledCount++;
			}
		}

		}
		break;
	}
}

void MPPT_SOC_CONTROL(void){
	if(BATTERY_SOC_PERCENT > BATTERY_SOC_THRESHOLD_HIGH){
		SOC_CONTROL_STATE = MPPT_STATE_ALL_OFF;
	}
	else if(BATTERY_SOC_PERCENT < BATTERY_SOC_THRESHOLD_MID - BATTERY_SOC_HYSTERESIS
			&& SOC_CONTROL_STATE == MPPT_STATE_ALL_OFF){
		SOC_CONTROL_STATE = MPPT_STATE_TWO_ON;
	}
	else if(BATTERY_SOC_PERCENT < BATTERY_SOC_THRESHOLD_LOW - BATTERY_SOC_HYSTERESIS
			&& SOC_CONTROL_STATE == MPPT_STATE_TWO_ON){
		SOC_CONTROL_STATE = MPPT_STATE_ALL_ON;
	}
	else if(BATTERY_SOC_PERCENT > BATTERY_SOC_THRESHOLD_MID + BATTERY_SOC_HYSTERESIS
			&& SOC_CONTROL_STATE == MPPT_STATE_TWO_ON){
		SOC_CONTROL_STATE = MPPT_STATE_ALL_OFF;
	}

	switch(SOC_CONTROL_STATE)
	{
		case MPPT_STATE_ALL_ON:
			MPPT_ALL_ON();
			break;

		case MPPT_STATE_TWO_ON:
			MPPT_COOLEST_TWO_ON();
			break;

		case MPPT_STATE_ALL_OFF:
			MPPT_ALL_OFF();
			break;

		default:
			break;
	}
}

void MPPT_TEMPERATURE_SHUTDOWN(void)
{
	if(MPPT1_MOSFET_TEMPERATURE > MPPT_MOSFET_THRESHOLD_TEMPERATURE + MPPT_TEMPERATURE_HYSTERESIS){
		Contactor_Off(CONTACTOR_MPPT1_IN);
	}
	if(MPPT2_MOSFET_TEMPERATURE > MPPT_MOSFET_THRESHOLD_TEMPERATURE + MPPT_TEMPERATURE_HYSTERESIS){
		Contactor_Off(CONTACTOR_MPPT2_IN);
	}
	if(MPPT3_MOSFET_TEMPERATURE > MPPT_MOSFET_THRESHOLD_TEMPERATURE + MPPT_TEMPERATURE_HYSTERESIS){
		Contactor_Off(CONTACTOR_MPPT3_IN);
	}
}


// INDICATOR code in interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INDI_HAZARD_PIN || GPIO_Pin == INDI_LEFT_IN_PIN || GPIO_Pin == INDI_RIGHT_IN_PIN)
	{
		if(HAL_GPIO_ReadPin(INDI_PORT, INDI_HAZARD_PIN) == GPIO_PIN_SET ||
			HAL_GPIO_ReadPin(INDI_PORT, INDI_LEFT_IN_PIN) == GPIO_PIN_SET ||
			HAL_GPIO_ReadPin(INDI_PORT, INDI_RIGHT_IN_PIN) == GPIO_PIN_SET )
		{
			blinking_enabled = 1;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
    // Contactor economizer state machine
    for (int i = 0; i < 4; i++) {
      if (contactorStates[i].state == CONTACTOR_STATE_PULLIN) {
        contactorStates[i].counter++;
        if (contactorStates[i].counter >= CONTACTOR_PULLIN_TIME) {
          // Switch to economizer mode (50% duty)
          switch(i) {
            case CONTACTOR_MPPT1_IN:
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 5); // 50% duty
              break;
            case CONTACTOR_MPPT2_IN:
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 5); // 50% duty
              break;
            case CONTACTOR_MPPT3_IN:
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 5); // 50% duty
              break;
            case CONTACTOR_MPPT_OUT:
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 5); // 50% duty
              break;
          }
          contactorStates[i].state = CONTACTOR_STATE_ECONOMIZER;
        }
      }
    }
    
    // Existing indicator code
		if(blinking_enabled)
		{
			if(HAL_GPIO_ReadPin(INDI_PORT, INDI_HAZARD_PIN) == GPIO_PIN_SET)
			{
				blink_counter++;
				if(blink_counter >= 10000) // 500 ms at 20 KHz
				{
					HAL_GPIO_TogglePin(INDI_PORT, INDI_LEFT_OUT_PIN);
					GPIO_PinState left_state = HAL_GPIO_ReadPin(INDI_PORT, INDI_LEFT_OUT_PIN);
					HAL_GPIO_WritePin(INDI_PORT, INDI_RIGHT_OUT_PIN, left_state);
					blink_counter = 0;
				}
			}
			else if(HAL_GPIO_ReadPin(INDI_PORT, INDI_LEFT_IN_PIN) == GPIO_PIN_SET)
			{
				blink_counter++;
				if(blink_counter >= 10000) // 500 ms at 20 KHz
				{
					HAL_GPIO_TogglePin(INDI_PORT, INDI_LEFT_OUT_PIN);
					HAL_GPIO_WritePin(INDI_PORT, INDI_RIGHT_OUT_PIN, GPIO_PIN_RESET);
					blink_counter = 0;
				}

			}
			else if(HAL_GPIO_ReadPin(INDI_PORT, INDI_RIGHT_IN_PIN) == GPIO_PIN_SET)
			{
				blink_counter++;
				if(blink_counter >= 10000) // 500 ms at 20 KHz
				{
					HAL_GPIO_TogglePin(INDI_PORT, INDI_RIGHT_OUT_PIN);
					HAL_GPIO_WritePin(INDI_PORT, INDI_LEFT_OUT_PIN, GPIO_PIN_RESET);
					blink_counter = 0;
				}
			}
			else
			{
				blinking_enabled = 0;
				blink_counter = 0;
				HAL_GPIO_WritePin(INDI_PORT, INDI_LEFT_OUT_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(INDI_PORT, INDI_RIGHT_OUT_PIN, GPIO_PIN_RESET);
			}
		}
	}
}
/* USER CODE END 4 */

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
