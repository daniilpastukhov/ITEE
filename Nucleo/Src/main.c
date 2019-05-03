/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"


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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId motor0TaskHandle;
osThreadId motor1TaskHandle;
osThreadId ADCTaskHandle;
osThreadId controlTaskHandle;
osThreadId backupTask0Handle;
osThreadId backupTask1Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void motor0Loop(void const * argument);
void motor1Loop(void const * argument);
void ADCLoop(void const * argument);
void controlLoop(void const * argument);
void backupLoop0(void const * argument);
void backupLoop1(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motor0Task */
  osThreadDef(motor0Task, motor0Loop, osPriorityIdle, 0, 128);
  motor0TaskHandle = osThreadCreate(osThread(motor0Task), NULL);

  /* definition and creation of motor1Task */
  osThreadDef(motor1Task, motor1Loop, osPriorityIdle, 0, 128);
  motor1TaskHandle = osThreadCreate(osThread(motor1Task), NULL);

  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, ADCLoop, osPriorityIdle, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, controlLoop, osPriorityIdle, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of backupTask0 */
  osThreadDef(backupTask0, backupLoop0, osPriorityIdle, 0, 128);
  backupTask0Handle = osThreadCreate(osThread(backupTask0), NULL);

  /* definition and creation of backupTask1 */
  osThreadDef(backupTask1, backupLoop1, osPriorityIdle, 0, 128);
  backupTask1Handle = osThreadCreate(osThread(backupTask1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|AIN1_Pin|AIN2_Pin|PWMB_Pin 
                          |BN1_Pin|BN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, backupOut1_Pin|backupOut0_Pin|PWMA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : backupIn1_Pin */
  GPIO_InitStruct.Pin = backupIn1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(backupIn1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : backupIn0_Pin */
  GPIO_InitStruct.Pin = backupIn0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(backupIn0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : backupOut1_Pin backupOut0_Pin PWMA_Pin */
  GPIO_InitStruct.Pin = backupOut1_Pin|backupOut0_Pin|PWMA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin PWMB_Pin BN1_Pin 
                           BN2_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|PWMB_Pin|BN1_Pin 
                          |BN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

#ifndef __MOTOR_PINS
#define __MOTOR_PINS
#define AIN1_PORT	GPIOA
#define AIN1_PIN	GPIO_PIN_8

#define AIN2_PORT	GPIOA
#define AIN2_PIN	GPIO_PIN_9

#define PWMA_PORT	GPIOC
#define PWMA_PIN	GPIO_PIN_9

#define BIN1_PORT	GPIOA
#define BIN1_PIN	GPIO_PIN_11

#define BIN2_PORT	GPIOA
#define BIN2_PIN	GPIO_PIN_12

#define PWMB_PORT	GPIOA
#define PWMB_PIN	GPIO_PIN_10

#define ERROR_LED_PORT	GPIOA
#define ERROR_LED_PIN	GPIO_PIN_5

#define MAX_SPEED 100
#endif

extern enum State{On,Off};
extern enum Dir{Forw,Backw};

extern enum State m1State = On;
extern enum State m2State = On;

extern enum Dir m1Dir = Backw;
extern enum Dir m2Dir = Backw;

extern int16_t m1Speed = 50;
int16_t lastM1Speed = 0;

extern int16_t m2Speed = 50;
int16_t lastM2Speed = 0;

extern bool errorMot;


extern void setCorrectDirM1(void);
extern void resetDirM1(void);
extern void setCorrectDirM2(void);
extern void resetDirM2(void);
extern void handleSpeedAndDirM1(void);
extern void handleSpeedAndDirM2(void);

void modifyTimer0PWM(uint16_t value){
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void modifyTimer1PWM(uint16_t value){
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}



/* USER CODE BEGIN Header_motor0Loop */
/**
* @brief Function implementing the motor0Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor0Loop */
void motor0Loop(void const * argument)
{

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
  /* USER CODE BEGIN motor0Loop */
  /* Infinite loop */
  for(;;)
  {
	  if(m1State == On){

		  osDelay(1);

		  if (m1Speed != lastM1Speed){

			  modifyTimer0PWM(m1Speed * 655);

			  lastM1Speed = m1Speed;

		  	  }

		  osDelay(1);
		  setCorrectDirM1();
		  osDelay(1);
	  }
  }
  /* USER CODE END motor0Loop */
}

/* USER CODE BEGIN Header_motor1Loop */
/**
* @brief Function implementing the motor1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor1Loop */
void motor1Loop(void const * argument)
{
  /* USER CODE BEGIN motor1Loop */
  /* Infinite loop */

  for(;;)
  {
	  if(m2State == On){

		  osDelay(1);
		  if (m2Speed != lastM2Speed){
			  modifyTimer1PWM(m2Speed * 655);
			  lastM2Speed = m2Speed;
		  	  }
		   osDelay(1);
		   setCorrectDirM2();
			osDelay(1);
	  	  }
  	  }

//
//
//		  if(m2Speed>MAX_SPEED || m2Speed<0){
//			  HAL_GPIO_WritePin(ERROR_LED_PORT,ERROR_LED_PIN,GPIO_PIN_SET);
//			  errorMot = true;
//		  }
//		  else{
//			  HAL_GPIO_WritePin(PWMB_PORT,PWMB_PIN,GPIO_PIN_SET);
//			  setCorrectDirM2();
//			  osDelay(m2Speed);
//			  resetDirM2();
//			  HAL_GPIO_WritePin(PWMB_PORT,PWMB_PIN,GPIO_PIN_RESET);
//			  osDelay(MAX_SPEED-m2Speed);
//		  }
  /* USER CODE END motor1Loop */
}

/* USER CODE BEGIN Header_ADCLoop */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCLoop */
void ADCLoop(void const * argument)
{
	volatile uint16_t adcl1 = adc1, adcl2 = adc2, adcl3 = adc3, adcl4 = adc4, adcl5 = adc5;

	while (1)
	{

	config(1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100000);
	adcl5 = HAL_ADC_GetValue(&hadc1);
	adcl5 = adcl5 / 4;

	config(6);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100000);
	adcl2 = HAL_ADC_GetValue(&hadc1);
	adcl2 = adcl2 / 4;

	config(7);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100000);
	adcl1 = HAL_ADC_GetValue(&hadc1);
	adcl1 = adcl1 / 4;

	config(8);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100000);
	adcl4 = HAL_ADC_GetValue(&hadc1);
	adcl4 = adcl4 / 4;

	config(9);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100000);
	adcl3 = HAL_ADC_GetValue(&hadc1);
	adcl3 = adcl3 / 4;

	osDelay(2);

	adc1 = adcl1;
	adc2 = adcl2;
	adc3 = adcl3;
	adc4 = adcl4;
	adc5 = adcl5;

	osDelay(10);
	}
}

void config(int a) {
ADC_ChannelConfTypeDef sTmp = {0};
switch(a) {
case 1:
sTmp.Channel = ADC_CHANNEL_1;
break;
case 6:
sTmp.Channel = ADC_CHANNEL_6;
break;
case 7:
sTmp.Channel = ADC_CHANNEL_7;
break;
case 8:
sTmp.Channel = ADC_CHANNEL_8;
break;
case 9:
sTmp.Channel = ADC_CHANNEL_9;
break;
default:
sTmp.Channel = ADC_CHANNEL_1;
}
sTmp.Rank = ADC_REGULAR_RANK_1;
sTmp.SingleDiff = ADC_SINGLE_ENDED;
sTmp.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
sTmp.OffsetNumber = ADC_OFFSET_NONE;
sTmp.Offset = 0;
if (HAL_ADC_ConfigChannel(&hadc1, &sTmp) != HAL_OK)
{
Error_Handler();
}
}

#define CONTROL_HYST			100
#define STOPPER 			   900
/* USER CODE BEGIN Header_controlLoop */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlLoop */
void controlLoop(void const * argument)
{

		m1State = On;
		m2State = On;
		m1Speed = 0;
		m2Speed = 0;

	while(1){

		osDelay(1);
	}
//	  /* USER CODE BEGIN controlLoop */
//		/* Infinite loop */
//
//
//		for(;;)
//		{
//		int middle = (0 * adc1 + 1000 * adc2 + 2000 * adc3 + 3000 * adc4 + 4000 * adc5) / (adc1 + adc2 + adc3 + adc4 + adc5);
//
//		int data= (adc1 + adc2 + adc3 + adc4 + adc5) / 5;
//
//
//		if ((adc1 + adc2 + adc3 + adc4 + adc5) / 5 >= STOPPER) {
//			m1State = Off;
//			m2State = Off;
//
//		}
//		else{
//			m1State = On;
//			m2State = On;
//
//			if (middle + CONTROL_HYST > 2000) {
//				turnLeft();
//
//			}
//				else
//					if (middle - CONTROL_HYST < 2000) {
//						turnRight();
//
//						}
//						else {
//
//							goAhead();
//							}
//			osDelay(50);
//			}
//		}
//	  /* USER CODE END controlLoop */
}

void changeSpeed(int leftSpeed, int rightSpeed) {
if (leftSpeed > m1Speed) m1Speed++;
else if (leftSpeed < m1Speed) m1Speed--;
if (rightSpeed > m2Speed) m2Speed++;
else if (rightSpeed < m2Speed) m2Speed--;
}

void goAhead() {
changeSpeed(5,5);
}

void turnLeft() {
changeSpeed(3, 5);
}

void turnRight() {
changeSpeed(5, 3);
}
/* USER CODE BEGIN Header_backupLoop0 */
/**
* @brief Function implementing the backupTask0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_backupLoop0 */
void backupLoop0(void const * argument)
{
  /* USER CODE BEGIN backupLoop0 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END backupLoop0 */
}

/* USER CODE BEGIN Header_backupLoop1 */
/**
* @brief Function implementing the backupTask1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_backupLoop1 */
void backupLoop1(void const * argument)
{
  /* USER CODE BEGIN backupLoop1 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END backupLoop1 */
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
