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
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================================================
// 1. DEFINITIONS DES MASQUES (ADDRESSAGE LOGIQUE)
// ============================================================
// Ces définitions permettent de donner un nom aux bits de votre variable d'état
// on décale les bits de 0,1,2,3 rangs vers la gauche
#define W_AVD (1 << 0) // Bit 0
#define W_AVG (1 << 1) // Bit 1
#define W_ARD (1 << 2) // Bit 2
#define W_ARG (1 << 3) // Bit 3

// --- Capteurs Distance (4 bits) ---
#define DIST_AVC ( 1 << 0) // Avant Centre
#define DIST_AVG ( 1 << 1) // Avant Gauche
#define DIST_AVD ( 1 << 2) // Avant Droit
#define DIST_ARC ( 1 << 3) // Arrière Centre

// ============================================================
// 2. CONFIGURATION DES CANAUX ADC
// ============================================================
// Remplacez ADC_CHANNEL_X par le canal correspondant à vos pins
// Exemple: PA0 -> Channel 5, PA1 -> Channel 6, etc.
#define CH_DIST_AVG ADC_CHANNEL_10
#define CH_DIST_AVC ADC_CHANNEL_11
#define CH_DIST_AVD ADC_CHANNEL_9
#define CH_DIST_ARC ADC_CHANNEL_8





// Seuil de détection (0 à 255 car ADC 8 bits)
// 3.3V = 255.
// Si le capteur sort 1V à 30cm => (1/3.3)*255 = ~77
// Ajustez cette valeur expérimentalement (afficher la valeur avec printf)
#define SEUIL_DISTANCE 120 	   // plus le seuil est petit et plus on capte loin //max 167 et c'est la valeur quand le robot est au plus proche
#define Vitesse_roue 100      // 40 valeur min robot se déplace //100
#define DUREE_L 100
#define DUREE_D 1000



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Variable globale d'état (volatile car modifiée par le hardware/boucle)
volatile uint8_t etat_capteurs_White = 0;
volatile uint8_t etat_capteur_Dist = 0;
static uint8_t memoire_double_frontal = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

// Prototypes des fonctions moteurs (A remplir avec fonctions PWM)
//void Moteur_Avancer(void);
//void Moteur_Reculer_Tourner_Gauche(void);
//void Moteur_Reculer_Tourner_Droite(void);
//void Moteur_Stop(void);

GPIO_PinState SIGNAL_Read(void);
void Attendre_Signal_Start(void);
void Strategie_white_sensor(void);
void Strategie_Dist_sensors(void);


void MOTEUR_Init(void);
void Moteur_STOP(void);
void Moteur_AVANCER(void );
void Moteur_RECULER(void);
void Moteur_PIVO_Droite(void);
void Moteur_PIVO_Gauche(void);
void Moteur_TOURNER_Droite(void);
void Moteur_TOURNER_Gauche(void);

void Set_MoteurD_Speed(uint8_t, uint8_t);
void Set_MoteurG_Speed(uint8_t, uint8_t);
void Lire_Capteurs_White(void);
void Lire_Capteurs_Dist(void);
uint8_t ADC_Read_Channel(uint32_t channel);
_Bool stop_urgent = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart2;

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    return len;
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  MOTEUR_Init();

  /* Démarrage PWM pour les 2 canaux (gauche/droite) */
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


   /* Attente du micro-start (SIGNAL) */
   //Attendre_Signal_Start();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      // --- LOGIQUE PRINCIPALE ---
	  /*if(SIGNAL_Read()==1){
		  while (stop_urgent==0){
			  Moteur_STOP();
		  }
	  }
	  */
	  //Lire_Capteurs_White();
	  //Lire_Capteurs_Dist();
	  //printf("lecture capteur ligne blanche %d\n\r",etat_capteurs_White);
	  //printf( "lecture capteur Distance %d\n\r",etat_capteur_Dist);
	  //printf("lecture capteur signal %d\n\r", SIGNAL_Read());
	  //printf("lecture source capteur distance arriere centre  %d\n\r", (ADC_Read_Channel(CH_DIST_ARC)));
/*
	  Lire_Capteurs_White();
	  if (etat_capteurs_White !=0 )
		{
			Strategie_white_sensor();
		}
	  else
	  {
		  Strategie_Dist_sensors();
	  }

	  */
	  Strategie_white_sensor();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR2_Pin|DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIGNAL_Pin CAPW_AVG_Pin CAPW_AVD_Pin */
  GPIO_InitStruct.Pin = SIGNAL_Pin|CAPW_AVG_Pin|CAPW_AVD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2_Pin DIR1_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAPW_ARD_Pin CAPW_ARG_Pin */
  GPIO_InitStruct.Pin = CAPW_ARD_Pin|CAPW_ARG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//====================SIGNAL START======================//
GPIO_PinState SIGNAL_Read(void) {
	return HAL_GPIO_ReadPin(SIGNAL_GPIO_Port, SIGNAL_Pin);
}

void Attendre_Signal_Start(void){
	while (SIGNAL_Read() == 0)											// vérifier que signal_read retourne bien 1 quand il ne capte pas, sinon changer la valeur en 0 .
	{
		Moteur_STOP();
	}
	while (SIGNAL_Read()==1)
	{
		Moteur_STOP();
	}
	HAL_Delay(1000); 													//attendre 1s, règle du jeub
}
//=====================SIGNAL START======================//


//=====================SIGNAL STOP======================//


//=====================MOTEUR STOP======================//
void Moteur_STOP(void){

    Set_MoteurD_Speed(0, 0);
    Set_MoteurG_Speed(0, 0);
}
//=====================MOTEUR STOP======================//


//=====================MOTEUR AVANCER======================//
void Moteur_AVANCER(void){

	Set_MoteurD_Speed(Vitesse_roue,1);
	Set_MoteurG_Speed(Vitesse_roue,1);
}
//=====================MOTEUR AVANCER======================//


//=====================MOTEUR RECULER======================//
void Moteur_RECULER(void){

	Set_MoteurD_Speed(Vitesse_roue,0);
	Set_MoteurG_Speed(Vitesse_roue,0);
}
//=====================MOTEUR RECULER======================//


//==================MOTEUR TOURNER D PIVO==================// Rotation 180°
void Moteur_PIVO_Droite(void){
	Set_MoteurD_Speed(Vitesse_roue,0);
	Set_MoteurG_Speed(Vitesse_roue,1);
}
//==================MOTEUR TOURNER D PIVO==================//


//==================MOTEUR TOURNER G PIVO==================// Rotation 180°
void Moteur_PIVO_Gauche(void){
	Set_MoteurD_Speed(Vitesse_roue,1);
	Set_MoteurG_Speed(Vitesse_roue,0);
}
//==================MOTEUR TOURNER G PIVO==================//


//====================MOTEUR TOURNER D=====================// Rotation 90°
void Moteur_TOURNER_Droite(void){
	Set_MoteurD_Speed(0,1);
	Set_MoteurG_Speed(Vitesse_roue,1);
}
//====================MOTEUR TOURNER D=====================//


//====================MOTEUR TOURNER G=====================// Rotation 90°
void Moteur_TOURNER_Gauche(void){
	Set_MoteurD_Speed(Vitesse_roue,1);
	Set_MoteurG_Speed(0,0);
}
//====================MOTEUR TOURNER G=====================//


//=====================MOTEUR INIT======================//

void MOTEUR_Init(void) {															// Initialisation des PWM moteur D et Moteur G

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);										// 1. Démarrer le PWM pour le Moteur 1 (TIM2, Channel 2 -> PA1)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);										// 2. Démarrer le PWM pour le Moteur 2 (TIM1, Channel 2 -> PA9)
																					// TIM1 étant un timer avancé, il faut parfois activer explicitement les sorties principales (MOE bit)

    __HAL_TIM_MOE_ENABLE(&htim1);													// Commande spécifique souvent requise pour TIM1/TIM8 pour activer la sortie physique
}
//=====================MOTEUR INIT======================//


//=====================SET MOTEUR D=====================//
//controler le moteur D
void Set_MoteurD_Speed(uint8_t speed, uint8_t direction) {

																					// 1. Gestion de la direction (PA12)
    if (direction == 0)(HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)); 	// Avancer
    else(HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET));						// Reculer

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);							// 2. Gestion de la vitesse (PWM sur TIM2 CH2)
																					// On écrit directement dans le registre CCR2 (Capture Compare Register)
																					// Comme on a configuré le "Counter Period" à 255, speed (0-255) correspond directement.
}
//=====================SET MOTEUR D=====================//


//=====================SET MOTEUR G=====================//
																					//Controler le moteur G
void Set_MoteurG_Speed(uint8_t speed, uint8_t direction){
																					// 1. Gestion de la direction (PA10)
    if (direction == 0) (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)); 		// Avancer
    else (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET));   				// Reculer

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);							// 2. Gestion de la vitesse (PWM sur TIM1 CH2)
}
//=====================SET MOTEUR G=====================//


//====================LECTURE CAPTEUR WHITE===================//

/*	Cette fonction remplace HAL_GPIO_ReadPin.
	Elle lit directement la mémoire du microcontrôleur.
*/
void Lire_Capteurs_White(void) {

    etat_capteurs_White = 0; // Remise à zéro

    if (!(GPIOA->IDR & GPIO_PIN_11)) (etat_capteurs_White |= W_AVD); 				// On vérifie le bit 11 du port A	// Si PA11 est HIGH, on allume le bit 0
    if (!(GPIOA->IDR & GPIO_PIN_8)) (etat_capteurs_White |= W_AVG);					// On vérifie le bit 8 du Port A 	// Si PA8 est HIGH, on allume le bit 1

    if (!(GPIOB->IDR & GPIO_PIN_4)) (etat_capteurs_White |= W_ARD);					// On vérifie le bit 4 du Port B	// Si PB4 est HIGH, on allume le bit 2
    if (!(GPIOB->IDR & GPIO_PIN_5)) (etat_capteurs_White |= W_ARG);					// On vérifie le bit 5 du Port B 	// Si PB5 est HIGH, on allume le bit 3
}
//====================LECTURE CAPTEUR WHITE===================//


//=======================STRATEGIE=====================//

// CERVEAU DU ROBOT (MACHINE A ÉTATS) capteur ligne blanche
void Strategie_white_sensor(void)
{
	// Variables "static" : elles gardent leur valeur entre chaque appel de la fonction
	// C'est la mémoire du robot.

	static uint32_t temps_fin_virage = 0;
	static uint8_t en_cours_de_correction = 0;

	Lire_Capteurs_White(); //nécéssaire de relir l'information pour le switch case

	// ---------------------------------------------------------//
	// CAS 1 : On détecte une ligne MAINTENANT					//
	// ---------------------------------------------------------//

	//ici on choisit si la surface est noir ou blanche
	if (etat_capteurs_White != 0)
	{
		// On enregistre l'heure (Tick) à laquelle on aura le droit d'arrêter de tourner
		temps_fin_virage = HAL_GetTick() + DUREE_L;
		en_cours_de_correction = 1;

		// 1. DÉTECTION DU DOUBLE IMPACT//
		if ((etat_capteurs_White & W_AVD) && (etat_capteurs_White & W_AVG))
		{
			memoire_double_frontal = 1;
		}
		// --- EXÉCUTION DES MOUVEMENTS ---
		if (memoire_double_frontal)
		{
			// Si on a touché avec les deux, on RECULE quoi qu'il arrive
			// tant qu'un des deux capteurs avant voit encore du blanc.
			// 2. LOGIQUE DE MOUVEMENT//
			if (etat_capteurs_White & (W_AVD | W_AVG))
			{
				// Si on est en mémoire de double frontal, on force le recul tant qu'un capteur avant touche
				Moteur_RECULER();
			}
			/*else
			{
				// Si les deux avant ne voient plus rien mais qu'on est encore proche de la ligne
				Moteur_PIVO_Droite(); // Pivot pour se réorienter vers le centre
			}*/
		}
		else
		{
			// Sinon, on suit la stratégie classique
			switch (etat_capteurs_White)
			{
				case  W_AVD:             Moteur_TOURNER_Gauche(); 		break;
				case  W_AVG:             Moteur_TOURNER_Droite(); 		break;
				case (W_AVD | W_AVG): 	 Moteur_RECULER();				break; 		// Reculer prend la priorité     bon et note pour la double mémoire
				case  W_ARD:             Moteur_PIVO_Gauche(); 			break;
				case  W_ARG:             Moteur_TOURNER_Droite(); 		break;
				case (W_ARD | W_ARG):    Moteur_TOURNER_Droite(); 		break;
				case (W_ARD | W_AVD):    Moteur_TOURNER_Gauche(); 		break; 	// Toute la droite -> Fuir à gauche
				case (W_ARG | W_AVG):    Moteur_TOURNER_Droite(); 		break; 	// Toute la gauche -> Fuir à droite
				case (W_AVD | W_ARG):	 Moteur_TOURNER_Gauche();	  	break;
				case (W_AVG | W_ARD):	 Moteur_TOURNER_Droite();	  	break;
				default:                 Moteur_STOP();     			break; 			// Sécurité
			}//fin switch case
		}//fin else
	}//fin if etat_capteur_white

	//

	// idée si la dernière valeur de etat_capteur_white = (W_AVD | W_AVG) alors si le capteur avant gauche capte seulement, faire tourner moteur droit en arrière
	// 																			si le capteur avant droite capte seulement, faire tourner moteur gauche en arrière

	//


	// ---------------------------------------------------------
	// CAS 2 : On ne voit PLUS la ligne, mais on vérifie le TIMER
	// ---------------------------------------------------------
	else {
			// Est-ce qu'on est encore dans la période de "virage forcé" ?
			if (en_cours_de_correction == 1)
			{
				if (HAL_GetTick() < temps_fin_virage)
				{
					// OUI : On ne fait rien !
					// On laisse les moteurs faire ce qu'ils faisaient au tour précédent (Tourner)
					// C'est ça qui remplace le HAL_Delay sans bloquer le code.
				}
				else
				{
					// NON : Le temps est écoulé, on peut repasser en mode normal
					en_cours_de_correction = 0;
					memoire_double_frontal = 0; // ON RESET LA MÉMOIRE ICI
				}
			}
			else
			{
				// Si aucune correction n'est en cours
				Strategie_Dist_sensors();

			}
		}
}
//=======================STRATEGIE=====================//


//====================LECTURE CAPTEUR DISTANCE===================//
void Lire_Capteurs_Dist(void) {

    etat_capteur_Dist = 0;
    if (ADC_Read_Channel(CH_DIST_AVC) > SEUIL_DISTANCE) (etat_capteur_Dist |= DIST_AVC);
    if (ADC_Read_Channel(CH_DIST_AVG) > SEUIL_DISTANCE) (etat_capteur_Dist |= DIST_AVG);
    if (ADC_Read_Channel(CH_DIST_AVD) > SEUIL_DISTANCE) (etat_capteur_Dist |= DIST_AVD);
    if (ADC_Read_Channel(CH_DIST_ARC) > SEUIL_DISTANCE) (etat_capteur_Dist |= DIST_ARC);
}
//====================LECTURE CAPTEUR DISTANCE===================//


//====================LECTURE ADC ChANNEL IN===================//
// Configure le canal, lance la conversion et retourne la valeur

uint8_t ADC_Read_Channel(uint32_t channel) {

	ADC_ChannelConfTypeDef sConfig = {0};
																				// 1. Configurer le canal
    sConfig.Channel 		= channel;
    sConfig.Rank 			= ADC_REGULAR_RANK_1;
    sConfig.SamplingTime 	= ADC_SAMPLETIME_640CYCLES_5; 						// Temps d'échantillonnage(640)
    sConfig.SingleDiff 		= ADC_SINGLE_ENDED;
    sConfig.OffsetNumber 	= ADC_OFFSET_NONE;
    sConfig.Offset 			= 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0;			// Gérer l'erreur si besoin return 0;
    HAL_ADC_Start(&hadc1);														// 2. Lancer la conversion

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {						// 3. Attendre la fin

        uint8_t val = HAL_ADC_GetValue(&hadc1);									// 4. Lire la valeur
        HAL_ADC_Stop(&hadc1);													// arrêter de convertir
        return val;																//retourner la valeur pour l'exploiter
    }

    HAL_ADC_Stop(&hadc1);														// s'éxécute so la conversion à mis plus de 20ms et force l'arrêt du convertisseur return 0;
    return 0;
}
//====================LECTURE ADC HELPER===================//


//=======================STRATEGIE DISTANCE=====================//
void Strategie_Dist_sensors(void)
{

	// Variables "static" : elles gardent leur valeur entre chaque appel de la fonction
	// C'est la mémoire du robot.
	static uint32_t temps_fin_virage = 0;
	static uint8_t en_cours_de_correction = 0;

	Lire_Capteurs_Dist();
	// ---------------------------------------------------------
	// CAS 1 : On détecte un robot MAINTENANT
	// ---------------------------------------------------------
	if (etat_capteur_Dist != 0)
	{
		switch ((uint16_t)etat_capteur_Dist)
		{
			case DIST_AVC							: Moteur_AVANCER(); 			break;
			case DIST_AVG							: Moteur_TOURNER_Gauche(); 		break;
			case DIST_AVD							: Moteur_TOURNER_Droite(); 		break;
			case DIST_ARC							: Moteur_RECULER(); 			break;
			default									: Moteur_AVANCER(); 			break;
		}
		if (etat_capteur_Dist == DIST_ARC)
				{
					// On enregistre l'heure (Tick) à laquelle on aura le droit d'arrêter de tourner
					temps_fin_virage = HAL_GetTick() + DUREE_D;
					en_cours_de_correction = 1;
				}

	}
	// ---------------------------------------------------------
	// CAS 2 : On ne voit PLUS le robot, mais on vérifie le TIMER
	// ---------------------------------------------------------
	else
	{
		// Est-ce qu'on est encore dans la période de "demi forcé" ?
		if (en_cours_de_correction == 1)
		{
			if (HAL_GetTick() < temps_fin_virage)
			{
			// OUI : On ne fait rien !
			// On laisse les moteurs faire ce qu'ils faisaient au tour précédent (Tourner)
			// C'est ça qui remplace le HAL_Delay sans bloquer le code.
			}
			else
			{
				// NON : Le temps est écoulé, on peut repasser en mode normal
				en_cours_de_correction = 0; // Ou redonner la main à la stratégie distance
			}
		}
		else
		{
			// Si aucune correction n'est en cours, on avance (ou autre stratégie)
			Moteur_AVANCER();
		}
	}
}
//=======================STRATEGIE DISTANCE=====================//



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
