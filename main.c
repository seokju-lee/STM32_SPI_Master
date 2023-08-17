/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14159265358979
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint16_t T_buffer[66] = {0, };
uint16_t Rx_buffer[66];
uint16_t R_buffer[66];
double tar_q1;
double tar_q2;
double tar_q3;
double tar_qd;
int n = 0;
uint16_t checksum;
uint16_t rxchecksum;
typedef union _float_to_uint{
	float data;
	uint16_t buffer[2];
}ftu;
struct spi_data_t
{
    float q_abad[2];
    float q_hip[2];
    float q_knee[2];
    float qd_abad[2];
    float qd_hip[2];
    float qd_knee[2];
    int32_t flags[2];
    int32_t checksum;
}spi_data;
ftu tar_qa, tar_qa2;
ftu tar_qh, tar_qh2;
ftu tar_qk, tar_qk2;
ftu tar_qd_, tar_qd_2;
ftu tau_ff_a, tau_ff_a2;
ftu tau_ff_h, tau_ff_h2;
ftu tau_ff_k, tau_ff_k2;
ftu kpa, kph, kpk, kda, kdh, kdk;
ftu kpa2, kph2, kpk2, kda2, kdh2, kdk2;
float f = 0.1;
float Ts = 0.002;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spiTransmitReceiveData(unsigned char* T_data, unsigned char* R_data){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);

	if(HAL_SPI_TransmitReceive(&hspi1, T_data, R_data, 66, 100) != HAL_OK){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}

	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


void float_for_spi(ftu a, int i){
	for (int j = 0; j < 2; j ++){
		T_buffer[i+j] = a.buffer[j];
	}
}

void spicommand(void){
	float_for_spi(tar_qa, 0);
	float_for_spi(tar_qa2, 2);
	float_for_spi(tar_qh, 4);
	float_for_spi(tar_qh2, 6);
	float_for_spi(tar_qk, 8);
	float_for_spi(tar_qk2, 10);

	float_for_spi(tar_qd_, 12);
	float_for_spi(tar_qd_2, 14);
	float_for_spi(tar_qd_, 16);
	float_for_spi(tar_qd_2, 18);
	float_for_spi(tar_qd_, 20);
	float_for_spi(tar_qd_2, 22);

	float_for_spi(kpa, 24);
	float_for_spi(kpa2, 26);
	float_for_spi(kph, 28);
	float_for_spi(kph2, 30);
	float_for_spi(kpk, 32);
	float_for_spi(kpk2, 34);

	float_for_spi(kda, 36);
	float_for_spi(kda2, 38);
	float_for_spi(kdh, 40);
	float_for_spi(kdh2, 42);
	float_for_spi(kdk, 44);
	float_for_spi(kdk2, 46);

	float_for_spi(tau_ff_a, 48);
	float_for_spi(tau_ff_a2, 50);
	float_for_spi(tau_ff_h, 52);
	float_for_spi(tau_ff_h2, 54);
	float_for_spi(tau_ff_k, 56);
	float_for_spi(tau_ff_k2, 58);
}

void spidata(void){
	for (int i = 0; i < 30; i++){
		((uint16_t*)(&spi_data))[i] = Rx_buffer[i];
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
  tar_qa.data = 0;
  tar_qh.data = 0;
  tar_qk.data = 0;
  tar_qd_.data = 0;
  tau_ff_a.data = 0;
  tau_ff_h.data = 0;
  tau_ff_k.data = 0;
  kpa.data = 2000;
  kda.data = 80;
  kph.data = 1500;
  kdh.data = 80;
  kpk.data = 1000;
  kdk.data = 80;

  tar_qa2.data = 0;
  tar_qh2.data = 0;
  tar_qk2.data = 0;
  tar_qd_2.data = 0;
  tau_ff_a2.data = 0;
  tau_ff_h2.data = 0;
  tau_ff_k2.data = 0;
  kpa2.data = 2000;
  kda2.data = 80;
  kph2.data = 1500;
  kdh2.data = 80;
  kpk2.data = 1000;
  kdk2.data = 80;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_SPI_Init(&hspi1) != HAL_OK){};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	spiTransmitReceiveData(T_buffer, Rx_buffer);

//	tar_q1 = 6*sin(2*pi*f*n*Ts);
//	tar_q2 = 6*cos(2*pi*f*n*Ts);
//	tar_q3 = -6*sin(2*pi*f*n*Ts);
//	n += 1;
//
//	tar_qa.data = (float) tar_q1;
//	tar_qh.data = (float) tar_q2;
//	tar_qk.data = (float) tar_q3;
	spidata();
	spicommand();
	HAL_Delay(2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
