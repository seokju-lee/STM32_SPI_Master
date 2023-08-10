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
#include "cmsis_os.h"

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint16_t T_buffer[66] = {0, };
uint16_t Rx_buffer[66];
uint16_t R_buffer[66];
float Kp1 = 700;
float Kd1 = 70;
float Kp2 = 200;
float Kd2 = 30;
float Kp3 = 300;
float Kd3 = 60;
double tar_q1;
double tar_q2;
double tar_q3;
double tar_qd;
int n = 0;
int bytecount = 0;
int cnt_ = 0;
int cnta = 0;
int cnth = 0;
int cntk = 0;
float angle1;
float angle2;
float angle3;
float qd1;
float qd2;
float qd3;
float init_angle1;
float init_angle2;
float init_angle3;
float f = 0.1;
float Ts = 0.002;
float debug = 0;
uint16_t checksum;
uint16_t rxchecksum;
typedef union _packet {
      int16_t data;
      uint16_t buffer[2];
}Packet;
typedef union _packet2 {
	uint16_t data;
	uint16_t  buffer[2];
}Packet2;
typedef union _packet3{
    uint32_t data;
    uint16_t buffer[4];
}Packet3;
typedef union _float_to_uint{
	float data;
	uint16_t buffer[2];
}ftu;
Packet2 position1;
Packet2 position2;
Packet2 position3;
uint16_t init_data1;
uint16_t init_data2;
uint16_t init_data3;
Packet speed1;
Packet speed2;
Packet speed3;
ftu tar_qa;
ftu tar_qh;
ftu tar_qk;
ftu tar_qd_;
ftu tau_ff_a;
ftu tau_ff_h;
ftu tau_ff_k;
ftu kpa, kph, kpk, kda, kdh, kdk;
uint16_t prev_pos1 = 0;
int16_t prev_vel1 = 0;
uint16_t prev_pos2 = 0;
int16_t prev_vel2 = 0;
uint16_t prev_pos3 = 0;
int16_t prev_vel3 = 0;
Packet3 init_pos1;
Packet3 init_pos2;
Packet3 init_pos3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t xor_checksum(uint16_t* data, size_t len)
{
	uint16_t t = 0;
	for(int i = 0; i < len; i++){
		t = t ^ data[i];
	}
	return t;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_SPI_Init(&hspi1) != HAL_OK){};
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
void spiTransmitReceiveData(unsigned char* T_data, unsigned char* R_data){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);

	if(HAL_SPI_TransmitReceive(&hspi1, T_data, R_data, 66, 10000) != HAL_OK){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}

	for(int k = 0; k < 66; k++){
		R_buffer[k] = R_data[k];
	}

	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

//void init_data(void){
//	for(int i = 0; i < 4; i++){
//		init_pos1.buffer[i] = R_buffer[i];
//		init_pos2.buffer[i] = R_buffer[i+4];
//		init_pos3.buffer[i] = R_buffer[i+8];
//	}
//	init_angle1 = ((float) init_pos1.data)*0.001*(pi/180);
//	init_angle2 = ((float) init_pos2.data)*0.001*(pi/180);
//	init_angle3 = ((float) init_pos3.data)*0.001*(pi/180);
//}
//
//void convertData(void){
//	prev_pos1 = position1.data;
//	prev_pos2 = position2.data;
//	prev_pos3 = position3.data;
//
//	for(int i=0; i<2; i++){
//		position1.buffer[i] = R_buffer[i];
//		speed1.buffer[i] = R_buffer[i+2];
//		position2.buffer[i] = R_buffer[i+4];
//		speed2.buffer[i] = R_buffer[i+6];
//		position3.buffer[i] = R_buffer[i+8];
//		speed3.buffer[i] = R_buffer[i+10];
//	}
//	if(position1.data > 16383){
//		position1.data = prev_pos1;
//	}
//	if(position2.data > 16383){
//		position2.data = prev_pos2;
//	}
//	if(position3.data > 16383){
//		position3.data = prev_pos3;
//	}
//}
//
//void countEncoder(){
//	if ((position1.data < 5000) && (prev_pos1 > 10000)){
//		cnta += 1;
//	}
//	else if ((prev_pos1 < 5000) && (position1.data > 10000)){
//		cnta -= 1;
//	}
//	if ((position2.data < 5000) && (prev_pos2 > 10000)){
//		cnth += 1;
//	}
//	else if ((prev_pos2 < 5000) && (position2.data > 10000)){
//		cnth -= 1;
//	}
//	if ((position3.data < 5000) && (prev_pos3 > 10000)){
//		cntk += 1;
//	}
//	else if ((prev_pos3 < 5000) && (position3.data > 10000)){
//		cntk -= 1;
//	}
//}
//
//void save_init_data(void){
//	for(int i=0; i<2; i++){
//		position1.buffer[i] = R_buffer[i];
//		position2.buffer[i] = R_buffer[i+4];
//		position3.buffer[i] = R_buffer[i+8];
//	}
//	init_data1 = position1.data;
//	init_data2 = position2.data;
//	init_data3 = position3.data;
//	cnta = 0;
//	cnth = 0;
//	cntk = 0;
//}
//
//
//float lowerbound(float angle){
//	while(angle < 0){
//		angle += 2*pi;
//	}
//	return angle;
//}
//
//float upperbound(float angle){
//	while(angle > 2*pi){
//		angle -= 2*pi;
//	}
//	return angle;
//}

void float_for_spi(ftu a, int i){
	for (int j = 0; j < 2; j ++){
		T_buffer[i+j] = a.buffer[j];
	}
}

void spicommand(void){
	float_for_spi(tar_qa, 0);
	float_for_spi(tar_qh, 4);
	float_for_spi(tar_qk, 8);

	float_for_spi(tar_qd_, 12);
	float_for_spi(tar_qd_, 16);
	float_for_spi(tar_qd_, 20);

	float_for_spi(kpa, 24);
	float_for_spi(kph, 28);
	float_for_spi(kpk, 32);

	float_for_spi(kda, 36);
	float_for_spi(kdh, 40);
	float_for_spi(kdk, 44);

	float_for_spi(tau_ff_a, 48);
	float_for_spi(tau_ff_h, 52);
	float_for_spi(tau_ff_k, 56);
}

//void spidata(void){
//
//}

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
  /* Infinite loop */
  tar_qa.data = 0;
  tar_qh.data = 0;
  tar_qk.data = 0;
  tar_qd_.data = 0;
  tau_ff_a.data = 0;
  tau_ff_h.data = 0;
  tau_ff_k.data = 0;
  kpa.data = 700;
  kda.data = 70;
  kph.data = 1000;
  kdh.data = 80;
  kpk.data = 1000;
  kdk.data = 80;
  for(;;)
  {
	spiTransmitReceiveData(T_buffer, Rx_buffer);

//	tar_q1 = 6*sin(2*pi*f*n*Ts);
//	tar_q2 = 6*cos(2*pi*f*n*Ts);
//	tar_q3 = -6*sin(2*pi*f*n*Ts);
//	n += 1;
//
//	tar_qa.data = (float) tar_q1;
//	tar_qh.data = (float) tar_q2;
//	tar_qk.data = (float) tar_q3;

	spicommand();
    osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(2);
  }
  /* USER CODE END StartTask02 */
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
