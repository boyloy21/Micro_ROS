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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Accel_stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bno055_vector_t v;
bno055_vector_t V;
Acceleration_t Stepper1;
Acceleration_t Stepper2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R 0.03
#define L
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pi= 3.14;
float r= 0.03; //[m]
float CPR=1200.00;
//Parameter to find Encoder
float V;
volatile float nowA[4];
volatile float nowB[4];
volatile float lastA[4];
volatile float lastB[4];
volatile float dir[4];
float cnt[4];
float Enc_count[1];

//parameter to find Distance
float dis;
float revolution;
float diff;
float c;
float new_count;
//Parameter to find Speed
float count; // count pulse from encoder
float count_state;
float rev; // difference between count and new_count in a sample time
float speed;
float d_prev=0.00;
float sampleTime =1000.00; //[2ms]
float d_current=0.00;
float deg=0.0;
uint16_t omega_left=0;
uint16_t omega_right=0;
float V_x;
float V_z;
int stepDelay = 500; // 1000us more delay means less speed
signed int set_theta1=6400*38;
int counter=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float encoder(int i)
{
	if (nowA[i] != lastA[i])
	{
		lastA[i] = nowA[i];
		if (lastA[i] == 0)
		{
			if (nowB[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowB[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i])
	{
		lastB[i] = nowB[i];
		if (lastB[i] == 0)
		{
			if (nowA[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowA[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //IMU
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();

  //Stepper Motor
  Accel_Stepper_SetPin(&Stepper1, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7);
  Accel_Stepper_SetTimer(&Stepper1, &htim4);

  //TIMER
  HAL_TIM_Base_Start_IT(&htim4);//10ms
  HAL_TIM_Base_Start_IT(&htim5);//10ms

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == R1_C1_Pin || R1_C2_Pin)
	{ // Rotary encoder1
		nowA[0] = HAL_GPIO_ReadPin(R1_C1_GPIO_Port, R1_C1_Pin);
		nowB[0] = HAL_GPIO_ReadPin(R1_C2_GPIO_Port, R1_C2_Pin);
		Enc_count[0] = encoder(0);
	}
	if (GPIO_Pin == R2_C1_Pin || R2_C2_Pin)
	{ // Rotary encoder 2
		nowA[1] = HAL_GPIO_ReadPin(R2_C1_GPIO_Port, R2_C1_Pin);
		nowB[1] = HAL_GPIO_ReadPin(R2_C2_GPIO_Port, R2_C2_Pin);
		Enc_count[1] = encoder(1);
	}
	/*if (GPIO_Pin == M3_C1_Pin || M3_C2_Pin)
	{ // ENCODER Motor 3
		nowA[2] = HAL_GPIO_ReadPin(M3_C1_GPIO_Port, M3_C1_Pin);
		nowB[2] = HAL_GPIO_ReadPin(M3_C2_GPIO_Port, M3_C2_Pin);
		Enc_count[2] = encoder(2);
	}
	if (GPIO_Pin == M4_C1_Pin || M4_C2_Pin)
	{ // ENCODER Motor 4
		nowA[3] = HAL_GPIO_ReadPin(M4_C1_GPIO_Port, M4_C1_Pin);
		nowB[3] = HAL_GPIO_ReadPin(M4_C2_GPIO_Port, M4_C2_Pin);
		Enc_count[3] = encoder(3);
	}*/
}
/* USER CODE END 4 */

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
  if (htim->Instance == TIM4)
  {
		new_count = Enc_count[0];
		rev = new_count / CPR;
		c = pi * r;
		d_current = c * rev;
		if (d_current >= 0) {
			d_current = d_current;
		} else {
			d_current = -1 * d_current;
		}
		diff= d_current- d_prev;
		speed=abs(diff)*1000.0f/sampleTime;//ms
		d_prev=d_current;
  }
  if (htim->Instance == TIM5)
  {
	  omega_left=(2/R)*(V_x - L*V_z);
	  omega_right=(2/R)*(V_x + L*V_z);
	  if(Stepper1.run_status == 0){
	  	 Accel_Stepper_Move(&Stepper1, set_theta1, 1000, 1000, 1000);
	  	 set_theta1 = 0;//reset steps to 0 (prevent re-run after done)
	  }
  }

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
