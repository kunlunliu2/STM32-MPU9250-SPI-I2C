/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "MPU9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void send_uart(char * string)
{
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t *) string, len, 1000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char msg[90];
	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
	float yaw, pitch, roll;
	float Now = 0.0;
	//float q[4] = {1.0,0.0,0.0,0.0};
	//float deltat = 0.0;

	MPU9250_t MyMPU = { {1.0,0.0,0.0,0.0}, 0.0, {0.0,0.0,0.0}, 0 };

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if(c == 0x71){
	  sprintf(msg, "MPU9250 start, Address = %x \r\n", c);
	  send_uart(msg);

	  MPU9250SelfTest(SelfTest);
	  HAL_Delay(1000);
	  calibrateMPU9250(gyroBias, accelBias);
	  HAL_Delay(1000);
	  initMPU9250();
	  HAL_Delay(1000);
	  initAK8963(magCalibration);
	  HAL_Delay(1000);

  }else
  {
	  sprintf(msg, "MPU9250 fail \r\n");
	  send_uart(msg);
  }

  c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  sprintf(msg, "AK8963 start, Address = %x \r\n", c);
  send_uart(msg);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { // On interrupt, check if data ready interrupt
		    readAccelData(accelCount);  // Read the x/y/z adc values
		    getAres();

		    // Now we'll calculate the accleration value into actual g's
		    ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		    ay = (float)accelCount[1]*aRes; // - accelBias[1];
		    az = (float)accelCount[2]*aRes; // - accelBias[2];

		    readGyroData(gyroCount);  // Read the x/y/z adc values
		    getGres();

		    // Calculate the gyro value into actual degrees per second
		    gx = (float)gyroCount[0]*gRes*PI/180.0;  // get actual gyro value, this depends on scale being set
		    gy = (float)gyroCount[1]*gRes*PI/180.0;
		    gz = (float)gyroCount[2]*gRes*PI/180.0;

		    readMagData(magCount);  // Read the x/y/z adc values
		    getMres();
		    magbias[0] = +491.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
		    magbias[1] = -495.;  // User environmental x-axis correction in milliGauss
		    magbias[2] = -1015.;  // User environmental x-axis correction in milliGauss

		    // Calculate the magnetometer values in milliGauss
		    // Include factory calibration per data sheet and user environmental corrections
		    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
		    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
		    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

		  Now = HAL_GetTick();
		  MyMPU.deltat = ((Now - MyMPU.lastUpdate)*0.001); // set integration time by time elapsed since last filter update
		  MyMPU.lastUpdate = Now;

		  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
		  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
		  // We have to make some allowance for this orientationmis match in feeding the output to the quaternion filter.
		  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
		  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
		  // This is ok by aircraft orientation standards!
		  // Pass gyro rate as rad/s
		  MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, my, mx, mz, MyMPU.q, MyMPU.deltat);
		 // MahonyQuaternionUpdate(ax, ay, az, gx, gy, gz, my, mx, mz, q, deltat);

		 // MadgwickQuaternionUpdate(&MyMPU);

		   // Mheading = atan2(my, mx)*57.29578;

		  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		  // In this coordinate system, the positive z-axis is down toward Earth.
		  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
		  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		  yaw = atan2(2.0f * (MyMPU.q[1] * MyMPU.q[2] + MyMPU.q[0] * MyMPU.q[3]), MyMPU.q[0] * MyMPU.q[0] + MyMPU.q[1] * MyMPU.q[1] - MyMPU.q[2] * MyMPU.q[2] - MyMPU.q[3] * MyMPU.q[3]);

		  pitch = -asin(2.0f * (MyMPU.q[1] * MyMPU.q[3] - MyMPU.q[0] * MyMPU.q[2]));
		  roll = atan2(2.0f * (MyMPU.q[0] * MyMPU.q[1] + MyMPU.q[2] * MyMPU.q[3]), MyMPU.q[0] * MyMPU.q[0] - MyMPU.q[1] * MyMPU.q[1] - MyMPU.q[2] * MyMPU.q[2] + MyMPU.q[3] * MyMPU.q[3]);
		  pitch *= 180.0f / PI;
		  yaw *= 180.0f / PI;
		  yaw -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		  roll *= 180.0f / PI;

	    sprintf(msg, "%7ld,yaw=%7.2f,pitch=%7.2f,roll=%7.2f,mx=%7.2f,my=%7.2f,mz=%7.2f\r\n", HAL_GetTick(), yaw, pitch, roll, mx, my, mz);
	    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
	    HAL_Delay(100);

	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

