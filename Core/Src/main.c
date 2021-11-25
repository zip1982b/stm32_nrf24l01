/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
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

/* USER CODE BEGIN PV */
 uint8_t res;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin	== IRQ_Pin)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"IRQ\n", strlen("IRQ\n"), 1000);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //const uint64_t pipe0 = 0x787878787878;
  const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес первой трубы
  //const uint64_t pipe2 = 0xE8E8F0F0A2LL;
  //const uint64_t pipe3 = 0xE8E8F0F0D1LL;
  //const uint64_t pipe4 = 0xE8E8F0F0C3LL;
  //const uint64_t pipe5 = 0xE8E8F0F0E7LL;
  
  res = isChipConnected(); // проверяет подключён ли модуль к SPI

  char str[64] = {0,};
  snprintf(str, 64, "Connected: %s\n", 1 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  res = NRF_Init(); // инициализация

  snprintf(str, 64, "Init: %s\n", res > 0 && res < 255 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  ////////////// SET ////////////////
  enableAckPayload();
  //setAutoAck(false);
  //setPayloadSize(3);
  setChannel(19);
  #if defined PTX
  openWritingPipe(pipe1);
  #endif
  #if defined PRX
  openReadingPipe(1, pipe1);
  startListening();
  #endif
  
  ///////////////////////////////////

  ////////////////////////// Вывод всяких статусов, для работы не нужно /////////////////////////////
  uint8_t status = get_status();
  snprintf(str, 64, "get_status: 0x%02x\n", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getPALevel();
  snprintf(str, 64, "getPALevel: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_MIN\n", strlen("RF24_PA_MIN\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_LOW\n", strlen("RF24_PA_LOW\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_HIGH\n", strlen("RF24_PA_HIGH\n"), 1000);
  }
  else if(status == 0x03)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_MAX\n", strlen("RF24_PA_MAX\n"), 1000);
  }

  status = getChannel();
  snprintf(str, 64, "getChannel: 0x%02x № %d\n", status, status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getDataRate();
  snprintf(str, 64, "getDataRate: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_250KBPS\n", strlen("RF24_250KBPS\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_2MBPS\n", strlen("RF24_2MBPS\n"), 1000);
  }
  else
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_1MBPS\n", strlen("RF24_1MBPS\n"), 1000);
  }

  status = getPayloadSize();
  snprintf(str, 64, "getPayloadSize: %d\n", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getCRCLength();
  snprintf(str, 64, "getCRCLength: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_DISABLED\n", strlen("RF24_CRC_DISABLED\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_8\n", strlen("RF24_CRC_8\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_16\n", strlen("RF24_CRC_16\n"), 1000);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////


  maskIRQ(true, true, true); // маскируем прерывания
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ///////////////////////////////////// ПРИЁМ /////////////////////////////////////////////
	
	
	#if defined PRX
	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	//static uint8_t remsg = 0;
	char remsg[] = "Hello Zhan!";
	uint8_t pipe_num = 0;
	
	if(available(&pipe_num)) // проверяем пришло ли что-то
	{
		//remsg++;

		writeAckPayload(pipe_num, &remsg, sizeof(remsg)); // отправляем полезную нагрузку вместе с подтверждением

		if(pipe_num == 0) // проверяем в какую трубу пришли данные
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 0\n", strlen("pipe 0\n"), 1000);
		}

		else if(pipe_num == 1)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 1\n", strlen("pipe 1\n"), 1000);

			uint8_t count = getDynamicPayloadSize(); // смотрим сколько байт прилетело

			read(&nrf_data, count); // Читаем данные в массив nrf_data и указываем сколько байт читать
                        //writeAckPayload(pipe_num, &nrf_data[0], sizeof(&nrf_data[0])); // отправляем полезную нагрузку вместе с подтверждением
			if(nrf_data[0] == 77 && nrf_data[1] == 86 && nrf_data[2] == 97) // проверяем правильность данных
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				snprintf(str, 64, "data[0]=%d data[1]=%d data[2]=%d\n", nrf_data[0], nrf_data[1], nrf_data[2]);
				HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
			}
		}

		else if(pipe_num == 2)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 2\n", strlen("pipe 2\n"), 1000);
		}

		else
		{
			while(availableMy()) // если данные придут от неуказанной трубы, то попадут сюда
			{
				read(&nrf_data, sizeof(nrf_data));
				HAL_UART_Transmit(&huart2, (uint8_t*)"Unknown pipe\n", strlen("Unknown pipe\n"), 1000);
			}
		}
	}
	#endif
	

	#if defined PTX
	uint8_t nrf_data[32] = {0,}; 
	nrf_data[0] = 77;
	nrf_data[1] = 86;
	nrf_data[2] = 97;
	//uint8_t remsg = 0; // переменная для приёма байта пришедшего вместе с ответом
	char remsg[30] = {0,};
	if(write(&nrf_data, strlen((const char*)nrf_data))) // отправляем данные
	  {
		if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
		{
			read(&remsg, sizeof(remsg));
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			snprintf(str, 64, "Ack: %s\n", remsg);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
		}
	  }
	else HAL_UART_Transmit(&huart2, (uint8_t*)"Not write\n", strlen("Not write\n"), 1000);

	HAL_Delay(100);
	#endif
	
	
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
