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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "gps.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	huart2.Instance->DR = (uint8_t) ch;

	/* Loop until the end of transmission */
	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
	{
	}

	return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern char USART_RX_BUF[USART_REC_LEN];
extern _SaveData Save_Data;
char rx_flag = 0;
int size = 0, rx_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("hello\r\n");
	/*
	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	 __HAL_DMA_ENABLE(&hdma_usart1_rx);
	 HAL_UART_Receive_DMA(&huart1, USART_RX_BUF, 1600);
	 */
	//开启DMA空闲中断
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)&USART_RX_BUF, 1600);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//获取到数据，标志位已置1
		if (Save_Data.isGetData == 1)
		{
			//接收标志位置1
			rx_flag = 0;
			//反转电平，LED等闪烁
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			// HAL_UART_Transmit_DMA(&huart2, Save_Data.GPS_Buffer, 80);
			//解析接收到的数据
			parseGpsBuffer();
			Save_Data.isGetData = 0;
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//回调函数，当接收到数据时调用此函数
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	extern char USART_RX_BUF[USART_REC_LEN];
	extern DMA_HandleTypeDef hdma_usart1_rx;
	if (huart == &huart1)
	{
		HAL_UART_DMAStop(huart);
		/***********************************************/
		char *p;
		//寻找“strstr”字符串，将地址给p
		p = strstr(USART_RX_BUF, "GNRMC");
		//如果找到“strstr”，则地址不为空
		if (p != NULL)
		{
			//printf("s:%d\r\n",(uint32_t)(p - USART_RX_BUF));
			//判断字符串出现的位置
			if ((p >= USART_RX_BUF) && (p < &USART_RX_BUF[1000])
					&& ((p - USART_RX_BUF) < 500))
			{

				memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);//清空数据缓冲区
				memcpy(Save_Data.GPS_Buffer, p, 80); 	//保存数据
				Save_Data.isGetData = true;//获取到数据，标志位置1

			}
			rx_flag = 1;//接收标志位置1
		}
		//memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		rx_count++;
		/********************************************************************************/
		//当前接收到的数据大小
		size = 1600 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		//清除空闲接收标志位
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		//重新开启DMA接收
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)&USART_RX_BUF, 1600);

	}
}
//回调函数，当接收到数据时调用此函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (Size == 800)
	{
		return;
	}
	if (huart == &huart1)
	{

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		printf("size:%d\r\n", Size);
		/*************************************/
		char *p;
		p = strstr(USART_RX_BUF, "GNRMC");
		if (p != NULL)
		{
			printf("s:%d\r\n", (uint32_t)(p - USART_RX_BUF));
			if ((p >= USART_RX_BUF) && (p < &USART_RX_BUF[1000])
					&& ((p - USART_RX_BUF) < 1000))
			{
				memset(&Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
				memcpy(&Save_Data.GPS_Buffer, p, 80); 	//保存数据
				Save_Data.isGetData = true;
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)(&Save_Data.GPS_Buffer), 80);
			}
		}
		/**********************************/
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*)&USART_RX_BUF, 1600);
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
