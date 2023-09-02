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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 DIR dp;
 FILINFO fno;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
FRESULT res;    /* FatFs function common result code */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
uint32_t Buffer_Tx[512], Buffer_Rx[512];
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
	  uint32_t byteswritten, bytesread;                     /* File write/read counts */
	  uint8_t wtext[] = "Hello world!";                     /* File write buffer */
	  uint8_t rtext[100];
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
  MX_SDIO_SD_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  printf("> Demo Start \r\n");
  HAL_Delay(500);

  /*##Register the file system object to the FatFs module ##############*/
  printf("> Register the file system object to the FatFs module: ");
  if((res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 0)) != FR_OK) {
    /* FatFs Initialization Error */
    Error_Handler();
  }
  printf("OK\r\n");

  /*##Create and Open a new text file object with write access #####*/
  printf("> Create and Open a new text file object with write access: ");
  if((res = f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
  {
    /* 'STM32.TXT' file Open for write Error */
    Error_Handler();
  }
  printf("OK\r\n");

  /*##Write data to the text file ################################*/
  printf("> Write data to the text file: ");
  if((res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten)) != FR_OK)
  {
    /* Write data Error */
    Error_Handler();
  }
  printf("OK\r\n");

  /*##Close the open text file #################################*/
  printf("> Close the open text file: ");
  if((res = f_close(&MyFile)) != FR_OK )
  {
    Error_Handler();
  }
  if(byteswritten == 0)
  {
    /* 'STM32.TXT' file Write or EOF Error */
    printf("'STM32.TXT' file Write or EOF Error\r\n");
    while(1);
  }
  printf("OK\r\n");

  /*##Open the text file object with read access ###############*/
  printf("> Open the text file object with read access: ");
  if((res = f_open(&MyFile, "STM32.TXT", FA_READ)) != FR_OK)
  {
    /* 'STM32.TXT' file Open for read Error */
    Error_Handler();
  }
  printf("OK\r\n");


  /*##Read data from the text file ###########################*/
  printf("> Read data from the text file: ");
  if((res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread)) != FR_OK)
  {
    Error_Handler();
  }
  if(bytesread == 0)
  {
    /* 'STM32.TXT' file Read or EOF Error */
    printf("'STM32.TXT' file Read or EOF Error\r\n");
    while(1);
  }
  printf("OK\r\n");

  /*##Close the open text file #############################*/
  printf("> Close the open text file: ");
  if((res = f_close(&MyFile)) != FR_OK )
  {
    Error_Handler();
  }
  printf("OK\r\n");

  /*##Compare read data with the expected data ############*/
  printf("> Compare read data with the expected data: ");
  if(bytesread != byteswritten)
  {
    /* Read data is different from the expected data */
    printf("Error: Read data is different from the expected data\r\n");
    while(1);
  }
  else
  {
    /* Success of the demo: no error occurrence */
    printf("OK\r\n");
    printf("> Demo Success\r\n");
  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }


  /**Configure the Systick interrupt time
  */
HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /**Configure the Systick
  */
HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

/* SysTick_IRQn interrupt configuration */
HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
	  switch(res)
	  {
	    case FR_OK:
	      printf("FR_OK = 0, (0) Succeeded \r\n");
	      break;
	    case FR_DISK_ERR:
	      printf("FR_DISK_ERR, (1) A hard error occurred in the low level disk I/O layer \r\n");
	      break;
	    case FR_INT_ERR:
	      printf("FR_INT_ERR, (2) Assertion failed \r\n");
	      break;
	    case FR_NOT_READY:
	      printf("FR_NOT_READY, (3) The physical drive cannot work \r\n");
	      break;
	    case FR_NO_FILE:
	      printf("FR_NO_FILE, (4) Could not find the file \r\n");
	      break;
	    case FR_NO_PATH:
	      printf("FR_NO_PATH, (5) Could not find the path \r\n");
	      break;
	    case FR_INVALID_NAME:
	      printf("FR_INVALID_NAME, (6) The path name format is invalid \r\n");
	      break;
	    case FR_DENIED:
	      printf("FR_DENIED, (7) Access denied due to prohibited access or directory full \r\n");
	      break;
	    case FR_EXIST:
	      printf("FR_EXIST, (8) Access denied due to prohibited access \r\n");
	      break;
	    case FR_INVALID_OBJECT:
	      printf("FR_INVALID_OBJECT, (9) The file/directory object is invalid \r\n");
	      break;
	    case FR_WRITE_PROTECTED:
	      printf("FR_WRITE_PROTECTED, (10) The physical drive is write protected \r\n");
	      break;
	    case FR_INVALID_DRIVE:
	      printf("FR_INVALID_DRIVE, (11) The logical drive number is invalid \r\n");
	      break;
	    case FR_NOT_ENABLED:
	      printf("FR_NOT_ENABLED, (12) The volume has no work area \r\n");
	      break;
	    case FR_NO_FILESYSTEM:
	      printf("FR_NO_FILESYSTEM, (13) There is no valid FAT volume \r\n");
	      break;
	    case FR_MKFS_ABORTED:
	      printf("FR_MKFS_ABORTED, (14) The f_mkfs() aborted due to any parameter error \r\n");
	      break;
	    case FR_TIMEOUT:
	      printf("FR_TIMEOUT, (15) Could not get a grant to access the volume within defined period \r\n");
	      break;
	    case FR_LOCKED:
	      printf("FR_LOCKED, (16) The operation is rejected according to the file sharing policy \r\n");
	      break;
	    case FR_NOT_ENOUGH_CORE:
	      printf("FR_NOT_ENOUGH_CORE, (17) LFN working buffer could not be allocated \r\n");
	      break;
	    case FR_TOO_MANY_OPEN_FILES:
	      printf("FR_TOO_MANY_OPEN_FILES, (18) Number of open files > _FS_SHARE \r\n");
	      break;
	    case FR_INVALID_PARAMETER:
	      printf("FR_INVALID_PARAMETER	    (19) Given parameter is invalid \r\n");
	      break;
	    default:
	      printf("Unknown Error... \r\n");
	      break;
	  }
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
