/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "cmsis_os2.h"
#include "rtx_os.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"

#include "stm32l4xx_hal_dma.h"

#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_tim.h"

#include "Driver_SPI.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "rl_fs.h"

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

/* USER CODE BEGIN PV */
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MX_GPIO_Init(void);
static void DMA_Init(void);
static void SPI_1_Init(void);

#define MAIN_THREAD_STACK_SIZE (4096U)
uint64_t main_thread_stack[MAIN_THREAD_STACK_SIZE / 8];
const osThreadAttr_t main_thread_attributes = {
  .stack_mem  = &main_thread_stack[0],
  .stack_size = sizeof(main_thread_stack),
	.priority = osPriorityAboveNormal7 
};

__NO_RETURN void Main_Thread (void *arg)
{
	(void)arg;

	fsStatus status;
	
	int a = 0;
	
	// Initialize file system
	status = finit("M0:");
	if(status == fsOK) 
	{
		// Mount SD card in M0:
		status = fmount("M0:");
		if(status == fsOK)
		{
			// Successfuly mounted M0:
			a = 0;
		}
		else
		{
			a = 1;
		}
	}
	
	while(1)
	{
	}
}

/**
  Callback function used to read Card Detect (CD) pin state when Memory Card is used in SPI mode.
  \param[in]  drive_num                Memory Card Drive number
  \return     1:card detected, 0:card not detected, or error
*/
int32_t fs_mc_read_cd (uint32_t drive_num)
{
	if (HAL_GPIO_ReadPin(SDIO_IN_PORT, SDIO_IN_PIN) == GPIO_PIN_RESET)
	{
		return 1;
	}
  return 0;
}

/**
  Callback function used to read Write Protect (WP) pin state when Memory Card is used in SPI mode.
  \param[in]  drive_num                Memory Card Drive number
  \return     1:write protected, 0:not write protected, or error
*/
int32_t fs_mc_read_wp (uint32_t drive_num)
{
  return 0;
}

/**
  Callback function used to control Slave Select signal when Memory Card is used in SPI mode.
  \param[in]  drive_num                Memory Card Drive number
  \param[in]  ss                       Slave select signal state (0=inactive, 1=active)
  \return     slave select signal state
*/
int32_t fs_mc_spi_control_ss (uint32_t drive_num, uint32_t ss)
{
	if(ss == 1)
	{
		HAL_GPIO_WritePin(SDIO_CS_PORT, SDIO_CS_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(SDIO_CS_PORT, SDIO_CS_PIN, GPIO_PIN_SET);
	}
	
  return ss;
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

	// Init GPIO
	MX_GPIO_Init();
	
	// Init DMA
	DMA_Init();
	
	// Init SPI_1
	SPI_1_Init();
	
	// Initialise RT OS
	osKernelInitialize();
	
	osThreadNew(Main_Thread, NULL, &main_thread_attributes);
	
	osKernelStart();

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
	/* HSE CLOCK */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  //Configure the main internal regulator output voltage
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  //Initializes the RCC Oscillators according to the specified parameters
  //in the RCC_OscInitTypeDef structure.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
	
	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	

//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
}

/* USER CODE BEGIN 4 */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
	
	/*Configure GPIO pin : SD_IN_Pin (PB13) */
  GPIO_InitStruct.Pin		= SDIO_IN_PIN;
  GPIO_InitStruct.Mode	= GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull	= GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_IN_PORT, &GPIO_InitStruct);
	
	/*Configure GPIO pin : SDIO_CS (PB12) */
	GPIO_InitStruct.Pin		= SDIO_CS_PIN;
	GPIO_InitStruct.Mode	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull	= GPIO_NOPULL;
	GPIO_InitStruct.Speed	= GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SDIO_CS_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(SDIO_CS_PORT, SDIO_CS_PIN, GPIO_PIN_SET);
}

void DMA_Init(void)
{
	  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
	
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void SPI_1_Init(void)
{
		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();
	
		hspi1.Instance								= SPI1;
		hspi1.Init.Mode								= SPI_MODE_MASTER;
		hspi1.Init.Direction					= SPI_DIRECTION_2LINES;
		hspi1.Init.DataSize						= SPI_DATASIZE_8BIT;
	
		hspi1.Init.CLKPolarity				= SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase						= SPI_PHASE_1EDGE;
	
		hspi1.Init.NSS								= SPI_NSS_SOFT;	
		hspi1.Init.NSSPMode						= SPI_NSS_PULSE_ENABLED;
	
		hspi1.Init.BaudRatePrescaler	= SPI_BAUDRATEPRESCALER_2;	
		hspi1.Init.FirstBit						= SPI_FIRSTBIT_MSB;
	
		hspi1.Init.TIMode							= SPI_TIMODE_DISABLED;
	
		hspi1.Init.CRCCalculation			= SPI_CRCCALCULATION_DISABLED;
		hspi1.Init.CRCLength					= SPI_CRC_LENGTH_DATASIZE;
		hspi1.Init.CRCPolynomial			= 7;
			
		/* SPI_1 GPIO Configuration */
    // PA1 -> SDIO_CK_PIN
    // PA6 -> SDIO_MISO_PIN
    // PA7 -> SDIO_MOSI_PIN
		GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin				= SDIO_CK_PIN | SDIO_MISO_PIN | SDIO_MOSI_PIN;
    GPIO_InitStruct.Mode			= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull			= GPIO_NOPULL;
    GPIO_InitStruct.Speed			= GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate	= GPIO_AF5_SPI1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		
		
		if (HAL_SPI_Init(&hspi1) != HAL_OK)
		{
			Error_Handler();
		}
		
		/* SPI1 DMA Init */
		
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance										= DMA1_Channel2;
    hdma_spi1_rx.Init.Direction							= DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc							= DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc								= DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment		= DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment			= DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode									= DMA_NORMAL;
    hdma_spi1_rx.Init.Priority							= DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance										= DMA1_Channel3;
    hdma_spi1_tx.Init.Direction							= DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc							= DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc								= DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment		= DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment			= DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode									= DMA_NORMAL;
    hdma_spi1_tx.Init.Priority							= DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);
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
