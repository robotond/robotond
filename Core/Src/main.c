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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "ism330dhcx_reg.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	BOOT_TIME          10 //ms
#define	U1_BUFFER_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
uint8_t result;
char buffer[U1_BUFFER_SIZE];
uint8_t *p_usb_rx=NULL;
uint16_t *p_usb_len=NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void ism330dhcx_read_data_polling(void);
void I2C_Scanner(void);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  I2C_Scanner();
  ism330dhcx_read_data_polling();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);
	  HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDWH_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDWH_Pin PB9 */
  GPIO_InitStruct.Pin = LEDWH_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void I2C_Scanner(void){
	char buffer[100];
	uint8_t i,n;
	  HAL_GPIO_WritePin(LEDWH_GPIO_Port, LEDWH_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LEDWH_GPIO_Port, LEDWH_Pin, GPIO_PIN_RESET);
	  n=snprintf(buffer,U1_BUFFER_SIZE-1,"\r\nI2C BUS #:");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 	for (i=1; i<128; i++)
	 	{
	 	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
	 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	 	  {
	 		  n=snprintf(buffer,U1_BUFFER_SIZE-1,".");
	 		  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 		  // No ACK received at that address
	 	  }
	 	  if (result == HAL_OK)
	 	  {
	 		  n=snprintf(buffer,U1_BUFFER_SIZE-1,"[0x%X]", i);
	 		  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 		  // Received an ACK at that address
	 	  }
	 	}
	 	n=snprintf(buffer,U1_BUFFER_SIZE-1,"\r\nI2C BUS #2:");
	 	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 	HAL_Delay(200);
	 	for (i=1; i<128; i++)
	 	{
	 	  result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 2, 2);
	 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	 	  {
	 		  n=snprintf(buffer,U1_BUFFER_SIZE-1,".");
	 		  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 		  // No ACK received at that address
	 	  }
	 	  if (result == HAL_OK)
	 	  {
	 		  n=snprintf(buffer,U1_BUFFER_SIZE-1,"[0x%X]", i);
	 		  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, n, 100);
	 		  // Received an ACK at that address
	 	  }
	 	}
	 	HAL_Delay(200);

}


void ism330dhcx_read_data_polling(void)
{
uint8_t led_state=GPIO_PIN_RESET;

  stmdev_ctx_t dev_ctx;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c2;
  /* Wait sensor boot time */
  HAL_Delay(BOOT_TIME);
  /* Check device ID */
  ism330dhcx_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ISM330DHCX_ID)
    while (1);

  /* Restore default configuration */
  ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    ism330dhcx_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start device configuration. */
  ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable Block Data Update */
  ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate 52Hz; shall I increase? */
  ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_52Hz);
  ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_52Hz);
  /* Set full scale;Csabi needs most sensitive settings-to be revised */
  ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
  ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_4000dps);
  /* Configure filtering chain(No aux interface)
   * What is it?
   * Accelerometer - LPF1 + LPF2 path
   * What is it?
   */
  ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
  ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* Read samples in polling mode (no int) (TODO: INT driven data aquisition/RTOS thread?) */
  while (1) {
    uint8_t reg;
    /*Easy to determine actual data reading frequency*/
    HAL_GPIO_WritePin(LEDWH_GPIO_Port, LEDWH_Pin, led_state);
    led_state=!led_state;
    HAL_Delay(20); /*20ms ensures success if 52Hz is chosen! */
    /* Read output only if new xl value is available */
    ism330dhcx_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration field data */

      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[2]);
      sprintf((char *)tx_buffer,
              "%012lu,%6.2f,%6.2f,%6.2f,",HAL_GetTick(),
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer)); //UART output version if no USB is present
      CDC_Transmit_FS(tx_buffer, strlen((char const *)tx_buffer));
    }

    ism330dhcx_gy_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      sprintf((char *)tx_buffer,
              "%6.2f,%6.2f,%6.2f,",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer)); //UART output version if no USB is present
      CDC_Transmit_FS(tx_buffer, strlen((char const *)tx_buffer));
    }

    ism330dhcx_temp_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      ism330dhcx_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = ism330dhcx_from_lsb_to_celsius(
                           data_raw_temperature);
      sprintf((char *)tx_buffer,
              "%06.2f\r\n", temperature_degC);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer)); //UART output version if no USB is present
      CDC_Transmit_FS(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, ISM330DHCX_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}


/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
  while (1)
  {
      sprintf((char *)tx_buffer,"Ooooop, something went wrong %012lu", HAL_GetTick());
      CDC_Transmit_FS(tx_buffer, strlen((char const *)tx_buffer));
      HAL_Delay(500);
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
