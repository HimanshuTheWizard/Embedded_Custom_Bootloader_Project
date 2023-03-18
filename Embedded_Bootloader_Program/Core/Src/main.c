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
#include "string.h"
#include "stm32f4xx_hal.h"

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
#define MAX_COMMAND_LEN 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t Command_Rcv_Buffer[MAX_COMMAND_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
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
     uint32_t tick_cnt;
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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//When button is pressed
		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			bootloader_uart_read_data();
		}
		else
		{
			bootloader_jump_to_usr_application();
		}
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
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
Description : used to send ack to host
*/
void bootloader_send_ack(uint8_t len)
{
	uint8_t ack_data_buffer[2];
	ack_data_buffer[0] = BOOTLOADER_ACK;
	ack_data_buffer[1] = len;
	HAL_UART_Transmit(&huart2, ack_data_buffer, 2, HAL_MAX_DELAY);
}

/*
Description : send nack
*/
void bootloader_send_nack(void)
{
	uint8_t nack_data_buffer = BOOTLOADER_NACK;
	HAL_UART_Transmit(&huart2, &nack_data_buffer, 1, HAL_MAX_DELAY);
}

/*
Description : Verify CRC
*/
uint8_t Verify_CRC(uint8_t *received_data, uint8_t len, uint32_t target_CRC)
{
	uint32_t accumulated_crc = 0xFF;
	uint32_t i_data;
	for(int i=0; i< len; i++)
	{	
		i_data = received_data[i];
		accumulated_crc = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	if(accumulated_crc == target_CRC)
	{
		return VERIFY_CRC_SUCCESS;
	}
	else
	{
		return VERIFY_CRC_FAIL;
	}
}

/*
Description : get boot loader version
*/
uint8_t get_bootloader_version(void)
{
	return BOOTLOADER_VERSION;
}

/*
Description : get bootloader version cmd handler
*/
void Bootloader_handle_get_version_cmd(uint8_t *received_data)
{
	uint8_t bootloader_ver;
	uint8_t cmd_len = received_data[0]+1;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;

	if(VERIFY_CRC_SUCCESS == Verify_CRC(&received_data[0], cmd_len-4, host_crc))
	{
		//send ack
		bootloader_send_ack(BOOTLOADER_VERSION_RESPONSE_LEN);
		//get bootloader version
		bootloader_ver = get_bootloader_version();
		//bootloader uart write data
		HAL_UART_Transmit(&huart2, &bootloader_ver, BOOTLOADER_VERSION_RESPONSE_LEN, HAL_MAX_DELAY);
	}
	else
	{
		//send nack
		bootloader_send_nack();
	}	
}

/*
Description : get boot loader commands
*/
void Bootloader_get_help_cmd(uint8_t *received_data)
{
	uint8_t supported_cmd_arr[] = {
																	BL_GET_VER,
																	BL_GET_CID,
																	BL_GET_HELP,
																	BL_GET_RDP_STATUS,
																	BL_GO_TO_ADDR,
																	BL_FLASH_ERASE,
																	BL_MEM_WRITE,
																	BL_ENABLE_R_W_PROTECT,
																	BL_MEM_READ,
																	BL_READ_SECTOR_STATUS,
																	BL_OTP_READ,
																	BL_DIS_R_W_PROTECT
																};
	uint8_t cmd_len = received_data[0]+1;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len - 4, host_crc))
	{
		bootloader_send_ack(sizeof(supported_cmd_arr));
		HAL_UART_Transmit(&huart2, supported_cmd_arr, sizeof(supported_cmd_arr), HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}
/*
Description : get bootloader CID
*/
uint16_t get_chip_id(void)
{
	uint16_t Chip_ID = (DBGMCU->IDCODE)&0x0FFF;;
	return Chip_ID;
}

/*
Description : get bootloader CID command handler
*/
void Bootloader_handle_get_cid_cmd(uint8_t *received_data)
{
	uint16_t Chip_ID;
	uint8_t cmd_len = received_data[0]+1;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	//get chip ID
	Chip_ID = get_chip_id();
	//check CRC
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len-4, host_crc))
	{
		bootloader_send_ack(BOOTLOADER_CID_RESPONSE_LEN);
		HAL_UART_Transmit(&huart2, (uint8_t *)&Chip_ID, BOOTLOADER_CID_RESPONSE_LEN, HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}
/*

Description : Get boot loader RDP status
*/
uint8_t get_rdp_status(void)
{
	return ((FLASH->OPTCR)&0xFF00);
}
/*
Description : Get boot loader RDP status command handler
*/
void Bootloader_get_rdp_status_cmd(uint8_t *received_data)
{
	uint8_t rdp_status;
	uint8_t cmd_len = received_data[0]+1;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	//get read protection level
	rdp_status = get_rdp_status();
	//check CRC
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len-4, host_crc))
	{
		bootloader_send_ack(BOOTLOADER_RDP_RESPONSE_LEN);
		HAL_UART_Transmit(&huart2, &rdp_status, BOOTLOADER_RDP_RESPONSE_LEN, HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}

/*
Description : redirect bootloader to an address
*/
void Bootloader_goto_addr_cmd(uint8_t *received_data)
{
	uint32_t *goto_address;
	uint8_t addr_validity_ack;
	uint8_t cmd_len = received_data[0]+1;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	
	//check CRC
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len-4, host_crc))
	{
		bootloader_send_ack(BOOTLOADER_GOTO_ADDR_RESPONSE_LEN);
		//extract go-to address
		goto_address = (uint32_t *)&received_data[2];
		goto_address+=1;
		//check address validity
		addr_validity_ack = check_address_validity(goto_address);
		HAL_UART_Transmit(&huart2, &addr_validity_ack, BOOTLOADER_GOTO_ADDR_RESPONSE_LEN, HAL_MAX_DELAY);
		if(addr_validity_ack == VALID_ADDRESS)
		{
			void (*jump_add)(void) = (void *)goto_address;
			jump_add();
		}
	}
	else
	{
		bootloader_send_nack();
	}
}
/*
Description : To be fully implemented
*/
uint8_t check_address_validity(uint32_t *goto_address)
{
	return 0;
}
/*
Description : erase flash cmd handler
*/
void Bootloader_erase_flash_cmd(uint8_t *received_data)
{
	uint8_t sector_number;
	uint8_t number_of_sectors;
	uint8_t cmd_len = received_data[0]+1;
	uint32_t sector_error;
	HAL_StatusTypeDef ret_val;
	FLASH_EraseInitTypeDef flash_erase = {
																					FLASH_TYPEERASE_SECTORS,
																					sector_number,
																					FLASH_SECTOR_0,
																					number_of_sectors,
																					FLASH_VOLTAGE_RANGE_3
																				};
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	//get sector number and number of sectors
	sector_number = received_data[2];
	number_of_sectors = received_data[3];
	//check CRC
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len-4, host_crc))
	{
		bootloader_send_ack(BOOTLOADER_MEM_ERASE_RESPONSE_LEN);
		ret_val = HAL_FLASHEx_Erase(&flash_erase, &sector_error);
		HAL_UART_Transmit(&huart2, &ret_val, BOOTLOADER_MEM_ERASE_RESPONSE_LEN, HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}

/*
Description : memory write cmd handler
*/
void Bootloader_mem_write_cmd(uint8_t *received_data)
{
	uint8_t cmd_len = received_data[0]+1;
	uint32_t base_memory_add =  *(uint32_t *)&received_data[2];
	uint8_t *payload = &received_data[7];
	uint8_t payload_len = received_data[6];
	
	HAL_StatusTypeDef ret_val;
	uint32_t host_crc = *((uint32_t * ) (received_data+cmd_len - 4) ) ;
	//check CRC
	if(VERIFY_CRC_SUCCESS == Verify_CRC(received_data, cmd_len-4, host_crc))
	{
		bootloader_send_ack(1);
		if(check_address_validity((uint32_t *)base_memory_add))
		{
			HAL_FLASH_Unlock();
			for(int i=0; i<payload_len; i++)
			{
				ret_val = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)base_memory_add+i, payload[i]);
			}
			HAL_FLASH_Lock();
		}
		HAL_UART_Transmit(&huart2, &ret_val, 1, HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}

/*
Description : read write protect command handler 
*/
void Bootloader_enable_rw_protect_cmd(uint8_t *received_data)
{
	
}

/*
Description : memory read cmd handler
*/
void Bootloader_mem_read_cmd(uint8_t *received_data)
{

}

/*
Description : command handler for reading the protection level of sectors
*/
void Bootloader_read_sector_status_cmd(uint8_t *received_data)
{

}

/*
Description : otp read cmd handler
*/
void Bootloader_otp_read_cmd(uint8_t *received_data)
{

}

/*
Description : disables the protection level for sectors
*/
void Bootloader_diable_rw_protect_cmd(uint8_t *received_data)
{
		
}

/*
Description : When bootloader mode is enabled, user jump to this function
							and all the bootloader commands are handled in this function
*/
void bootloader_uart_read_data(void)
{
	uint8_t cmd_len;
	while(1)
	{
		memset(Command_Rcv_Buffer, 0, 100);
		//Reading cmd length
		HAL_UART_Receive(&huart2, &Command_Rcv_Buffer[0], 1, HAL_MAX_DELAY);
		cmd_len = Command_Rcv_Buffer[0];
		//Reading remaining command
		HAL_UART_Receive(&huart2, &Command_Rcv_Buffer[1], cmd_len, HAL_MAX_DELAY);
		switch(Command_Rcv_Buffer[1])
		{
			case BL_GET_VER:
				Bootloader_handle_get_version_cmd(Command_Rcv_Buffer);
				break;
			case BL_GET_HELP:
				Bootloader_get_help_cmd(Command_Rcv_Buffer);
				break;
			case BL_GET_CID:
				Bootloader_handle_get_cid_cmd(Command_Rcv_Buffer);
				break;
			case BL_GET_RDP_STATUS:
				Bootloader_get_rdp_status_cmd(Command_Rcv_Buffer);
				break;
			case BL_GO_TO_ADDR:
				Bootloader_goto_addr_cmd(Command_Rcv_Buffer);
				break;
			case BL_FLASH_ERASE:
				Bootloader_erase_flash_cmd(Command_Rcv_Buffer);
				break;
			case BL_MEM_WRITE:
				Bootloader_mem_write_cmd(Command_Rcv_Buffer);
				break;
			/*To be implemented*/
			case BL_ENABLE_R_W_PROTECT:
				Bootloader_enable_rw_protect_cmd(Command_Rcv_Buffer);
				break;
			/*To be implemented*/
			case BL_MEM_READ:
				Bootloader_mem_read_cmd(Command_Rcv_Buffer);
				break;
			/*To be implemented*/
			case BL_READ_SECTOR_STATUS:
				Bootloader_read_sector_status_cmd(Command_Rcv_Buffer);
				break;
			/*To be implemented*/
			case BL_OTP_READ:
				Bootloader_otp_read_cmd(Command_Rcv_Buffer);
				break;
			/*To be implemented*/
			case BL_DIS_R_W_PROTECT:
				Bootloader_diable_rw_protect_cmd(Command_Rcv_Buffer);
				break;
			default:
				break;
		}
	}
}

/*
Description : When bootloader mode is not enabled execution jump to this function
*/
void bootloader_jump_to_usr_application(void)
{
	//Declaration of function pointer
	void (*app_reset_handler)(void);
	
	//Reading MSP value of user application
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	
	//Setting MSP value
	__set_MSP(msp_value);
	
	//retrive the value of reset handler
	uint32_t reset_handler = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS+4);
	
	//setting the function pointer to reset handler
	app_reset_handler = (void *)reset_handler;
	
	//Jumping to reset handler of user application
	app_reset_handler();
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
