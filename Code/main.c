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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wizchip_conf.h"
#include "socket.h"
#include <stm32f4xx_hal.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SEPARATOR            "=============================================\r\n"
#define WELCOME_MSG  		 "Welcome to STM32Nucleo Ethernet configuration\r\n"
#define NETWORK_MSG  		 "Network configuration:\r\n"
#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"
#define GREETING_MSG 		 "Well done guys! Welcome to the IoT world. Bye!\r\n"
#define CONN_ESTABLISHED_MSG "Connection established with remote IP: %d.%d.%d.%d:%d\r\n"
#define SENT_MESSAGE_MSG	 "Sent a message. Let's close the socket!\r\n"
#define WRONG_RETVAL_MSG	 "Something went wrong; return value: %d\r\n"
#define WRONG_STATUS_MSG	 "Something went wrong; STATUS: %d\r\n"
#define LISTEN_ERR_MSG		 "LISTEN Error!\r\n"

#define PRINT_STR(msg) do  {										\
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);		\
} while(0)

#define PRINT_HEADER() do  {													\
  HAL_UART_Transmit(&huart5, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
  HAL_UART_Transmit(&huart5, (uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), 100);	\
  HAL_UART_Transmit(&huart5, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
} while(0)

#define PRINT_NETINFO(netInfo) do { 																					\
  HAL_UART_Transmit(&huart5, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);															\
} while(0)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char msg[60];
uint16_t okunan_veriler[20];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void cs_sel() {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel() {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void spi_wb(uint8_t b) {
	HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
}
void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart5, (uint8_t *) string, len, HAL_MAX_DELAY);  // transmit in blocking mode
}
uint8_t fuction6(uint16_t deger);
uint8_t fuction3(uint16_t *deger,uint8_t adet);

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
	uint8_t rcvBuf[20], bufSize[] = {2, 2, 2, 2};
	uint16_t deneme=0;
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  PRINT_HEADER();

    reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
    reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

    wizchip_init(bufSize, bufSize);
    wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
                            .ip 	= {192, 168, 1, 192},					// IP address
                            .sn 	= {255, 255, 255, 0},					// Subnet mask
                            .gw 	= {192, 168, 2, 1}};					// Gateway address
    wizchip_setnetinfo(&netInfo);
    wizchip_getnetinfo(&netInfo);
    PRINT_NETINFO(netInfo);
    HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (fuction6(deneme)==0)
	  	  {
	  		  send_uart("Fonksiyon 6 OK... \n");
	  	  }
	  	  else
	  	  {
	  		  send_uart("Fonksiyon 6 Error... \n");
	  		  while(1)
	  		  {
	  			  HAL_Delay(500);
	  			  HAL_GPIO_TogglePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin);
	  		  }
	  	  }
	  	  HAL_Delay(1000);
	  	  if (fuction3(okunan_veriler,5)==0)
	  	  {
	  		  send_uart("Fonksiyon 3 OK... \n");
	  		  for(int i=0;i<5;i++)
	  		  {
	  			  char mesaj[20];
	  			  sprintf(mesaj,"- %d",okunan_veriler[i]);
	  			  send_uart(mesaj);
	  			  send_uart("..\n");
	  		  }


	  	  }
	  	  else
	  	  {
	  		  send_uart("Fonksiyon 3 Error... \n");
	  		  while(1)
	  		  {
	  			  HAL_Delay(500);
	  			  HAL_GPIO_TogglePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin);
	  		  }
	  	  }
	  	  HAL_Delay(1000);
	  	  HAL_GPIO_TogglePin(yesil_led_GPIO_Port, yesil_led_Pin);
	  	  deneme++;
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
  RCC_OscInitStruct.PLL.PLLN = 82;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|Kirmizi_led_Pin|sari_led_Pin|yesil_led_Pin
                          |CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 Kirmizi_led_Pin sari_led_Pin yesil_led_Pin
                           CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|Kirmizi_led_Pin|sari_led_Pin|yesil_led_Pin
                          |CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t fuction6(uint16_t deger)
{
	uint8_t s=0;
	uint8_t time_out;
	uint8_t fonsiyon_6_istek[12];
	uint8_t fonsiyon_6_cevap[30];
	uint8_t slave_ip[4]={192,168,1,101};
	uint8_t cevap;
	uint8_t received_len=0,baglanma_istegi,RSR_len;

	cevap=socket(s,Sn_MR_TCP,502,SF_TCP_NODELAY);
	if(cevap!=0)
	{
		while(1)
		{
			HAL_GPIO_TogglePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin);
			HAL_Delay(500);
			return 1;
		}
	}
	for(int i=0;i<10;i++)
	{

		baglanma_istegi=connect(s,slave_ip,502);
		if(baglanma_istegi==SOCK_OK)
		{
			HAL_GPIO_WritePin(yesil_led_GPIO_Port, yesil_led_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin, GPIO_PIN_RESET);
			break;
			//send_uart("Baglanti Basarili \n");
		}
		else
		{
			HAL_GPIO_WritePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(yesil_led_GPIO_Port, yesil_led_Pin, GPIO_PIN_RESET);
			//send_uart("Baglanti Basarisiz.. \n");
		}
		HAL_Delay(100);
	}
	fonsiyon_6_istek[0]=34;
	fonsiyon_6_istek[1]=20;
	fonsiyon_6_istek[2]=0;
	fonsiyon_6_istek[3]=0;
	fonsiyon_6_istek[4]=0;
	fonsiyon_6_istek[5]=6;
	fonsiyon_6_istek[6]=1;
	fonsiyon_6_istek[7]=6;
	fonsiyon_6_istek[8]=0;
	fonsiyon_6_istek[9]=1;
	fonsiyon_6_istek[10]=(deger>>8)&0xff;
	fonsiyon_6_istek[11]=deger&0xff;

	send(s,fonsiyon_6_istek,12);
	while(received_len==0)
	{
		HAL_GPIO_WritePin(sari_led_GPIO_Port, sari_led_Pin, GPIO_PIN_SET);
		if ((RSR_len = getSn_RX_RSR(s)) > 0)
		{
			received_len=recv(s,fonsiyon_6_cevap,RSR_len);
		  if(fonsiyon_6_cevap[0]==fonsiyon_6_istek[0]&&fonsiyon_6_cevap[1]==fonsiyon_6_istek[1])
		  {
			  return 0;
		  }
		  else
		  {
			  return 2;
		  }
		}
		else
		{
			time_out++;
			if(time_out>250)
			{
				return 4;
			}
		}

		HAL_Delay(1);
	}
	return 4;
}
uint8_t fuction3(uint16_t *deger,uint8_t adet)
{
	uint8_t s=0;
	uint8_t time_out;
	uint8_t fonsiyon_6_istek[12];
	uint8_t fonsiyon_6_cevap[30];
	uint8_t slave_ip[4]={192,168,1,101};
	uint8_t cevap;
	uint8_t received_len=0,baglanma_istegi,RSR_len;

	cevap=socket(s,Sn_MR_TCP,502,SF_TCP_NODELAY);
	if(cevap!=0)
	{
		while(1)
		{
			HAL_GPIO_TogglePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin);
			HAL_Delay(500);
			return 1;
		}
	}
	for(int i=0;i<10;i++)
	{

		baglanma_istegi=connect(s,slave_ip,502);
		if(baglanma_istegi==SOCK_OK)
		{
			HAL_GPIO_WritePin(yesil_led_GPIO_Port, yesil_led_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin, GPIO_PIN_RESET);
			break;
			//send_uart("Baglanti Basarili \n");
		}
		else
		{
			HAL_GPIO_WritePin(Kirmizi_led_GPIO_Port, Kirmizi_led_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(yesil_led_GPIO_Port, yesil_led_Pin, GPIO_PIN_RESET);
			//send_uart("Baglanti Basarisiz.. \n");
		}
		HAL_Delay(100);
	}
	fonsiyon_6_istek[0]=45;
	fonsiyon_6_istek[1]=80;
	fonsiyon_6_istek[2]=0;
	fonsiyon_6_istek[3]=0;
	fonsiyon_6_istek[4]=0;
	fonsiyon_6_istek[5]=6;
	fonsiyon_6_istek[6]=1;
	fonsiyon_6_istek[7]=3;
	fonsiyon_6_istek[8]=0;
	fonsiyon_6_istek[9]=0;
	fonsiyon_6_istek[10]=0;
	fonsiyon_6_istek[11]=adet;

	send(s,fonsiyon_6_istek,12);
	while(received_len==0)
	{
		HAL_GPIO_WritePin(sari_led_GPIO_Port, sari_led_Pin, GPIO_PIN_SET);
		if ((RSR_len = getSn_RX_RSR(s)) > 0)
		{
			received_len=recv(s,fonsiyon_6_cevap,RSR_len);
			if(fonsiyon_6_cevap[0]==fonsiyon_6_istek[0]&&fonsiyon_6_cevap[1]==fonsiyon_6_istek[1])
			{
				for(int i=0;i<adet;i++)
				{
					*(deger+i)=(fonsiyon_6_cevap[9+(i*2)]<<8)|fonsiyon_6_cevap[10+(i*2)];
				}
				return 0;
			}
			else
			{
				return 2;
			}
		}
		else
		{
			time_out++;
			if(time_out>250)
			{
				return 4;
			}
		}
		HAL_Delay(1);
	}
	return 4;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
