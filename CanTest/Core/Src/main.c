/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306.h"
#include "fonts.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FDCAN_FilterTypeDef sFilterConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int cnt;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

//for view
int view;

uint16_t adc_value;
char buffer[20];

//for can
uint32_t setted_id = 0x123;
uint8_t setted_prescaler = 1;
uint8_t can_rxdata[8] = {255,255,255,255,255,255,255,255};
uint8_t pre_can_rxdata[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void){

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    /* Configure global filter to reject all non-matching frames */
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK){
        Error_Handler();
    }
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
        Error_Handler();
    }
    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    for(uint8_t i = 0;i<8;i++){
    	pre_can_rxdata[i] = can_rxdata[i];
    }
	FDCAN_RxHeaderTypeDef RxHeader;

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, (uint8_t*)can_rxdata) != HAL_OK){
            Error_Handler();
        }
    }
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
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  //for can
  FDCAN_Config();

  //for ssd1306
  ssd1306_Init(&hi2c3);
  ssd1306_SetCursor(0, 36);
  ssd1306_WriteString("Hello World", Font_11x18, White);
  ssd1306_UpdateScreen(&hi2c3);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_value, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  //setting
	  uint8_t adc_value_configured = (255 - adc_value)/16;
	  char content[20];
	  memcpy(buffer, 0, sizeof(0));
	  memcpy(content, 0, sizeof(0));
	  ssd1306_Fill(Black);
	  uint8_t go_flag;
	  uint8_t back_flag;
	  uint8_t go_state;
	  uint8_t back_state;

	  //read GPIO pin
	  if(go_flag && HAL_GPIO_ReadPin(GO_GPIO_Port, GO_Pin)==GPIO_PIN_SET){
		  go_state = 1;
		  go_flag = 0;
	  }else{
		  go_state = 0;
		  go_flag = 1;
	  }
	  if(back_flag && HAL_GPIO_ReadPin(BACK_GPIO_Port, BACK_Pin)==GPIO_PIN_SET){
		  back_state = 1;
		  back_flag = 0;
	  }else{
		  back_state = 0;
		  back_flag = 1;
	  }

	  if(view == 1){//rx mode, set id of 0xF00;
		  setted_id = setted_id & 0x0FF;
		  setted_id += (adc_value_configured)<<8;
		  sprintf(content, "id 0x%d%d%d   rx mode",(setted_id & 0xF00)>>8, (setted_id & 0x0F0)>>4, setted_id & 0x00F);
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString(content, Font_7x10, White);
		  if(go_state){
			  view = 0;
		  }
	  }else if(view == 2){//rx mode, set id of 0x0F0;
		  setted_id = setted_id & 0xF0F;
		  setted_id += (adc_value_configured)<<4;
		  sprintf(content, "id 0x%d%d%d   rx mode",(setted_id & 0xF00)>>8, (setted_id & 0x0F0)>>4, setted_id & 0x00F);
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString(content, Font_7x10, White);
		  if(go_state){
			  view = 0;
		  }
	  }else if(view == 3){//rx mode, set id of 0x00F;
		  setted_id = setted_id & 0xFF0;
		  setted_id += (adc_value_configured);
		  sprintf(content, "id 0x%d%d%d   rx mode",(setted_id & 0xF00)>>8, (setted_id & 0x0F0)>>4, setted_id & 0x00F);
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString(content, Font_7x10, White);
		  if(go_state){
			  view = 0;
		  }
	  }else if(view == 4){
		 if(adc_value_configured > 7){
			 setted_prescaler = 1;
		 }else{
			 setted_prescaler = 2;
		 }
		 if(setted_prescaler == 1){
			  sprintf(content, "1M");
		  }else if(setted_prescaler == 2){
			  sprintf(content, "500k");
		  }
		  ssd1306_SetCursor(0, 8);
		  ssd1306_WriteString(content, Font_7x10, Black);
		 if(go_state){
			  hfdcan1.Instance = FDCAN1;
			  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
			  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
			  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
			  hfdcan1.Init.AutoRetransmission = DISABLE;
			  hfdcan1.Init.TransmitPause = DISABLE;
			  hfdcan1.Init.ProtocolException = DISABLE;
			  hfdcan1.Init.NominalPrescaler = setted_prescaler;
			  hfdcan1.Init.NominalSyncJumpWidth = 1;
			  hfdcan1.Init.NominalTimeSeg1 = 11;
			  hfdcan1.Init.NominalTimeSeg2 = 4;
			  hfdcan1.Init.DataPrescaler = 1;
			  hfdcan1.Init.DataSyncJumpWidth = 1;
			  hfdcan1.Init.DataTimeSeg1 = 1;
			  hfdcan1.Init.DataTimeSeg2 = 1;
			  hfdcan1.Init.StdFiltersNbr = 1;
			  hfdcan1.Init.ExtFiltersNbr = 0;
			  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
			  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
			  {
			    Error_Handler();
			  }
			  FDCAN_Config();
			 view = 0;
		 }
	  }else if(view == 0){
		  sprintf(buffer, "id 0x%d%d%d   rx mode",(setted_id&0xF00)>>8,(setted_id&0x0F0)>>4,(setted_id&0x00F));
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  if(setted_prescaler == 1){
			  sprintf(buffer, "1M");
		  }else if(setted_prescaler == 2){
			  sprintf(buffer, "500k");
		  }
		  ssd1306_SetCursor(0, 8);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  //put rxdata
		  sprintf(content,"%d %d %d %d", can_rxdata[0],can_rxdata[1],can_rxdata[2],can_rxdata[3]);
		  ssd1306_SetCursor(0,45);
		  ssd1306_WriteString(content, Font_7x10, White);
		  sprintf(content,"  %d %d %d %d", can_rxdata[4],can_rxdata[5],can_rxdata[6],can_rxdata[7]);
		  ssd1306_SetCursor(0,53);
		  ssd1306_WriteString(content, Font_7x10, White);
		  sprintf(content,"%d %d %d %d", pre_can_rxdata[0], pre_can_rxdata[1], pre_can_rxdata[2], pre_can_rxdata[3]);
		  ssd1306_SetCursor(0,25);
		  ssd1306_WriteString(content, Font_7x10, White);
		  sprintf(content,"  %d %d %d %d", pre_can_rxdata[4], pre_can_rxdata[5], pre_can_rxdata[6], pre_can_rxdata[7]);
		  ssd1306_SetCursor(0,33);
		  ssd1306_WriteString(content, Font_7x10, White);
		  //carsor figured;
		  uint8_t column = adc_value_configured;
		  if(column == 5){
			  //hilight cursor
			  sprintf(content, "%d",(setted_id&0xF00)>>8);
			  ssd1306_SetCursor(35, 0);
			  ssd1306_WriteString(content, Font_7x10, Black);
			  if(go_state){
				  view = 1;
			  }
		  }else if(column == 6){
			  //hilight cursor
			  sprintf(content, "%d",(setted_id&0x0F0)>>4);
			  ssd1306_SetCursor(42, 0);
			  ssd1306_WriteString(content, Font_7x10, Black);
			  if(go_state){
				  view = 2;
			  }
		  }else if(column == 7){
			  //hilight cursor
			  sprintf(content, "%d",(setted_id&0x00F));
			  ssd1306_SetCursor(49, 0);
			  ssd1306_WriteString(content, Font_7x10, Black);
			  if(go_state){
				  view = 3;
			  }
		  }else if(column == 9){
			  //hilight cursor
			  if(setted_prescaler == 1){
				  sprintf(content, "1M");
			  }else if(setted_prescaler == 2){
				  sprintf(content, "500k");
			  }
			  ssd1306_SetCursor(0, 8);
			  ssd1306_WriteString(content, Font_7x10, Black);
			  if(go_state){
				  view = 4;
			  }
		  }
	  }

	  //show
	  ssd1306_UpdateScreen(&hi2c3);

	  //can filter config

	  sFilterConfig.FilterID1 = setted_id;
	  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
		  Error_Handler();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00503D58;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : GO_Pin BACK_Pin */
  GPIO_InitStruct.Pin = GO_Pin|BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
