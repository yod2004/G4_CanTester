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
#include "Robstride.h"
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
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

RobStride_Motor RobStride_01(0x7F, false);
uint8_t mode = 2;
int cnt;
uint8_t torque = 5;
uint8_t angle;
uint8_t speed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN1_Config(void){
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;
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

	TxHeader.Identifier = 0x1200FD01;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = 8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	FDCAN_RxHeaderTypeDef RxHeader;

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
		if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  FDCAN1_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(mode)
	  	      {
	  	          // ===== 普通模式接口 =====
	  	          case 0: // 使能（普通模式）
	  	              RobStride_01.Enable_Motor();
	  	              break;
	  	          case 1: // 失能（普通模式）
	  	              RobStride_01.Disenable_Motor(1);
	  	              break;
	  	          case 2: // 运控模式
	  	              HAL_Delay(5);
	  	              RobStride_01.RobStride_Motor_move_control(torque, 0, 0, 0.0, 0.0);
	  	              break;
	  	          case 3: // PP位置模式
	  	              RobStride_01.RobStride_Motor_Pos_control(2.0, 2);
	  	  						HAL_Delay(5);
	  	              break;
	  	  				case 4:	//CSP位置模式
	  	  						RobStride_01.RobStride_Motor_CSP_control(2.0, 2.0);
	  	  						HAL_Delay(5);
	  	  						break;
	  	          case 5: // 速度模式
	  	              RobStride_01.RobStride_Motor_Speed_control(3.5, 5.0);
	  	  						HAL_Delay(5);
	  	              break;
	  	          case 6: // 电流模式
	  	              HAL_Delay(5);
	  	              RobStride_01.RobStride_Motor_current_control(1.2);
	  	              break;
	  	          case 7: // 设置机械零点
	  	              RobStride_01.Set_ZeroPos();
	  	              break;
	  	          case 8: // 读取参数
	  	              RobStride_01.Get_RobStride_Motor_parameter(0x7014);
	  	              break;
	  	          case 9: // 设置参数
	  	              RobStride_01.Set_RobStride_Motor_parameter(0x7014, 0.35f, Set_parameter);
	  	              break;
	  	          case 10: // 协议切换（如切MIT协议/Canopen/私有协议）
	  	              RobStride_01.RobStride_Motor_MotorModeSet(0x02); // 0x02=MIT
	  	              break;

	  	          // ===== MIT模式接口（只能用MIT专用函数！） =====
	  	          case 11: // MIT 使能
	  	              RobStride_01.RobStride_Motor_MIT_Enable();
	  	              break;
	  	          case 12: // MIT 失能
	  	              RobStride_01.RobStride_Motor_MIT_Disable();
	  	              break;
	  	          case 13: // MIT 综合控制
	  	  						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
	  	              RobStride_01.RobStride_Motor_MIT_Enable();
	  	              HAL_Delay(5);
	  	              RobStride_01.RobStride_Motor_MIT_Control(0, 0, 0, 0, -1.0f);
	  	              break;
	  	          case 14: // MIT 位置控制
	  	  						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
	  	  						RobStride_01.RobStride_Motor_MIT_Enable();
	  	              HAL_Delay(5);
	  	              RobStride_01.RobStride_Motor_MIT_PositionControl(1.57f, 3.0f);
	  	              break;
	  	          case 15: // MIT 速度控制
	  	  						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x02);
	  	  						RobStride_01.RobStride_Motor_MIT_Enable();
	  	              HAL_Delay(5);
	  	              RobStride_01.RobStride_Motor_MIT_SpeedControl(4.5f, 3.2f);
	  	              break;
	  	          case 16: // MIT 零点设置（运行前需保证 MIT_Type != positionControl）
	  	              RobStride_01.RobStride_Motor_MIT_SetZeroPos();
	  	              break;
	  	          case 17: // MIT 清错
	  	              RobStride_01.RobStride_Motor_MIT_ClearOrCheckError(0x01);
	  	              break;
	  	          case 18: // MIT 设置电机运行模式
	  	              RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
	  	              break;
	  	          case 19: // MIT 设置电机ID
	  	              RobStride_01.RobStride_Motor_MIT_SetMotorId(0x05);
	  	              break;
	  	          case 20: //主动上报
	  	              RobStride_01.RobStride_Motor_ProactiveEscalationSet(0x00);
	  	              break;
	  	          case 21: // 波特率修改
	  	              RobStride_01.RobStride_Motor_BaudRateChange(0x01);
	  	              break;
	  	          case 22: // MIT 参数保存
	  	              RobStride_01.RobStride_Motor_MotorDataSave();
	  	              break;
	  	  				case 23: // MIT 协议切换（如切MIT协议/Canopen/私有协议）
	  	  						RobStride_01.RobStride_Motor_MIT_MotorModeSet(0x00);
	  	  						break;

	  	          default:
	  	              break;
	  	      }
//	  	  cnt++;
//	  	  if(cnt<100){
//	  		  mode = 2;
//	  	  }else if(cnt < 200){
//	  		  torque = 5;
//	  	  }else if(cnt < 300){
//	  		  torque = 0;
//	  	  }else if(cnt < 400){
//	  		  cnt = 0;
//	  	  }
	  	  		HAL_Delay(50);
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
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 4;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
