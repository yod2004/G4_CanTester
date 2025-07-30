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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCREEN_HEIGHT 360
#define PADDLE_HEIGHT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */
uint8_t adc_value;//0~255

float player_paddle_y;
float cpu_paddle_y;

uint8_t button1_state;
uint8_t button2_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
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
  MX_ADC1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init(&hi2c3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);

  player_paddle_y = SCREEN_HEIGHT / 2 -
  PADDLE_HEIGHT / 2;
  cpu_paddle_y = SCREEN_HEIGHT / 2 -
  PADDLE_HEIGHT / 2;
  resetBall(); // ボールを初期位置に
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  player_paddle_y = (int)(((float)adc_value / 4095.0f) * (SCREEN_HEIGHT - PADDLE_HEIGHT));

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) ==
	         GPIO_PIN_RESET) {
	            if (button1_state == 0) {
	              if (current_game_state ==
	         GAME_STATE_START || current_game_state ==
	         GAME_STATE_GAMEOVER) {
	                player_score = 0;
	                cpu_score = 0;
	                resetBall();
	                current_game_state =
	         GAME_STATE_PLAYING;
	              }
	            }
	            button1_state = 1;
	          } else {
	            button1_state = 0;
	          }

	          // ボタン2の処理 (ポーズ/リセット -
//	         必要であれば実装)
	          // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) ==
	         GPIO_PIN_RESET) { ... }


	          // --- ゲームロジック ---
	          if (current_game_state == GAME_STATE_PLAYING)
	         {
	              // ボールの移動
	              ball_x += ball_vx;
	              ball_y += ball_vy;

	              // ボールと上下の壁の衝突
	              if (ball_y <= 0 || ball_y >=
	         SCREEN_HEIGHT - BALL_SIZE) {
	                  ball_vy *= -1; // Y方向の速度を反転
	              }

	              // ボールとプレイヤーパドルの衝突
	              if (ball_x <= PADDLE_WIDTH &&
	                  ball_y + BALL_SIZE >= player_paddle_y
	         &&
	                  ball_y <= player_paddle_y +
	         PADDLE_HEIGHT) {
	                  ball_vx *= -1; // X方向の速度を反転
	                  //
	         ボールがパドルのどこに当たったかでY方向の速度を微
	         調整すると面白い
	              }

	              // ボールとCPUパドルの衝突
	              if (ball_x >= SCREEN_WIDTH - PADDLE_WIDTH
	         - BALL_SIZE &&
	                  ball_y + BALL_SIZE >= cpu_paddle_y &&
	                  ball_y <= cpu_paddle_y +
	         PADDLE_HEIGHT) {
	                  ball_vx *= -1; // X方向の速度を反転
	              }

	              // ボールが左右の壁を越えた場合（得点）
	              if (ball_x < 0) { // CPUが得点
	                  cpu_score++;
	                  resetBall();
	                  if (cpu_score >= 5) { // 例:
	         5点先取でゲームオーバー
	                      current_game_state =
	         GAME_STATE_GAMEOVER;
	                  }
	              } else if (ball_x > SCREEN_WIDTH -
	         BALL_SIZE) { // プレイヤーが得点
	                  player_score++;
	                  resetBall();
	                  if (player_score >= 5) { // 例:
	         5点先取でゲームオーバー
	                      current_game_state =
	         GAME_STATE_GAMEOVER;
	                  }
	              }

	              // CPUパドルのAI (シンプルにボールを追従)
	              if (ball_y < cpu_paddle_y + PADDLE_HEIGHT
	         / 2) {
	                  cpu_paddle_y -= 1; //
	         ボールがパドルより上なら上に移動
	              } else if (ball_y > cpu_paddle_y +
	         PADDLE_HEIGHT / 2) {
	                  cpu_paddle_y += 1; //
	         ボールがパドルより下なら下に移動
	              }
	              // パドルが画面外に出ないように制限
	              if (cpu_paddle_y < 0) cpu_paddle_y = 0;
	              if (cpu_paddle_y > SCREEN_HEIGHT -
	         PADDLE_HEIGHT) cpu_paddle_y = SCREEN_HEIGHT -
	         PADDLE_HEIGHT;
	          }


	          // --- 描画処理 ---
	          ssd1306_Fill(Black); //
	         毎フレーム画面をクリア

	          if (current_game_state == GAME_STATE_START) {
	              ssd1306_SetCursor(20, 20);
	              ssd1306_WriteString("PONG GAME",
	         Font_7x10, White);
	              ssd1306_SetCursor(10, 40);
	              ssd1306_WriteString("Press BTN1 to Start"
	         , Font_6x8, White);
	          } else if (current_game_state ==
	         GAME_STATE_PLAYING) {
	              // パドルとボールを描画
	              ssd1306_DrawFilledRectangle(0,
	         player_paddle_y, PADDLE_WIDTH, PADDLE_HEIGHT,
	         White); // プレイヤーパドル
	              ssd1306_DrawFilledRectangle(SCREEN_WIDTH
	         - PADDLE_WIDTH, cpu_paddle_y, PADDLE_WIDTH,
	         PADDLE_HEIGHT, White); // CPUパドル
	              ssd1306_DrawFilledRectangle(ball_x,
	         ball_y, BALL_SIZE, BALL_SIZE, White); // ボール

	              // スコア表示
	              char score_str[10];
	              sprintf(score_str, "%d : %d",
	         player_score, cpu_score);
	              ssd1306_SetCursor(SCREEN_WIDTH / 2 - (
	         strlen(score_str) * Font_7x10.FontWidth) / 2, 0);
	         // 中央上部に表示
	              ssd1306_WriteString(score_str, Font_7x10,
	         White);
	          } else if (current_game_state ==
	         GAME_STATE_GAMEOVER) {
	              ssd1306_SetCursor(20, 20);
	              ssd1306_WriteString("GAME OVER",
	         Font_7x10, White);
	              char final_score_str[20];
	              sprintf(final_score_str, "Final: %d - %d"
	         , player_score, cpu_score);
	              ssd1306_SetCursor(10, 40);
	              ssd1306_WriteString(final_score_str,
	         Font_6x8, White);
	              ssd1306_SetCursor(10, 50);
	              ssd1306_WriteString("Press BTN1 to
	         Restart", Font_6x8, White);
	          }

	          ssd1306_UpdateScreen(); //
	         描画内容をディスプレイに反映

	          HAL_Delay(10); // ゲームの速度調整
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
