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
#include <stdbool.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   // snprintf, sscanf
#include <string.h>  // strlen
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIM2_PWM_MAX 4199U  // 20kHz @ 84MHz
#define TIM3_PWM_MAX  49U   // 20kHz @ 1MHz (PSC=83)  (브러시를 20kHz로 올릴 경우)
 #define TxBufferSize   (countof(TxBuffer) - 1)
 #define TxBufferSize_10 (countof(TxBuffer_10) - 1)
 #define RxBufferSize   0xFF
 #define countof(a)   (sizeof(a) / sizeof(*(a)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char uart_buffer[64];
uint8_t uart_index = 0;
char uart2_buffer[64];
uint8_t uart2_index = 0;

uint8_t uart2_rx_byte = 0;  // ✅ 추가: UART2 인터럽트 수신용 변수

volatile bool stop_requested = false; // 인터럽트 플래그
volatile bool motor_enabled = false;
volatile bool pulse_requested = false;

uint8_t TxBuffer[] = "\n\rUART Ready for communication!!\n\r\n\r";
uint8_t TxBuffer_10[] = "\n\r A Message from HAL_UART_TxCpltCallback() !!\n\r\n\r";
uint8_t RxBuffer[RxBufferSize];

uint16_t current_duty = 0;
GPIO_PinState current_dir = GPIO_PIN_RESET;

void blink_all_leds_twice(void);
void sequential_leds(void);
void driveDualMotor(uint16_t left_duty, GPIO_PinState left_dir,
                    uint16_t right_duty, GPIO_PinState right_dir,
                    uint32_t duration_ms);
#define BRUSH_DIR_PORT GPIOC
#define BRUSH_DIR_PIN  GPIO_PIN_3

#define VACUUM_DIR_PORT GPIOF
#define VACUUM_DIR_PIN  GPIO_PIN_3

#define BRUSH_PWM_CHANNEL TIM_CHANNEL_1  // PC6 (TIM3_CH1)
#define VACUUM_PWM_CHANNEL TIM_CHANNEL_2 // PC7 (TIM3_CH2)

bool extra_motor_on = false;  // USER 버튼으로 on/off 상태 저장

// === Cleaning toggle state (토글 전용) ===
static volatile bool cleaning_toggle_req = false;   // ISR에서 set, 메인루프에서 처리
static bool vacuum_is_on = false;                   // 현재 진공모터 상태
static uint32_t last_clean_toggle_ms = 0;           // 마지막 토글 시각
#define CLEAN_TOGGLE_COOLDOWN_MS 1500               // 연속 토글 최소 간격 (ms)

// --- Slew & Deadband config (모터 2채널 각각 퍼센트 기반) ---
static volatile int g_tgt_pct_L = 0;   // 목표 duty [%] (0~100)
static volatile int g_tgt_pct_R = 0;   // 목표 duty [%]
static volatile GPIO_PinState g_tgt_dir_L = GPIO_PIN_RESET; // 목표 방향
static volatile GPIO_PinState g_tgt_dir_R = GPIO_PIN_RESET;

static volatile int g_cur_pct_L = 0;   // 현재 적용된 duty [%]
static volatile int g_cur_pct_R = 0;
static volatile GPIO_PinState g_cur_dir_L = GPIO_PIN_RESET; // 현재 방향
static volatile GPIO_PinState g_cur_dir_R = GPIO_PIN_RESET;

// 데드밴드: 이 값 미만은 0으로 처리(정지). 필요시 전/후진 분리 가능.
#define DEAD_MIN_PCT  0      // 예: 8% 미만은 힘이 안 실리는 영역

// 슬루(변화율) 제한: 한 틱(10ms)마다 변화 가능한 최대 퍼센트
#define SLEW_PCT_PER_TICK  100 // 예: 2%/tick → 200ms에 40% 변화

// 내부용: % → CCR 변환
static inline uint16_t pct_to_ccr_tim2(int pct){
  if(pct<0)pct=0; if(pct>100)pct=100;
  return (uint16_t)((TIM2_PWM_MAX * pct)/100);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

static void toggleCleaningSystem(void);
void set_motor_targets(int pctL, GPIO_PinState dirL, int pctR, GPIO_PinState dirR);
static int apply_deadband(int pct);
static void slew_step(void);  // 10ms마다 한 스텝씩 현재값을 목표로 이동
static void apply_to_timer_ccr_from_current(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 데드밴드 적용: DEAD_MIN_PCT 미만 → 0
static int apply_deadband(int pct){
  if (pct < DEAD_MIN_PCT) return 0;
  if (pct > 100) pct = 100;
  if (pct < 0) pct = 0;
  return pct;
}

// 라즈베리파이 명령 → 목표 저장 (즉시 타이머에 쓰지 않음)
void set_motor_targets(int pctL, GPIO_PinState dirL, int pctR, GPIO_PinState dirR)
{
  if (pctL < 0) pctL = 0; if (pctL > 100) pctL = 100;
  if (pctR < 0) pctR = 0; if (pctR > 100) pctR = 100;

  g_tgt_pct_L  = pctL;
  g_tgt_pct_R  = pctR;
  g_tgt_dir_L  = dirL;
  g_tgt_dir_R  = dirR;
}

// 한 틱(10ms)마다 현재[%]를 목표[%]로 이동 (슬루 제한 + 데드밴드)
static void slew_step(void)
{
  // 방향 변화가 있으면, 먼저 현재 듀티를 0으로 내려 안전하게 방향 전환
  if (g_cur_dir_L != g_tgt_dir_L && g_cur_pct_L > 0) {
    int dec = (g_cur_pct_L > SLEW_PCT_PER_TICK) ? SLEW_PCT_PER_TICK : g_cur_pct_L;
    g_cur_pct_L -= dec;
  } else {
    // 동일 방향: 목표로 접근
    int deltaL = g_tgt_pct_L - g_cur_pct_L;
    if      (deltaL >  SLEW_PCT_PER_TICK) g_cur_pct_L += SLEW_PCT_PER_TICK;
    else if (deltaL < -SLEW_PCT_PER_TICK) g_cur_pct_L -= SLEW_PCT_PER_TICK;
    else                                  g_cur_pct_L  = g_tgt_pct_L;
    // 방향 동기
    if (g_cur_pct_L == 0) g_cur_dir_L = g_tgt_dir_L;
    else                  g_cur_dir_L = g_tgt_dir_L;
  }

  if (g_cur_dir_R != g_tgt_dir_R && g_cur_pct_R > 0) {
    int dec = (g_cur_pct_R > SLEW_PCT_PER_TICK) ? SLEW_PCT_PER_TICK : g_cur_pct_R;
    g_cur_pct_R -= dec;
  } else {
    int deltaR = g_tgt_pct_R - g_cur_pct_R;
    if      (deltaR >  SLEW_PCT_PER_TICK) g_cur_pct_R += SLEW_PCT_PER_TICK;
    else if (deltaR < -SLEW_PCT_PER_TICK) g_cur_pct_R -= SLEW_PCT_PER_TICK;
    else                                  g_cur_pct_R  = g_tgt_pct_R;
    if (g_cur_pct_R == 0) g_cur_dir_R = g_tgt_dir_R;
    else                  g_cur_dir_R = g_tgt_dir_R;
  }

  // 데드밴드 보정
  g_cur_pct_L = apply_deadband(g_cur_pct_L);
  g_cur_pct_R = apply_deadband(g_cur_pct_R);
}

// 현재[%]와 방향을 실제 타이머 CCR/핀에 반영
static void apply_to_timer_ccr_from_current(void)
{
  // 방향핀
	HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, !g_cur_dir_L); // Left DIR
	HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, !g_cur_dir_R); // Right DIR

  // CCR
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pct_to_ccr_tim2(g_cur_pct_L));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pct_to_ccr_tim2(g_cur_pct_R));
}

// 퍼센트 기반 블로킹 구동: 목표 설정 후 duration_ms 기다림
static void driveDualMotorPct(int pctL, GPIO_PinState dirL,
                              int pctR, GPIO_PinState dirR,
                              uint32_t duration_ms)
{
  set_motor_targets(pctL, dirL, pctR, dirR);
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < duration_ms) {
    if (stop_requested) break;
    HAL_Delay(5);
  }
  // 필요시 정지:
  // set_motor_targets(0, dirL, 0, dirR);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart3, (uint8_t*)"OK\r\n", 4, 100);

  // UART2 통신 확인 (라즈베리파이 ↔ STM32)
  HAL_UART_Transmit(&huart3, (uint8_t*)"UART2 RX echo mode start...\r\n", 30, 100);

  uint32_t last_tx_time = HAL_GetTick();

  // 기존 UART3 메시지 전송
  HAL_UART_Transmit(&huart3, (uint8_t*)TxBuffer, TxBufferSize , 0xFFFF);

  // PWM 초기화
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  // PC6 - 브러쉬
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PC6

  // UART2 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  // UART3 수신 인터럽트 시작 (유지)
  HAL_UART_Receive_IT(&huart3, RxBuffer, 1);

  HAL_TIM_Base_Start_IT(&htim6);

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 	      // UART2 TX 송신 처리 (STM32 → 라즈베리파이)
	 	      if (HAL_GetTick() - last_tx_time >= 1000)  // 1초마다
	 	      {
	 	          const char* msg = "STM32 TX Test\r\n";
	 	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	 	          //HAL_UART_Transmit(&huart3, (uint8_t*)"UART2 TX sent\r\n", 16, 100);  // 디버깅 출력
	 	          last_tx_time = HAL_GetTick();  // 타이머 갱신
	 	      }

	 	      sequential_leds();  // LED 점멸
	 	     // 청소 토글 요청 처리 (있을 때만 1회 실행)
	 	     if (cleaning_toggle_req) {
	 	    	cleaning_toggle_req = false;
	 	    	toggleCleaningSystem();
	 	     }
	 	     if (pulse_requested)
	 	     {
	 	         pulse_requested = false;
	 	         HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
	 	         HAL_Delay(100);  // 충분한 딱딱 지속 시간
	 	         HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
	 	     }

	 	      if (stop_requested)
	 	      {
	 	          slowStopMotor(current_duty, current_dir);
	 	          stop_requested = false;
	 	      }

	 	     if (motor_enabled && !stop_requested)
	 	     {
	 	    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	 	    	HAL_Delay(50);

	 	    	/*
	 	    	// [1] 전진 가속 테스트 (0% → 30%)
	 	    	for (int pct = 0; pct <= 30; pct += 5) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_SET,   // 왼쪽 전진
	 	    	                      pct, GPIO_PIN_SET,   // 오른쪽 전진
	 	    	                      500);
	 	    	}

	 	    	// [2] 좌/우 곡선 주행 (60% vs 15%)
	 	    	driveDualMotorPct(60, GPIO_PIN_SET,  // 좌 60%
	 	    	                  15, GPIO_PIN_SET,  // 우 15%  → 좌로 커브
	 	    	                  2000);
	 	    	driveDualMotorPct(15, GPIO_PIN_SET,  // 좌 15%
	 	    	                  60, GPIO_PIN_SET,  // 우 60%  → 우로 커브
	 	    	                  2000);

	 	    	// [3] 전진 증가/감소 (0→30→0%)
	 	    	for (int pct = 0; pct <= 30; pct += 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_SET,
	 	    	                      pct, GPIO_PIN_SET,
	 	    	                      1000);
	 	    	}
	 	    	for (int pct = 20; pct >= 0; pct -= 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_SET,
	 	    	                      pct, GPIO_PIN_SET,
	 	    	                      500);
	 	    	}

	 	    	HAL_Delay(1000);

	 	    	// [4] 후진 증가/감소 (0→30→0%)
	 	    	for (int pct = 0; pct <= 30; pct += 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_RESET,   // 후진
	 	    	                      pct, GPIO_PIN_RESET,
	 	    	                      500);
	 	    	}
	 	    	for (int pct = 20; pct >= 0; pct -= 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_RESET,
	 	    	                      pct, GPIO_PIN_RESET,
	 	    	                      250);
	 	    	}

	 	    	// [5] 완전 정지
	 	    	driveDualMotorPct(0, GPIO_PIN_SET,
	 	    	                  0, GPIO_PIN_SET,
	 	    	                  1000);

	 	    	// [6] 제자리 회전(반시계) : 좌 후진 / 우 전진
	 	    	for (int pct = 0; pct <= 30; pct += 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_RESET,   // 좌 후진
	 	    	                      pct, GPIO_PIN_SET,     // 우 전진
	 	    	                      1000);
	 	    	}
	 	    	driveDualMotorPct(40, GPIO_PIN_RESET,
	 	    	                  40, GPIO_PIN_SET,
	 	    	                  5000);
	 	    	for (int pct = 30; pct >= 0; pct -= 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_RESET,
	 	    	                      pct, GPIO_PIN_SET,
	 	    	                      500);
	 	    	}

	 	    	HAL_Delay(1000);

	 	    	// [7] 제자리 회전(시계) : 좌 전진 / 우 후진
	 	    	for (int pct = 0; pct <= 30; pct += 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_SET,     // 좌 전진
	 	    	                      pct, GPIO_PIN_RESET,   // 우 후진
	 	    	                      500);
	 	    	}
	 	    	driveDualMotorPct(50, GPIO_PIN_SET,
	 	    	                  50, GPIO_PIN_RESET,
	 	    	                  5000);
	 	    	for (int pct = 40; pct >= 0; pct -= 10) {
	 	    	    driveDualMotorPct(pct, GPIO_PIN_SET,
	 	    	                      pct, GPIO_PIN_RESET,
	 	    	                      250);
	 	    	}

	 	    	// [8] 완전 정지
	 	    	driveDualMotorPct(0, GPIO_PIN_SET,
	 	    	                  0, GPIO_PIN_SET,
	 	    	                  1000);
 */
	 	    	// [9] 원(반시계) (좌 느린 후진 / 우 빠른 전진)
	 	    	for (int left = 20; left <= 30; left += 2) {
	 	    	    for (int i = 0; i < 5; i++) {
	 	    	        driveDualMotorPct(left, GPIO_PIN_SET,   // 좌 전진 left%
	 	    	                          35,   GPIO_PIN_SET,   // 우 전진 35%
	 	    	                          3000);
	 	    	    }
	 	    	    driveDualMotorPct(0, GPIO_PIN_SET, 0, GPIO_PIN_SET, 500); // 정지 유지
	 	    	}

	 	    	// [10] 정지 유지
	 	    	driveDualMotorPct(0, GPIO_PIN_SET,
	 	    	                  0, GPIO_PIN_SET,
	 	    	                  1000);

	 	     }
	 	      else
	 	      {
	 	          HAL_Delay(100);
	 	      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2099;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR2_Pin|DIR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_EXTI13_Pin */
  GPIO_InitStruct.Pin = USER_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR4_Pin */
  GPIO_InitStruct.Pin = DIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2_Pin DIR3_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin|DIR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR1_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandler)
 {
 HAL_UART_Transmit(UartHandler, (uint8_t*)TxBuffer_10, TxBufferSize_10 , 0xFFFF);
 }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // ✅ 항상 제일 먼저 다음 수신 예약
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);

        // ✅ 오버런 오류 확인
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"USART2 ORE ERROR\r\n", 19, 100);
            __HAL_UART_CLEAR_OREFLAG(&huart2);
        }

        // 수신 버퍼에 저장
        if (uart2_index < sizeof(uart2_buffer) - 1)
        {
            uart2_buffer[uart2_index++] = uart2_rx_byte;
        }

        // 개행 문자 수신 시 파싱 시도
        if (uart2_rx_byte == '\r' || uart2_rx_byte == '\n')
        {
            uart2_buffer[uart2_index] = '\0';

            int d1, p1, d2, p2, clean_flag;
            if (sscanf((char*)uart2_buffer, "%d,%d,%d,%d,%d", &d1, &p1, &d2, &p2, &clean_flag) == 5)
            {
                // 0~500 → 0~100%
                if (p1 < 0) p1 = 0; if (p1 > 500) p1 = 500;
                if (p2 < 0) p2 = 0; if (p2 > 500) p2 = 500;

                int pctL = (p1 * 100) / 500;
                int pctR = (p2 * 100) / 500;

                set_motor_targets(
                    pctL, (d1 ? GPIO_PIN_SET : GPIO_PIN_RESET),
                    pctR, (d2 ? GPIO_PIN_SET : GPIO_PIN_RESET)
                );

                // ✅ 변경된 규칙: 1이면 토글 요청, 0이면 무반응
                if (clean_flag == 1) {
                    cleaning_toggle_req = true;   // 실제 하드웨어 조작은 메인 루프에서
                }

                // 디버그 (원하면 유지)
                char debug[96];
                snprintf(debug, sizeof(debug),
                         "UART2 OK d1=%d p1=%d(%d%%)  d2=%d p2=%d(%d%%)  clean=%d\r\n",
                         d1, p1, pctL, d2, p2, pctR, clean_flag);
                HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 100);
            }
            else {
                HAL_UART_Transmit(&huart3, (uint8_t*)"UART2 Parse Failed\r\n", 20, 100);
            }

            uart2_index = 0;
        }
    }

    // UART3 수신 처리 (그대로 유지)
    if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, RxBuffer, 1);

        char ch = (char)RxBuffer[0];
        if (ch == '\r') return;
        else if (ch == '\n')
        {
            uart_buffer[uart_index] = '\0';

            // 1) 라즈베리파이에게 단순 ACK (USART2로 회신)
                    const char ack[] = "ACK\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t*)ack, sizeof(ack)-1, 100);

                    // 2) PC(TeraTerm)에서도 수신 내용 보이도록 브리지(USART3로 복사)
                    //    예: "U2> <원문>" 형태로 찍기
                    char line[96];
                    int n = snprintf(line, sizeof(line), "U2> %s\r\n", uart2_buffer);
                    if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t*)line, (uint16_t)n, 100);

            int dir = 0, speed = 0;
            if (sscanf(uart_buffer, "%d %d", &dir, &speed) == 2)
            {
                if (speed < 0) speed = 0;
                if (speed > 100) speed = 100; // 퍼센트

                GPIO_PinState d = dir ? GPIO_PIN_SET : GPIO_PIN_RESET;

                // ★ 슬루/데드밴드 경유: 목표만 기록
                set_motor_targets(speed, d, speed, d);

                char response[64];
                snprintf(response, sizeof(response), "Parsed: dir=%d speed=%d\r\n", dir, speed);
                HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), 100);
            }
            else
            {
                HAL_UART_Transmit(&huart3, (uint8_t*)"Parse failed\r\n", 14, 100);
            }

            uart_index = 0;
        }
        else
        {
            if (uart_index < sizeof(uart_buffer) - 1)
                uart_buffer[uart_index++] = ch;
        }
    }
}

void sequential_leds(void)
{
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void driveDualMotor(uint16_t left_duty, GPIO_PinState left_dir,
                    uint16_t right_duty, GPIO_PinState right_dir,
                    uint32_t duration_ms)
{
    // 방향 설정
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, left_dir);   // 왼쪽 모터 DIR
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, right_dir);  // 오른쪽 모터 DIR

    // PWM 속도 설정
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_duty);   // PA0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_duty);  // PB11

    // duration_ms 동안 반복 체크
    uint32_t elapsed = 0;
        while (elapsed < duration_ms)
        {
            if (stop_requested)
            {
                // 평균 듀티를 사용하여 감속
                uint16_t avg_duty = (left_duty + right_duty) / 2;
                slowStopMotor(avg_duty, left_dir);  // 양쪽 같은 방향이라 가정
                break;
            }

            HAL_Delay(10);
            elapsed += 10;
    }

    // 정지 (브레이크 X, coast)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}
void driveBrushMotor(GPIO_PinState dir, uint16_t duty)
{
    HAL_GPIO_WritePin(BRUSH_DIR_PORT, BRUSH_DIR_PIN, dir);
    __HAL_TIM_SET_COMPARE(&htim3, BRUSH_PWM_CHANNEL, duty);
}
// 브러시/진공 "토글" (on <-> off), 쿨다운 적용
static void toggleCleaningSystem(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_clean_toggle_ms < CLEAN_TOGGLE_COOLDOWN_MS) {
        return; // 너무 짧은 간격의 중복 토글 방지
    }

    if (!vacuum_is_on) {
        // ON
        driveBrushMotor(GPIO_PIN_SET, 500);  // 브러시 PWM = 500
        pulse_requested = true;              // 진공 릴레이 "딱" ON
        vacuum_is_on = true;
    } else {
        // OFF
        driveBrushMotor(GPIO_PIN_RESET, 0);  // 브러시 OFF
        pulse_requested = true;              // 진공 릴레이 "딱" OFF
        vacuum_is_on = false;
    }
    last_clean_toggle_ms = now;
}
void blink_all_leds_twice(void)
{
  for (int i = 0; i < 2; i++)
  {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    HAL_Delay(300);

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);
  }
}
// Slowly ramp down the motor until it stops
void slowStopMotor(uint16_t current_duty, GPIO_PinState dir)
{
    for (int duty = current_duty; duty >= 0; duty -= 50)
    {
        // 좌우 모터 모두 감속
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty); // 왼쪽 (PA0)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty); // 오른쪽 (PB11)
        HAL_Delay(100);
    }

    // PWM 완전 중단
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // 왼쪽
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);  // 오른쪽

    // DIR 핀도 OFF로
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // 왼쪽 DIR
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // 오른쪽 DIR
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
	    motor_enabled = !motor_enabled;

	    if (!motor_enabled)
	    {
	        stop_requested = true;
	        extra_motor_on = false;

	        driveBrushMotor(GPIO_PIN_RESET, 0);
	    }
	    else
	    {
	        stop_requested = false;
	        extra_motor_on = true;

	        driveBrushMotor(GPIO_PIN_SET, 500);     // 약한 듀티
	    }
	    // ✅ 펄스 요청
	    pulse_requested = true;
	    HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // 디버깅 LED
	}
}

// 10ms 주기(TIM6)마다 호출되어, 슬루/데드밴드 적용 후 CCR 반영
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    slew_step();
    apply_to_timer_ccr_from_current();
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
