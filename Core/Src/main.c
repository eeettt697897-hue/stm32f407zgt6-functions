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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_task.h"
#include "dji_motor.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//灯光部分
typedef enum {
  MODE_OFF = 0,
  MODE_ON,
  MODE_BLINK,
  MODE_BREATHE
} LightMode_t;

volatile LightMode_t current_mode = MODE_OFF;

volatile uint8_t  target_brightness = 128;
volatile uint16_t target_freq = 1;

static uint8_t rx_char = 0;
static char    rx_buf[64];
static uint8_t rx_idx = 0;

static uint32_t last_tick_ms = 0;
static uint8_t  blink_state = 0;

static int8_t   breathe_dir = 1;
static float    breathe_val = 0.0f;

//测量部份
#define ADC_FS_HZ   200000u
#define ADC_BUF_LEN 2048u

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

static uint16_t adc_buf[ADC_BUF_LEN];
static volatile uint8_t adc_half_ready = 0;
static volatile uint8_t adc_full_ready = 0;

static volatile uint32_t freq_hz = 0;

//电机部分
static MotorCtrl_t g_motor;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//灯光部分
static void UART_Send(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static void rtrim(char *s)
{
  size_t n = strlen(s);
  while (n > 0) {
    char c = s[n - 1];
    if (c == ' ' || c == '\t') {
      s[n - 1] = '\0';
      n--;
    } else {
      break;
    }
  }
}

static uint16_t clamp_u16(int v, int lo, int hi)
{
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static uint8_t clamp_u8(int v, int lo, int hi)
{
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}

static void PWM_Set(uint8_t b)
{
  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, b);
}

static void parse_command(char *cmd)
{
  rtrim(cmd);

  if (strcmp(cmd, "set_mode on") == 0) {
    current_mode = MODE_ON;
    UART_Send("OK: mode=ON\r\n");
    return;
  }

  if (strcmp(cmd, "set_mode off") == 0) {
    current_mode = MODE_OFF;
    UART_Send("OK: mode=OFF\r\n");
    return;
  }

  if (strncmp(cmd, "set_mode blink", 14) == 0) {
    int f = atoi(cmd + 15);
    target_freq = clamp_u16(f, 1, 20);
    current_mode = MODE_BLINK;
    UART_Send("OK: mode=BLINK\r\n");
    return;
  }

  if (strncmp(cmd, "set_mode breathe", 16) == 0) {
    int f = atoi(cmd + 17);
    target_freq = clamp_u16(f, 1, 20);
    current_mode = MODE_BREATHE;
    UART_Send("OK: mode=BREATHE\r\n");
    return;
  }

  if (strncmp(cmd, "set_brightness", 14) == 0) {
    int b = atoi(cmd + 15);
    target_brightness = clamp_u8(b, 0, 255);
    UART_Send("OK: brightness updated\r\n");
    return;
  }



  //电机部分
  if (strncmp(cmd, "set_vel", 7) == 0) {
      int v = atoi(cmd + 8);
      MotorTask_SetVel(&g_motor, v);
      UART_Send("OK: motor set_vel\r\n");
      return;
  }

  if (strncmp(cmd, "set_pos", 7) == 0) {
      int p = atoi(cmd + 8);
      MotorTask_SetPos(&g_motor, p);
      UART_Send("OK: motor set_pos\r\n");
      return;
  }
  UART_Send("ERR: unknown command\r\n");

}

//测量部份
static uint32_t estimate_freq_hz(const uint16_t *buf, uint32_t len)
{
  uint16_t mn = 0xFFFF, mx = 0;
  for (uint32_t i = 0; i < len; i++) {
    uint16_t v = buf[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  uint16_t thr = (uint16_t)((mn + mx) / 2u);

  uint32_t edges[64];
  uint32_t ecnt = 0;

  uint16_t prev = buf[0];
  for (uint32_t i = 1; i < len; i++) {
    uint16_t cur = buf[i];
    if (prev < thr && cur > thr) {
      if (ecnt < 64) edges[ecnt++] = i;
    }
    prev = cur;
  }

  if (ecnt < 2) return 0;

  uint32_t sum = 0, cnt = 0;
  for (uint32_t k = 1; k < ecnt; k++) {
    uint32_t d = edges[k] - edges[k-1];
    if (d >= 10 && d <= 1000) { sum += d; cnt++; }
  }
  if (cnt == 0) return 0;

  uint32_t avg = sum / cnt;
  return (uint32_t)(ADC_FS_HZ / avg);
}

static void uart_print_freq(uint32_t f)
{
  char msg[64];
  int n = snprintf(msg, sizeof(msg), "MEAS: %lu Hz\r\n", (unsigned long)f);
  if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 200);
}

//数字转模拟 报文
#define ADC_MAX_CODE 4095u
#define ADC_VREF_MV  3300u

typedef struct {
  uint32_t freq_hz;
  uint16_t vmin_mv;
  uint16_t vmax_mv;
  uint16_t vpp_mv;
  uint16_t vavg_mv;
} MeasResult_t;

static void measure_voltage_and_freq(const uint16_t *buf, uint32_t len, MeasResult_t *out)
{
  uint32_t sum = 0;
  uint16_t mn = 0xFFFF, mx = 0;

  for (uint32_t i = 0; i < len; i++) {
    uint16_t v = buf[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }

  uint32_t avg_code = (len > 0) ? (sum / len) : 0;

  uint32_t vmin_mv = (uint32_t)mn * ADC_VREF_MV / ADC_MAX_CODE;
  uint32_t vmax_mv = (uint32_t)mx * ADC_VREF_MV / ADC_MAX_CODE;
  uint32_t vavg_mv = (uint32_t)avg_code * ADC_VREF_MV / ADC_MAX_CODE;

  out->vmin_mv = (uint16_t)vmin_mv;
  out->vmax_mv = (uint16_t)vmax_mv;
  out->vpp_mv  = (uint16_t)(vmax_mv - vmin_mv);
  out->vavg_mv = (uint16_t)vavg_mv;

  out->freq_hz = estimate_freq_hz(buf, len);
}


static void uart_send_meas_frame(const MeasResult_t *m)
{
  uint8_t frame[17];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = 0x01;
  frame[3] = 12;

  uint32_t f = m->freq_hz;
  frame[4]  = (uint8_t)(f);
  frame[5]  = (uint8_t)(f >> 8);
  frame[6]  = (uint8_t)(f >> 16);
  frame[7]  = (uint8_t)(f >> 24);

  uint16_t v;

  v = m->vmin_mv;
  frame[8]  = (uint8_t)(v);
  frame[9]  = (uint8_t)(v >> 8);

  v = m->vmax_mv;
  frame[10] = (uint8_t)(v);
  frame[11] = (uint8_t)(v >> 8);

  v = m->vpp_mv;
  frame[12] = (uint8_t)(v);
  frame[13] = (uint8_t)(v >> 8);

  v = m->vavg_mv;
  frame[14] = (uint8_t)(v);
  frame[15] = (uint8_t)(v >> 8);

  uint8_t cs = 0;
  for (int i = 2; i <= 15; i++) cs += frame[i];
  frame[16] = cs;

  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 200);
}

static void uart_print_meas_text(const MeasResult_t *m)
{
  char msg[96];
  int n = snprintf(msg, sizeof(msg),
                   "MEAS: %lu Hz, Vmin=%umV, Vmax=%umV, Vpp=%umV, Vavg=%umV\r\n",
                   (unsigned long)m->freq_hz,
                   (unsigned)m->vmin_mv,
                   (unsigned)m->vmax_mv,
                   (unsigned)m->vpp_mv,
                   (unsigned)m->vavg_mv);
  if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)strlen(msg), 200);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  //灯光部分
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart1, &rx_char, 1);

  UART_Send("System Boot OK. Awaiting commands...\r\n");

  PWM_Set(0);
  current_mode = MODE_OFF;

  last_tick_ms = HAL_GetTick();
  blink_state = 0;
  breathe_dir = 1;
  breathe_val = 0.0f;


  //测量部份
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  //电机部分
  MotorTask_Init(&g_motor);

  HAL_TIM_Base_Start_IT(&htim7);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  UART_Send("Motor module ready. Commands: set_vel / set_pos\r\n");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //灯光部分
	  uint32_t now = HAL_GetTick();

	  switch (current_mode)
	  {
	    case MODE_OFF:
	      PWM_Set(0);
	      break;

	    case MODE_ON:
	      PWM_Set((uint8_t)target_brightness);
	      break;

	    case MODE_BLINK:
	    {
	      uint16_t f = target_freq;
	      uint32_t half_ms = 500u / (uint32_t)f;

	      if ((now - last_tick_ms) >= half_ms) {
	        blink_state = (uint8_t)!blink_state;
	        PWM_Set(blink_state ? (uint8_t)target_brightness : 0);
	        last_tick_ms = now;
	      }
	      break;
	    }

	    case MODE_BREATHE:
	    {
	      if ((now - last_tick_ms) >= 10u) {
	        uint16_t f = target_freq;
	        float peak = (float)target_brightness;

	        float step = peak * ((float)f) / 50.0f;

	        breathe_val += (float)breathe_dir * step;

	        if (breathe_val >= peak) { breathe_val = peak; breathe_dir = -1; }
	        if (breathe_val <= 0.0f) { breathe_val = 0.0f; breathe_dir =  1; }

	        PWM_Set((uint8_t)(breathe_val + 0.5f));
	        last_tick_ms = now;
	      }
	      break;
	    }

	    default:
	      PWM_Set(0);
	      current_mode = MODE_OFF;
	      break;
	  }

	  //测量部份
	  /* ===== 测量：半缓冲/满缓冲处理 ===== */
	  if (adc_half_ready) {
	    adc_half_ready = 0;

	    MeasResult_t m;
	    measure_voltage_and_freq(&adc_buf[0], ADC_BUF_LEN/2, &m);
	    uart_send_meas_frame(&m);

	    uart_print_meas_text(&m);
	  }

	  if (adc_full_ready) {
	    adc_full_ready = 0;

	    MeasResult_t m;
	    measure_voltage_and_freq(&adc_buf[ADC_BUF_LEN/2], ADC_BUF_LEN/2, &m);

	    uart_send_meas_frame(&m);
	    uart_print_meas_text(&m);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 167;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 255;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//灯光部分
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {

    if (rx_char == '\r' || rx_char == '\n') {
      rx_buf[rx_idx] = '\0';
      if (rx_idx > 0) {
        parse_command(rx_buf);
      }
      rx_idx = 0;
    } else {
      if (rx_idx < (sizeof(rx_buf) - 1)) {
        rx_buf[rx_idx++] = (char)rx_char;
      } else {
        rx_idx = 0;
        UART_Send("ERR: line too long\r\n");
      }
    }

    HAL_UART_Receive_IT(&huart1, &rx_char, 1);
  }
}

//测量部份
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) adc_half_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) adc_full_ready = 1;
}


//电机部分
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        MotorTask_1ms(&g_motor);
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
#ifdef USE_FULL_ASSERT
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
