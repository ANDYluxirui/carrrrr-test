/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 两轮差速小车 —— 编码器 + PID 速度环
  *
  *   硬件连接（TB6612）：
  *     左轮  AIN1=PB0  AIN2=PB1  PWMA=PA6(TIM3_CH1)
  *     右轮  BIN1=PB10 BIN2=PB11 PWMB=PA7(TIM3_CH2)
  *   编码器（AB 相，四倍频）：
  *     左轮  A=PA0  B=PA1   (TIM2 编码器模式)
  *     右轮  A=PB6  B=PB7   (TIM4 编码器模式)
  *   串口（遥测）：
  *     TX=PA9  RX=PA10  115200 8N1
  *
  *   ★ 本文件所有外设都在代码里自己初始化，不依赖 CubeMX 生成，
  *     所以不管是否重新 Generate Code 都能编译通过。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "chassis.h"
#include "mpu6050.h"
#include "oled.h"
#include <string.h>
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;      /* 左编码器 PA0/PA1 */
TIM_HandleTypeDef htim4;      /* 右编码器 PB6/PB7 */
UART_HandleTypeDef huart_dbg; /* 遥测串口 PA9/PA10（独立句柄，避免与 CubeMX 的 huart1 冲突）*/

static PID_t    pid_l, pid_r;
static float    g_target = 0.0f;
static uint32_t t_ctrl = 0, t_tel = 0, t_boot = 0, t_odo = 0, t_ui = 0;
static uint8_t  g_oled_ok = 0;
static uint8_t  s_ui_page = 0;
#if (DEMO_MODE == 3) || (DEMO_MODE == 4)
static uint8_t  s_mission_started = 0;
#endif

/* ---- 极简串口输出（不依赖 printf，避开 newlib-nano 的浮点坑）---- */
static void uart_str(const char *s)
{
  HAL_UART_Transmit(&huart_dbg, (uint8_t *)s, (uint16_t)strlen(s), 50);
}

static void uart_int(int32_t v)
{
  char buf[12];
  int  i = 0, neg = 0;
  if (v < 0) { neg = 1; v = -v; }
  if (v == 0) buf[i++] = '0';
  while (v > 0) { buf[i++] = (char)('0' + v % 10); v /= 10; }
  if (neg) buf[i++] = '-';
  while (i > 0) HAL_UART_Transmit(&huart_dbg, (uint8_t *)&buf[--i], 1, 50);
}

/* 一行 CSV：
 *   t,target,vL,vR,uL,uR, cntL,cntR, PA0,PA1,PB6,PB7
 *   前 4 列给 plot_step.py 用；后 6 列是编码器诊断，不看可忽略。
 *   cntL/cntR : TIM2/TIM4 原始计数（手转轮子应该变化）
 *   PA0,PA1   : 左编码器 A/B 相电平
 *   PB6,PB7   : 右编码器 A/B 相电平
 *   手转轮子时这四个电平应该在 0/1 之间变化。 */
static void telemetry(uint32_t ms)
{
  uart_int((int32_t)ms);        uart_str(",");
  uart_int((int32_t)g_target);  uart_str(",");
  uart_int((int32_t)g_speed_l); uart_str(",");
  uart_int((int32_t)g_speed_r); uart_str(",");
  uart_int(g_duty_l);           uart_str(",");
  uart_int(g_duty_r);           uart_str(",");
  uart_int((int32_t)g_pose.x);  uart_str(",");
  uart_int((int32_t)g_pose.y);  uart_str(",");
  uart_int((int32_t)(g_pose.theta * 57.2958f)); uart_str(",");
  uart_int((int32_t)__HAL_TIM_GET_COUNTER(&htim2)); uart_str(",");
  uart_int((int32_t)__HAL_TIM_GET_COUNTER(&htim4)); uart_str(",");
  uart_int(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));    uart_str(",");
  uart_int(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));    uart_str(",");
  uart_int(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6));    uart_str(",");
  uart_int(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));    uart_str("\r\n");
}

/* OLED 仪表：每次只重画一页并刷新，避免长时间占用总线 */
static void ui_update(void)
{
  if (!g_oled_ok) return;

  switch (s_ui_page) {
  case 0:
    OLED_Str(0, 0, "X:");
    OLED_Int(0, 2, (int32_t)g_pose.x, 6);
    OLED_Str(0, 9, "mm");
    break;
  case 1:
    OLED_Str(1, 0, "Y:");
    OLED_Int(1, 2, (int32_t)g_pose.y, 6);
    OLED_Str(1, 9, "mm");
    break;
  case 2:
    OLED_Str(2, 0, "TH:");
    OLED_Int(2, 3, (int32_t)(g_pose.theta * 57.2958f), 5);
    OLED_Str(2, 9, "deg");
    break;
  case 3:
    OLED_Str(3, 0, "SPD");
    OLED_Int(3, 3, (int32_t)g_speed_l, 5);
    OLED_Int(3, 9, (int32_t)g_speed_r, 5);
    break;
  case 4:
    OLED_Str(4, 0, "TGT");
    OLED_Int(4, 3, (int32_t)g_target_l, 5);
    OLED_Int(4, 9, (int32_t)g_target_r, 5);
    break;
  case 5:
    OLED_Str(5, 0, g_mpu_ok ? "MPU:GYRO" : "MPU:ENC ");
    OLED_Str(5, 9, g_mpu_ok ? " OK" : " --");
    break;
  }
  OLED_FlushPage(s_ui_page);
  s_ui_page = (uint8_t)((s_ui_page + 1) % 6);
}

/* 演示模式：产生目标速度 */
static float demo_target(uint32_t ms)
{
#if DEMO_MODE == 0
  (void)ms;
  return DEMO_SPEED_MM_S;                       /* 恒速：看稳态误差和抖动 */

#elif DEMO_MODE == 1
  /* 阶跃：上电先静止 2s，再给目标，保持 DEMO_STEP_HOLD_S 秒后归零，循环 */
  uint32_t el = ms - t_boot;
  uint32_t hold_ms = (uint32_t)(DEMO_STEP_HOLD_S * 1000.0f);
  uint32_t phase;
  if (el < 2000u) return 0.0f;
  phase = (el - 2000u) / hold_ms;
  return (phase % 2u == 0u) ? DEMO_SPEED_MM_S : 0.0f;

#elif DEMO_MODE == 2
  return ((ms / 2000u) % 2u) ? DEMO_SPEED_MM_S : 0.0f;   /* 方波 */

#else
  (void)ms;
  return 0.0f;
#endif
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Encoder_Init(void);
static void MX_TIM4_Encoder_Init(void);
static void MX_USART_Dbg_Init(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM2_Encoder_Init();
  MX_TIM4_Encoder_Init();
  MX_USART_Dbg_Init();

  Motor_Init();
  Encoder_Init();

  /* 陀螺仪：检测不到也不影响，底盘会自动退化成纯编码器航向 */
  if (MPU6050_Init())
  {
    HAL_Delay(300);
    MPU6050_Calibrate();      /* 上电静止标定零偏，约 1 秒，此间别动车 */
  }
  Chassis_Init();

  OLED_Init();
  g_oled_ok = OLED_IsOk();
  if (g_oled_ok) {
    OLED_Str(0, 0, "PID CAR  v1.0");
    OLED_Str(1, 0, "boot...");
    OLED_Flush();
  }

  /* 开机自检：一眼看出陀螺仪有没有识别到 */
  if (g_mpu_ok) {
    uart_str("# MPU6050 OK, gyro bias = ");
    uart_int((int32_t)(g_gyro_bias * 10.0f));
    uart_str(" (x0.1 LSB)\r\n");
  } else {
    uart_str("# MPU6050 NOT FOUND -> 仅用编码器推算航向\r\n");
  }

  PID_Init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD,
           CTRL_DT_S, (float)(-PWM_ARR), (float)PWM_ARR);
  PID_Init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD,
           CTRL_DT_S, (float)(-PWM_ARR), (float)PWM_ARR);

  t_boot = HAL_GetTick();
  t_ctrl = t_boot;
  t_tel  = t_boot;

  HAL_Delay(20);
  uart_str("\r\n# t,target,vL,vR,uL,uR\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ---------------- 50Hz 里程计与定位任务 ---------------- */
    if (now - t_odo >= ODO_PERIOD_MS)
    {
      t_odo = now;

      Chassis_Update();                       /* 里程计 + 航向融合 */

#if DEMO_MODE == 3
      if (!s_mission_started && (now - t_boot) > 2000u) {
        s_mission_started = 1;
        Chassis_StartSquare(MISSION_SIDE_MM); /* 正方形一圈 */
      }
      Chassis_MissionUpdate();
#elif DEMO_MODE == 4
      if (!s_mission_started && (now - t_boot) > 2000u) {
        s_mission_started = 1;
        Chassis_StartStraight(1000.0f);       /* 定距直行 1m */
      }
      Chassis_MissionUpdate();
#endif
    }

    /* ---------------- 100Hz 速度环 ---------------- */
    if (now - t_ctrl >= CTRL_PERIOD_MS)
    {
      t_ctrl = now;

      Encoder_Update();                       /* 1. 读编码器 */

#if (DEMO_MODE == 3) || (DEMO_MODE == 4)
      /* 任务模式：左右轮目标由底盘运动学给出 */
      g_target = 0.5f * (g_target_l + g_target_r);
#else
      g_target = demo_target(now);            /* 2. 目标速度 */
      g_target_l = g_target;
      g_target_r = g_target;
#endif

      if (g_target_l == 0.0f && g_target_r == 0.0f)
      {
        /* 目标为 0 = 停机。
         * 增量式 PID 在误差为 0 时输出不变，会停在最后一次的值上，
         * 所以这里必须显式清零，否则车停不下来。 */
        PID_Reset(&pid_l, 0.0f);
        PID_Reset(&pid_r, 0.0f);
        Motor_Stop();
      }
      else
      {
        float u_l = PID_Update(&pid_l, g_target_l, g_speed_l);  /* 3. PID */
        float u_r = PID_Update(&pid_r, g_target_r, g_speed_r);

        Motor_SetLeft((int32_t)u_l);                            /* 4. 下发 */
        Motor_SetRight((int32_t)u_r);
      }
    }

    /* ---------------- 10Hz OLED 刷新（每次一页）---------------- */
    if (now - t_ui >= 100u)
    {
      t_ui = now;
      ui_update();
    }

    /* ---------------- 20Hz 遥测 ---------------- */
    if (now - t_tel >= 50u)
    {
      t_tel = now;
      telemetry(now - t_boot);
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
}

/**
  * @brief TIM3 Initialization —— PWM 20kHz
  *   72MHz / (0+1) / (3599+1) = 20kHz
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
  * @brief TIM2 编码器模式（左轮 PA0/PA1）
  */
static void MX_TIM2_Encoder_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 12;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 12;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 编码器模式（右轮 PB6/PB7）
  */
static void MX_TIM4_Encoder_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 12;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 12;

  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 初始化（遥测 115200 8N1，PA9=TX / PA10=RX）
  * @note  用独立句柄 huart_dbg，即使 CubeMX 另外生成了 huart1 也不冲突
  */
static void MX_USART_Dbg_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart_dbg.Instance = USART1;
  huart_dbg.Init.BaudRate = 115200;
  huart_dbg.Init.WordLength = UART_WORDLENGTH_8B;
  huart_dbg.Init.StopBits = UART_STOPBITS_1;
  huart_dbg.Init.Parity = UART_PARITY_NONE;
  huart_dbg.Init.Mode = UART_MODE_TX_RX;
  huart_dbg.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_dbg.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_dbg) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
