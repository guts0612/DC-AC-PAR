/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "arm_math.h"
#include "qpr_control.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DIV1 下，比较值避开计数端点；6 tick 是按 RM0440 p918 例值取的保守余量。 */
#define HRTIM_COMPARE_MARGIN_TICKS  6U
#define SPWM_MODULATION_LIMIT      QPR_MODULATION_LIMIT
#define SPWM_OUTPUT_FREQUENCY_HZ    QPR_FREQUENCY_HZ
/* dcac 的差分 ADC、HRTIM 触发与 UART4 配置。 */
#define ADC_DIFF_ZERO_CODE         2048
#define VOFA_OUTPUT_ENABLE         1U
/* 六通道：20 kHz / 10 = 2000 帧/s，50 Hz 正弦每周期 40 点。
 * 28 字节、8N1 占用 560000 bit/s；UART4 和 VOFA 均设置为 921600 baud。
 */
#define VOFA_SAMPLE_DIV            10U
/* 1: 400 个全速控制样本的有符号均值；0: 原六通道瞬时波形。 */
#define VOFA_MEAN_OUTPUT_ENABLE    0U
#define VOFA_MEAN_SAMPLE_COUNT     400U
#define VOFA_UART_TIMEOUT_MS       5U

#define VOLTAGE_ZERO_V (1.420631941e-01f)
#define CURRENT_ZERO_V (-1.768831080e-03f)

#if VOFA_SAMPLE_DIV < 1U
#error "VOFA_SAMPLE_DIV must be at least 1"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Period;
/* Keil Watch：目标峰值约 45.2548 V，对应 32 Vrms；本板暂未配置母线 ADC。 */
/* 已确认 25 ohm 带载工况；默认启动闭环，参考峰值约用 0.754 s 升到 45.2548 V。 */
/* 1：QPR 闭环；0：正弦开环输出，不是停机/关断栅极。 */
volatile uint8_t voltage_closed_loop_enable = 0U;
volatile float voltage_reference_peak_v = QPR_DEFAULT_REFERENCE_PEAK_V;
volatile float dc_bus_voltage_v = QPR_DEFAULT_DC_BUS_V;
volatile float voltage_reference_v;
/* 展开 qpr_control 可查看两环误差、校正电压、限幅次数和故障位。 */
QPR_Control qpr_control;
/* 系数在主循环准备，ADC 控制拍边界接管，避免并发改写和 ISR 内 double 计算。 */
static QPR_Control qpr_reinit_prepared;
static volatile uint8_t qpr_reinit_pending;
volatile uint32_t qpr_reinit_count;
volatile float modulation_command;
volatile uint32_t spwm_update_count;
float spwm_update_hz;
static float spwm_phase_rad;
static float phase_step_rad;
/* DMA 每次把 ADC1/ADC2 的 CDR 写入同一个 32 位字；仅在回调开头读一次。 */
volatile float vref_voltage = 3.0f;      /* 外部 VREF+，单位 V */
volatile uint32_t adc12_word;
volatile uint16_t adc_voltage_raw;
volatile uint16_t adc_current_raw;
volatile int16_t adc_voltage_code;
volatile int16_t adc_current_code;
volatile float adc_sample_voltage;       /* 电压采样通道的 ADC 差分电压，V */
volatile float adc_sample_current;       /* 电流采样通道的 ADC 差分电压，V，尚未换算为 A */
volatile float true_voltage;             /* 瞬时电压，V */
volatile float true_current;             /* 滤波电感瞬时电流，A（电容分流之前） */
volatile float voltage_gain = 17.7f;       /* 电压采样通道的增益，V/V */
volatile float cksr6_current_gain = 2.400384f;   /* 电流采样通道的增益，A/V */
volatile uint32_t adc_sample_count;      /* ADC1 DMA 完成回调次数 */


#if VOFA_OUTPUT_ENABLE
/* 单槽邮箱：保留同一窗口均值或同一控制拍的六个量；主循环发送。 */
volatile float vofa_ch1;
volatile float vofa_ch2;
volatile float vofa_ch3;
volatile float vofa_ch4;
volatile float vofa_ch5;
volatile float vofa_ch6;
/* 均值窗口只由 ADC ISR 访问，单精度累加使用 M4F 硬件浮点。 */
static float vofa_mean_sum[5];
static uint32_t vofa_mean_count;
static uint8_t vofa_mean_mode;
volatile uint32_t vofa_mean_window_count;
volatile uint8_t vofa_frame_ready;
volatile uint32_t adc_sample_div_count;
volatile uint32_t vofa_sent_count;        /* HAL 返回成功的完整帧数 */
volatile uint32_t vofa_tx_error_count;    /* HAL 超时、忙或错误的累计次数 */
volatile uint32_t vofa_overwrite_count;   /* 主循环未取走就被新采样覆盖的帧数 */
static uint8_t vofa_frame[28];  /* 六个 float（24 字节）+ JustFloat 帧尾（4 字节）。 */
typedef char VofaRequires32BitFloat[(sizeof(float) == 4U) ? 1 : -1];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 对应 singlephase/单极倍频调制：
 * dA=(1+m)/2，dB=(1-m)/2。
 * HRTIM 使用 EQUAL + Reset CMP1：上数清零、下数置位，与 TIM1 PWM1 一致。
 * A/B 的 ROM_VALLEY + REP=0 使两路预装载值在每个谷底一起生效。
 */
static void HRTIM_SetModulation(float modulation)
{
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t maximum = (uint32_t)Period - HRTIM_COMPARE_MARGIN_TICKS;

    if (modulation > SPWM_MODULATION_LIMIT) modulation = SPWM_MODULATION_LIMIT;
    if (modulation < -SPWM_MODULATION_LIMIT) modulation = -SPWM_MODULATION_LIMIT;

    compare_a = (uint32_t)(0.5f * (1.0f + modulation) * (float)Period + 0.5f);
    compare_b = (uint32_t)(0.5f * (1.0f - modulation) * (float)Period + 0.5f);
    if (compare_a < HRTIM_COMPARE_MARGIN_TICKS) compare_a = HRTIM_COMPARE_MARGIN_TICKS;
    if (compare_b < HRTIM_COMPARE_MARGIN_TICKS) compare_b = HRTIM_COMPARE_MARGIN_TICKS;
    if (compare_a > maximum) compare_a = maximum;
    if (compare_b > maximum) compare_b = maximum;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, compare_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, compare_b);
    modulation_command = modulation;
}

static void HRTIM_SPWM_Init(void)
{
    uint32_t master_period;
    uint32_t hrtim_clock_hz;

    Period = (uint16_t)__HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
    master_period = __HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
    /* 单更新：M/A/B 同为 DIV1、REP=0；Master 向上计数，PER 必须是 A/B 的两倍。 */
    if ((Period <= 2U * HRTIM_COMPARE_MARGIN_TICKS) ||
        (master_period != 2U * (uint32_t)Period) ||
        (__HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B) != Period) ||
        ((HRTIM1->sMasterRegs.MCR & HRTIM_MCR_CK_PSC) != HRTIM_PRESCALERRATIO_DIV1) ||
        ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxCR & HRTIM_TIMCR_CK_PSC) != HRTIM_PRESCALERRATIO_DIV1) ||
        ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].TIMxCR & HRTIM_TIMCR_CK_PSC) != HRTIM_PRESCALERRATIO_DIV1) ||
        (HRTIM1->sMasterRegs.MREP != 0U))
    {
        Error_Handler();
    }

    /* HRTIM 内核时钟来源见 RM0440 Figure 17；当前为 170 MHz。 */
    hrtim_clock_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) hrtim_clock_hz *= 2U;
    spwm_update_hz = (float)hrtim_clock_hz / (float)master_period;
    phase_step_rad = 2.0f * PI * SPWM_OUTPUT_FREQUENCY_HZ / spwm_update_hz;
    spwm_phase_rad = 0.0f;
    spwm_update_count = 0U;
    if (QPR_ControlInit(&qpr_control, spwm_update_hz) == 0U)
    {
        Error_Handler();
    }

    /* 两桥臂先装载相同的 50% */
    HRTIM_SetModulation(0.0f);
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_MASTER |
                                HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK)
    {
        Error_Handler();
    }

    /* RM0440 p954：进入 RUN 前预置谷部高电平；x2 由死区单元生成互补。 */
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                       HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                       HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_HRTIM_MASTER_CLEAR_FLAG(&hhrtim1, HRTIM_MASTER_FLAG_MREP);
    if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                    HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2) != HAL_OK)
    {
        Error_Handler();
    }
    /* 刚初始化的 M/A/B 计数器均为 0；HAL 用一次 MCR 写入同时启动三者。
     * 不使用“未开计数器时的软件复位”：RM0440 p895 说明此时复位请求无效。
     */
    if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER |
                                      HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
    {
        Error_Handler();
    }
}
/* 差分原始码以 2048 为零点；先去中点，再减零偏，最后换算物理量。 */
static int16_t ADC_DiffRawToSigned(uint16_t raw)
{
    return (int16_t)((int32_t)(raw & 0x0FFFU) - ADC_DIFF_ZERO_CODE);
}

static void ADC_Sampling_Init(void)
{
    /* 必须在 HRTIM 启动周期触发之前校准并挂好 DMA。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /* 当前 HAL 的此 API 会同时使能主、从 ADC；不单独启动 ADC2 的 DMA。 */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc12_word, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    /* 一个 32 位字没有需要处理的半缓冲，保留 TC/TE，关闭无用的 HT 中断。 */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
}

#if VOFA_OUTPUT_ENABLE
/* 每拍调用；完成窗口时一次发布六个结果。不是 RMS，也不是抽点平均。
 * 固定 400 点在当前 20 kHz/50 Hz 配置下对应一个基波周期。
 */
static void VOFA_AccumulateMean(uint8_t closed_loop)
{
    uint32_t i;
    const float scale = 1.0f / (float)VOFA_MEAN_SAMPLE_COUNT;

    if (closed_loop != vofa_mean_mode)
    {
        for (i = 0U; i < 5U; ++i) vofa_mean_sum[i] = 0.0f;
        vofa_mean_count = 0U;
        vofa_mean_mode = closed_loop;
        vofa_frame_ready = 0U; /* 丢弃尚未取走的旧模式结果。 */
    }
    /* 开环时 control.voltage_error_v 被清零，因此用同拍参考重新求误差。 */
    vofa_mean_sum[0] += qpr_control.reference_v - true_voltage;
    vofa_mean_sum[1] += qpr_control.current_reference_a;
    vofa_mean_sum[2] += true_current;
    vofa_mean_sum[3] += qpr_control.resonant_r;
    vofa_mean_sum[4] += qpr_control.qpr_unlimited_v;
    vofa_mean_count++;
    if (vofa_mean_count < VOFA_MEAN_SAMPLE_COUNT) return;

    if (vofa_frame_ready != 0U) vofa_overwrite_count++;
    vofa_ch1 = vofa_mean_sum[0] * scale; /* 平均电压误差，V */
    vofa_ch2 = vofa_mean_sum[1] * scale; /* 平均电流参考，A */
    vofa_ch3 = vofa_mean_sum[2] * scale; /* 平均实测电流，A */
    vofa_ch4 = vofa_mean_sum[3] * scale; /* 平均谐振项，V */
    vofa_ch5 = vofa_mean_sum[4] * scale; /* 平均未限幅 QPR 输出，V */
    vofa_ch6 = vofa_ch1 - QPR_VIRTUAL_R_OHM * vofa_ch2;
    /* CH6 为虚拟阻抗稳态残差(V)，动态过程可非零；开环不适用此关系。 */
    for (i = 0U; i < 5U; ++i) vofa_mean_sum[i] = 0.0f;
    vofa_mean_count = 0U;
    vofa_mean_window_count++;
    vofa_frame_ready = 1U;
}

static void VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6)
{
    HAL_StatusTypeDef status;

    memcpy(&vofa_frame[0], &ch1, sizeof(ch1));
    memcpy(&vofa_frame[4], &ch2, sizeof(ch2));
    memcpy(&vofa_frame[8], &ch3, sizeof(ch3));
    memcpy(&vofa_frame[12], &ch4, sizeof(ch4));
    memcpy(&vofa_frame[16], &ch5, sizeof(ch5));
    memcpy(&vofa_frame[20], &ch6, sizeof(ch6));
    vofa_frame[24] = 0x00U;
    vofa_frame[25] = 0x00U;
    vofa_frame[26] = 0x80U;
    vofa_frame[27] = 0x7FU;

    /* 主循环轮询发送，等待期间允许 HRTIM/ADC 中断继续运行。
     * 921600 下 28 字节在线时间约 0.304 ms，小于 0.5 ms 采样间隔。
     * 中断抢占会延长实际发送时间；仍保留 5 ms 超时及丢帧计数用于诊断。
     */
    status = HAL_UART_Transmit(&huart4, vofa_frame,
                               (uint16_t)sizeof(vofa_frame), VOFA_UART_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        vofa_sent_count++;
    }
    else
    {
        /* 丢弃失败帧，不阻塞重试；下一帧继续发送最新数据。 */
        vofa_tx_error_count++;
    }
}

static void VOFA_Process(void)
{
    uint32_t primask;
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch5;
    float ch6;

    if (vofa_frame_ready == 0U)
    {
        return;
    }

    /* 一次取走完整六通道快照；保留调用前的中断屏蔽状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    ch1 = vofa_ch1;
    ch2 = vofa_ch2;
    ch3 = vofa_ch3;
    ch4 = vofa_ch4;
    ch5 = vofa_ch5;
    ch6 = vofa_ch6;

    vofa_frame_ready = 0U;
    __set_PRIMASK(primask);

    VOFA_SendJustFloat(ch1, ch2, ch3, ch4, ch5, ch6);
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t key_number;

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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_UART4_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
  Key_Init();
  ADC_Sampling_Init();
  HRTIM_SPWM_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    key_number = Key();
    if (key_number == 1U)
    {
        /* 每次有效松手切换一次；ADC 回调在下一控制拍应用新模式。 */
        voltage_closed_loop_enable = (voltage_closed_loop_enable == 0U) ? 1U : 0U;
    }
    else if ((key_number == 2U) && (qpr_reinit_pending == 0U))
    {
        /* PB9 松手事件：准备完整初始化，下一控制拍开始重新软启动。 */
        if (QPR_ControlInit(&qpr_reinit_prepared, spwm_update_hz) == 0U)
        {
            Error_Handler();
        }
        __DMB();
        qpr_reinit_pending = 1U;
    }
#if VOFA_OUTPUT_ENABLE
    VOFA_Process();
#endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t sample;
    float modulation;
    uint8_t closed_loop;

    if (hadc->Instance != ADC1)
    {
        return;
    }

    /* 一次读取 CDR 快照，避免拆分两通道时跨到下一次 DMA 写入。 */
    sample = adc12_word;
    adc_voltage_raw = (uint16_t)(sample & 0x0FFFU);
    adc_current_raw = (uint16_t)((sample >> 16) & 0x0FFFU);
    adc_voltage_code = ADC_DiffRawToSigned(adc_voltage_raw);
    adc_current_code = ADC_DiffRawToSigned(adc_current_raw);
    adc_sample_voltage = -((adc_voltage_code/2048.0f)* vref_voltage)- VOLTAGE_ZERO_V;
    adc_sample_current = ((adc_current_code/2048.0f)* vref_voltage)- CURRENT_ZERO_V;
    true_voltage =(adc_sample_voltage ) * voltage_gain;
    true_current =(adc_sample_current ) * cksr6_current_gain;
    adc_sample_count++;

    /* Master 向上计数：170 MHz/8500=20 kHz；A/B完整三角波=20 kHz。
     * ISR 写下一拍的预装载值，下一谷底装载；每个50 Hz周期约400次更新。
     */
    if (qpr_reinit_pending != 0U)
    {
        /* 相位及开闭环模式保持，参考幅值和控制状态从零重新开始。 */
        qpr_control = qpr_reinit_prepared;
#if VOFA_OUTPUT_ENABLE
        memset(vofa_mean_sum, 0, sizeof(vofa_mean_sum));
        vofa_mean_count = 0U;
        adc_sample_div_count = 0U;
        vofa_frame_ready = 0U;
#endif
        qpr_reinit_count++;
        __DMB();
        qpr_reinit_pending = 0U;
    }
    closed_loop = (voltage_closed_loop_enable != 0U) ? 1U : 0U;
    if (closed_loop != 0U)
    {
        /* 反馈必须用标定后的物理 V/A，不能用 ADC 端的差分电压。 */
        modulation = QPR_ControlStep(&qpr_control, arm_sin_f32(spwm_phase_rad),
                                     voltage_reference_peak_v, true_voltage,
                                     true_current, dc_bus_voltage_v);
    }
    else
    {
        /* 开环：参考正弦直接换算调制度，不用电压/电流反馈修正。 */
        modulation = QPR_ControlOpenLoopStep(&qpr_control, arm_sin_f32(spwm_phase_rad),
                                             voltage_reference_peak_v, dc_bus_voltage_v);
    }
    if (qpr_control.fault_flags != QPR_FAULT_NONE)
    {
        Error_Handler();
    }
    HRTIM_SetModulation(modulation);
    /* 两种模式均连续推进相位；幅值斜坡由控制函数共用。 */
    spwm_phase_rad += phase_step_rad;
    if (spwm_phase_rad >= 2.0f * PI) spwm_phase_rad -= 2.0f * PI;
    voltage_reference_v = qpr_control.reference_v;
    spwm_update_count++;

    /* ADC ISR 只计算控制和准备遥测；阻塞式串口发送仍在主循环。 */
#if VOFA_OUTPUT_ENABLE
    if (VOFA_MEAN_OUTPUT_ENABLE != 0U)
    {
        VOFA_AccumulateMean(closed_loop);
    }
    else
    {
        adc_sample_div_count++;
        if (adc_sample_div_count >= VOFA_SAMPLE_DIV)
        {
            adc_sample_div_count = 0U;
            if (vofa_frame_ready != 0U)
            {
                vofa_overwrite_count++;
            }
            vofa_ch1 = voltage_reference_v;  /* CH1：参考电压，V */
            vofa_ch2 = true_voltage;         /* CH2：输出电压，V */
            vofa_ch3 =  qpr_control.current_reference_a;  /* CH3：电流参考，A */
            vofa_ch4 =   true_current;         /* CH4：滤波电感电流，A */
            vofa_ch5 = qpr_control.qpr_unlimited_v;  /* CH5：未限幅的 QPR 校正电压，V */
            vofa_ch6 = qpr_control.qpr_correction_v;  /* CH6：限幅后的 QPR 校正电压，V */
            vofa_frame_ready = 1U;
        }
    }
#endif
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC1) || (hadc->Instance == ADC2))
    {
        /* 采样/DMA 已报告错误时不能让功率级保持最后一次 PWM。 */
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 直接写输出禁用寄存器，避免 ISR 中等待 HAL 锁；初始化早期也可调用。 */
  if (hhrtim1.Instance != NULL)
  {
      hhrtim1.Instance->sCommonRegs.ODISR = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                          HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2;
  }
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
