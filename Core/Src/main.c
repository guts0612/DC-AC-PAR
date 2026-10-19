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
/* 八通道：20 kHz / 10 = 2000 帧/s，50 Hz 正弦每周期 40 个波形点。
 * 36 字节、8N1 占用 720000 bit/s；UART4 和 VOFA 均设置为 921600 baud。
 * CH8 的 RMS 使用全部 20 kHz 样本，每个 50 Hz 周期更新一次。
 */
#define VOFA_SAMPLE_DIV            10U
/* 1: CH1~6 为400点有符号均值；0: CH1~7为瞬时波形。CH8始终为RMS。 */
#define VOFA_MEAN_OUTPUT_ENABLE    0U
#define VOFA_MEAN_SAMPLE_COUNT     400U
#define VOFA_UART_TIMEOUT_MS       5U

#define VOLTAGE_ZERO_CODE (-3)
#define CURRENT_ZERO_CODE (-8)
/* 辅助电流零偏必须单独实测，不能沿用主机通道的标定结果。 */
#define AUX_CURRENT_ZERO_CODE (-15)
/* 母线零偏单位为换算后的 V；24=(24k+24k)/2k。 */
#define DC_BUS_ZERO_CODE (16)
#define DC_BUS_MIN_CONTROL_V 1.0f

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
/* Keil Watch：默认目标峰值约 33.9411 V，对应 24 Vrms；母线由 ADC3 实测。 */
/* 默认主机开环、从机关断；参考幅值保持原工程设定。 */
/* 1：QPR 闭环；0：正弦开环输出，不是停机/关断栅极。 */
volatile uint8_t voltage_closed_loop_enable = 0U;
volatile float voltage_reference_peak_v = QPR_DEFAULT_REFERENCE_PEAK_V;
volatile uint8_t dc_bus_ready;
volatile uint32_t adc_pair_error_count;
static uint8_t adc12_ready, adc34_ready;
volatile float voltage_reference_v;
/* 展开 qpr_control 可查看两环误差、校正电压、限幅次数和故障位。 */
QPR_Control qpr_control;
/* PB9只控制从机，不重新初始化主机。比例定义为iL从/iL主。 */
QPR_Control slave_qpr_control;
volatile uint8_t slave_enable_request = 0U;
volatile uint8_t slave_enabled = 0U;
volatile uint32_t slave_fault_flags;
volatile float slave_current_ratio = 1.0f;
volatile float slave_ratio_applied;
volatile float slave_dc_bus_voltage_v; /* 初始为0，由ADC3采样更新，单位V。 */
volatile float slave_modulation;
volatile uint32_t slave_update_count;
#define SLAVE_OUTPUTS (HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2)
#define SLAVE_REFERENCE_LIMIT_A 3.0f
#define SLAVE_OVERCURRENT_A 5.0f
#define SLAVE_RATIO_SLEW_PER_S 1.0f
#define SLAVE_FAULT_ADC 1U
#define SLAVE_FAULT_CURRENT 2U
#define SLAVE_FAULT_CONTROL 4U
#define SLAVE_FAULT_TIMING 8U
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
volatile float voltage_gain = 12.8f;       /* 电压采样通道的增益，V/V */
volatile float cksr6_current_gain = 2.400384f;   /* 电流采样通道的增益，A/V */
volatile uint32_t adc_sample_count;      /* ADC1 DMA 完成回调次数 */
/* ADC3/4 CDR：低16位为母线， 高16位为辅助电流。 */
volatile uint32_t adc34_word;
volatile uint16_t adc_bus_voltage_raw;
volatile uint16_t adc_aux_current_raw;
volatile int16_t adc_bus_voltage_code;
volatile int16_t adc_aux_current_code;
volatile float adc_sample_bus_voltage;
volatile float adc_sample_aux_current;   /* 去中点、减零偏后的ADC差分电压，V */
volatile float dc_bus_voltage_v; /* 本拍实测母线电压，V */
volatile float true_aux_current;         /* 辅助逆变器电感瞬时电流，A */
/* 暂按与主机相同的CKSR6链路换算，实际比例/极性需独立标定。 */
volatile float dc_bus_voltage_gain = 24.18717f;
volatile float aux_current_gain = 2.400384f;
volatile uint32_t adc4_sample_count;
volatile uint32_t adc4_error_count;


#if VOFA_OUTPUT_ENABLE
/* 单槽邮箱：CH1~7保留原定义；CH8为最近一个完整周期的电压RMS。 */
volatile float vofa_ch1;
volatile float vofa_ch2;
volatile float vofa_ch3;
volatile float vofa_ch4;
volatile float vofa_ch5;
volatile float vofa_ch6;
volatile float vofa_ch7;
volatile float vofa_ch8;
/* 仅用于观测，不反馈给QPR。这里的RMS是采样带宽内的有效值。
 * RMS=sqrt(sum(u[k]^2)/N)，保留采样信号的直流分量及谐波。
 * AC_RMS=sqrt(RMS^2-mean^2)供Watch对照交流档；互感器看不到实际输出DC。
 * 20 kHz/50 Hz时N=400；每拍平方累加，一周期才开方，避免串口抽点偏差。
 */
volatile float output_voltage_rms_v;
volatile float output_voltage_mean_v;
volatile float output_voltage_ac_rms_v;
volatile uint8_t output_voltage_rms_valid; /* 首个完整窗口之前及模式切换后为0。 */
volatile uint32_t output_voltage_rms_window_count;
uint32_t output_voltage_rms_window_samples;
static float voltage_rms_square_sum;
static float voltage_rms_sum;
static float voltage_rms_mean_scale;
static uint32_t voltage_rms_count;
static uint8_t voltage_rms_mode;
/* 均值窗口只由 ADC ISR 访问，单精度累加使用 M4F 硬件浮点。 */
static float vofa_mean_sum[6];
static uint32_t vofa_mean_count;
static uint8_t vofa_mean_mode;
volatile uint32_t vofa_mean_window_count;
volatile uint8_t vofa_frame_ready;
volatile uint32_t adc_sample_div_count;
volatile uint32_t vofa_sent_count;        /* HAL 返回成功的完整帧数 */
volatile uint32_t vofa_tx_error_count;    /* HAL 超时、忙或错误的累计次数 */
volatile uint32_t vofa_overwrite_count;   /* 主循环未取走就被新采样覆盖的帧数 */
static uint8_t vofa_frame[36];  /* 八个 float（32 字节）+ JustFloat 帧尾（4 字节）。 */
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

/* C/D与A/B相同的单极倍频调制，只有本函数写从机比较值。 */
static void HRTIM_SetSlaveModulation(float modulation)
{
    uint32_t c, d;
    uint32_t maximum = (uint32_t)Period - HRTIM_COMPARE_MARGIN_TICKS;
    if (modulation > SPWM_MODULATION_LIMIT) modulation = SPWM_MODULATION_LIMIT;
    if (modulation < -SPWM_MODULATION_LIMIT) modulation = -SPWM_MODULATION_LIMIT;
    c = (uint32_t)(0.5f * (1.0f + modulation) * Period + 0.5f);
    d = (uint32_t)(0.5f * (1.0f - modulation) * Period + 0.5f);
    if (c < HRTIM_COMPARE_MARGIN_TICKS) c = HRTIM_COMPARE_MARGIN_TICKS;
    if (d < HRTIM_COMPARE_MARGIN_TICKS) d = HRTIM_COMPARE_MARGIN_TICKS;
    if (c > maximum) c = maximum;
    if (d > maximum) d = maximum;
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, c);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, d);
    slave_modulation = modulation;
}

/* 关断是禁用四个栅极PWM，不是m=0的50%发波；不操作A/B。 */
static void Slave_Stop(uint32_t fault)
{
    HRTIM1->sCommonRegs.ODISR = SLAVE_OUTPUTS;
    slave_enabled = 0U;
    slave_enable_request = 0U;
    slave_fault_flags |= fault;
    slave_modulation = 0.0f;
}

/* 两组样本配齐并完成主机控制后执行；故障锁存后重启才能再接入。 */
static void Slave_ControlUpdate(void)
{
    float reference, modulation, ratio, master_current, output_voltage;
    uint32_t primask, sample_sequence;
    if ((voltage_closed_loop_enable == 0U) || (slave_enable_request == 0U))
    {
        Slave_Stop(0U);
        return;
    }
    if (slave_fault_flags != 0U) { Slave_Stop(0U); return; }
    /* 两组从同一触发启动；只有ADC1已发布本拍反馈才运行从机。 */
    primask = __get_PRIMASK();
    __disable_irq();
    sample_sequence = adc_sample_count;
    master_current = true_current;
    output_voltage = true_voltage;
    __set_PRIMASK(primask);
    if (sample_sequence != adc4_sample_count)
    {
        Slave_Stop(SLAVE_FAULT_TIMING);
        return;
    }
    if ((true_aux_current > SLAVE_OVERCURRENT_A) ||
        (true_aux_current < -SLAVE_OVERCURRENT_A))
    {
        Slave_Stop(SLAVE_FAULT_CURRENT);
        return;
    }
    ratio = slave_current_ratio;
    if (!(ratio >= 0.0f && ratio <= 2.0f))
    {
        Slave_Stop(SLAVE_FAULT_CONTROL);
        return;
    }
    if (slave_enabled == 0U)
    {
        /* 在参考电压过零附近接入，避免任意相位直接开启第一脉冲。 */
        if (!(voltage_reference_v >= -2.0f && voltage_reference_v <= 2.0f)) return;
        QPR_ControlReset(&slave_qpr_control);
        slave_ratio_applied = 0.0f;
    }
    if (slave_ratio_applied < ratio)
    {
        slave_ratio_applied += SLAVE_RATIO_SLEW_PER_S * slave_qpr_control.sample_period_s;
        if (slave_ratio_applied > ratio) slave_ratio_applied = ratio;
    }
    else if (slave_ratio_applied > ratio)
    {
        slave_ratio_applied -= SLAVE_RATIO_SLEW_PER_S * slave_qpr_control.sample_period_s;
        if (slave_ratio_applied < ratio) slave_ratio_applied = ratio;
    }
    /* 直接跟随本拍主机实测电感电流，不额外增加参考路径的滤波延迟。 */
    reference = slave_ratio_applied * master_current;
    if (reference > SLAVE_REFERENCE_LIMIT_A) reference = SLAVE_REFERENCE_LIMIT_A;
    if (reference < -SLAVE_REFERENCE_LIMIT_A) reference = -SLAVE_REFERENCE_LIMIT_A;
    modulation = QPR_CurrentStep(&slave_qpr_control, reference, true_aux_current,
                                 output_voltage, slave_dc_bus_voltage_v);
    if (slave_qpr_control.fault_flags != QPR_FAULT_NONE)
    {
        Slave_Stop(SLAVE_FAULT_CONTROL);
        return;
    }
    /* 短临界区保护从机输出接入。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (adc_sample_count != sample_sequence)
    {
        Slave_Stop(SLAVE_FAULT_TIMING);
        __set_PRIMASK(primask);
        return;
    }
    if ((voltage_closed_loop_enable != 0U) && (slave_enable_request != 0U) &&
        (slave_fault_flags == 0U))
    {
        HRTIM_SetSlaveModulation(modulation);
        if (slave_enabled == 0U)
        {
            /* 首次只更新C/D，绝不软件更新主机A/B的预装载值。 */
            if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_C | HRTIM_TIMERUPDATE_D) != HAL_OK)
            {
                Slave_Stop(SLAVE_FAULT_CONTROL);
            }
            else
            {
                if ((HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
                      HRTIM_OUTPUT_TC1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK) ||
                    (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
                      HRTIM_OUTPUT_TD1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK))
                    Slave_Stop(SLAVE_FAULT_CONTROL);
                else
                {
                    HRTIM1->sCommonRegs.OENR = SLAVE_OUTPUTS;
                    slave_enabled = 1U;
                }
            }
        }
        slave_update_count++;
    }
    __set_PRIMASK(primask);
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
#if VOFA_OUTPUT_ENABLE
    /* 按实际控制频率取一基波周期的点数，不依赖VOFA抽点倍数。 */
    output_voltage_rms_window_samples =
        (uint32_t)(spwm_update_hz / SPWM_OUTPUT_FREQUENCY_HZ + 0.5f);
    voltage_rms_mean_scale = 1.0f / (float)output_voltage_rms_window_samples;
#endif

    if ((__HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C) != Period) ||
        (__HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D) != Period) ||
        ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].TIMxCR & HRTIM_TIMCR_CK_PSC) != HRTIM_PRESCALERRATIO_DIV1) ||
        ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].TIMxCR & HRTIM_TIMCR_CK_PSC) != HRTIM_PRESCALERRATIO_DIV1)) Error_Handler();
    if (QPR_ControlInit(&slave_qpr_control, spwm_update_hz) == 0U) Error_Handler();
    Slave_Stop(0U);
    HRTIM_SetSlaveModulation(0.0f);
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_C | HRTIM_TIMERUPDATE_D) != HAL_OK) Error_Handler();
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
    /* 先启动采样；第一组有效母线样本到齐后才开启 A/B 栅极。 */
    /* M/A/B/C/D计数器均为0，一次MCR写入同步启动；C/D输出仍禁用。
     * 不使用“未开计数器时的软件复位”：RM0440 p895 说明此时复位请求无效。
     */
    if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER |
                                      HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
                                      HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D) != HAL_OK)
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
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
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
    /* 两组主ADC同时等待HRTIM_TRG1下降沿，ADC4只由ADC3硬件带动。 */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc3, (uint32_t *)&adc34_word, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle, DMA_IT_HT);
    /* 双模式的从ADC同样监测溢出，避免DMA中断延迟时静默丢样本。 */
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc4, ADC_FLAG_OVR);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_OVR);
    __HAL_ADC_ENABLE_IT(&hadc4, ADC_IT_OVR);
}

#if VOFA_OUTPUT_ENABLE
/* 在两组ADC样本配齐后、VOFA抽点之前调用，开环/闭环均可观测。
 * 模式切换时丢弃半个旧窗口；0表示尚未完成新窗口，约20ms后有效。
 */
static void VOFA_AccumulateVoltageRms(uint8_t closed_loop)
{
    float mean_square;
    float ac_mean_square;
    float rms_v;
    float ac_rms_v;
    const float voltage_v = true_voltage;

    if (closed_loop != voltage_rms_mode)
    {
        voltage_rms_square_sum = 0.0f;
        voltage_rms_sum = 0.0f;
        voltage_rms_count = 0U;
        voltage_rms_mode = closed_loop;
        output_voltage_rms_v = 0.0f;
        output_voltage_mean_v = 0.0f;
        output_voltage_ac_rms_v = 0.0f;
        output_voltage_rms_valid = 0U;
    }
    voltage_rms_square_sum += voltage_v * voltage_v;
    voltage_rms_sum += voltage_v;
    voltage_rms_count++;
    if (voltage_rms_count < output_voltage_rms_window_samples) return;

    mean_square = voltage_rms_square_sum * voltage_rms_mean_scale;
    output_voltage_mean_v = voltage_rms_sum * voltage_rms_mean_scale;
    (void)arm_sqrt_f32(mean_square, &rms_v);
    output_voltage_rms_v = rms_v;
    ac_mean_square = mean_square - output_voltage_mean_v * output_voltage_mean_v;
    if (ac_mean_square < 0.0f) ac_mean_square = 0.0f; /* 消除浮点舍入造成的微小负数。 */
    (void)arm_sqrt_f32(ac_mean_square, &ac_rms_v);
    output_voltage_ac_rms_v = ac_rms_v;
    voltage_rms_square_sum = 0.0f;
    voltage_rms_sum = 0.0f;
    voltage_rms_count = 0U;
    output_voltage_rms_window_count++;
    output_voltage_rms_valid = 1U;
}

/* 每拍调用；完成窗口时一次发布六个结果。不是 RMS，也不是抽点平均。
 * 固定 400 点在当前 20 kHz/50 Hz 配置下对应一个基波周期。
 */
static void VOFA_AccumulateMean(uint8_t closed_loop)
{
    uint32_t i;
    const float scale = 1.0f / (float)VOFA_MEAN_SAMPLE_COUNT;

    if (closed_loop != vofa_mean_mode)
    {
        for (i = 0U; i < 6U; ++i) vofa_mean_sum[i] = 0.0f;
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
    vofa_mean_sum[5] += dc_bus_voltage_v;
    vofa_mean_count++;
    if (vofa_mean_count < VOFA_MEAN_SAMPLE_COUNT) return;

    if (vofa_frame_ready != 0U) vofa_overwrite_count++;
    vofa_ch1 = vofa_mean_sum[0] * scale; /* 平均电压误差，V */
    vofa_ch2 = vofa_mean_sum[1] * scale; /* 平均电流参考，A */
    vofa_ch3 = vofa_mean_sum[2] * scale; /* 平均实测电流，A */
    vofa_ch4 = vofa_mean_sum[3] * scale; /* 平均谐振项，V */
    vofa_ch5 = vofa_mean_sum[4] * scale; /* 平均未限幅 QPR 输出，V */
    vofa_ch6 = vofa_mean_sum[5] * scale; /* CH6：平均实测母线电压，V */
    /* CH7始终为最近一次完成的辅助电流样本，不随CH1~6切换为均值。 */
    vofa_ch7 = true_aux_current;
    vofa_ch8 = output_voltage_rms_v; /* CH8不随CH1~6切换为均值。 */
    for (i = 0U; i < 6U; ++i) vofa_mean_sum[i] = 0.0f;
    vofa_mean_count = 0U;
    vofa_mean_window_count++;
    vofa_frame_ready = 1U;
}

static void VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7, float ch8)
{
    HAL_StatusTypeDef status;

    memcpy(&vofa_frame[0], &ch1, sizeof(ch1));
    memcpy(&vofa_frame[4], &ch2, sizeof(ch2));
    memcpy(&vofa_frame[8], &ch3, sizeof(ch3));
    memcpy(&vofa_frame[12], &ch4, sizeof(ch4));
    memcpy(&vofa_frame[16], &ch5, sizeof(ch5));
    memcpy(&vofa_frame[20], &ch6, sizeof(ch6));
    memcpy(&vofa_frame[24], &ch7, sizeof(ch7));
    memcpy(&vofa_frame[28], &ch8, sizeof(ch8));
    vofa_frame[32] = 0x00U;
    vofa_frame[33] = 0x00U;
    vofa_frame[34] = 0x80U;
    vofa_frame[35] = 0x7FU;

    /* 主循环轮询发送，等待期间允许 HRTIM/ADC 中断继续运行。
     * 921600 下 36 字节在线时间约 0.391 ms，小于 0.5 ms 采样间隔。
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
    float ch7;
    float ch8;

    if (vofa_frame_ready == 0U)
    {
        return;
    }

    /* 一次取走完整八通道快照；保留调用前的中断屏蔽状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    ch1 = vofa_ch1;
    ch2 = vofa_ch2;
    ch3 = vofa_ch3;
    ch4 = vofa_ch4;
    ch5 = vofa_ch5;
    ch6 = vofa_ch6;
    ch7 = vofa_ch7;
    ch8 = vofa_ch8;

    vofa_frame_ready = 0U;
    __set_PRIMASK(primask);

    VOFA_SendJustFloat(ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8);
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
  MX_ADC3_Init();
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
        if (voltage_closed_loop_enable == 0U) Slave_Stop(0U);
    }
    else if (key_number == 2U)
    {
        /* PB9松手：只在主机闭环时切换从机接入；开环时按键不排队。 */
        if (voltage_closed_loop_enable != 0U)
        {
            if (slave_enable_request != 0U) Slave_Stop(0U);
            else if (slave_fault_flags == 0U) slave_enable_request = 1U;
        }
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

    /* 两个DMA同抢占优先级，回调不能彼此打断。先来的只发布快照，
     * 后来的运行一次控制；不轮询等待，也不使用上一拍母线。
     * 同一组连续完成两次而另一组未到，禁止继续配对并关断全部输出。
     */
    if (hadc->Instance == ADC3)
    {
        if (adc34_ready != 0U) { adc_pair_error_count++; Error_Handler(); return; }
        sample = adc34_word;
        adc_bus_voltage_raw = (uint16_t)(sample & 0x0FFFU);
        adc_aux_current_raw = (uint16_t)((sample >> 16) & 0x0FFFU);
        adc_bus_voltage_code = ADC_DiffRawToSigned(adc_bus_voltage_raw)- DC_BUS_ZERO_CODE;
        adc_sample_bus_voltage = (adc_bus_voltage_code / 2048.0f) * vref_voltage;
        dc_bus_voltage_v = adc_sample_bus_voltage * dc_bus_voltage_gain ;
        /* 两台逆变器共用这条直流母线。 */
        slave_dc_bus_voltage_v = dc_bus_voltage_v;
        adc_aux_current_code = ADC_DiffRawToSigned(adc_aux_current_raw)- AUX_CURRENT_ZERO_CODE;
        adc_sample_aux_current = (adc_aux_current_code / 2048.0f) * vref_voltage;
        true_aux_current = adc_sample_aux_current * aux_current_gain;
        adc4_sample_count++;
        adc34_ready = 1U;
    }
    else if (hadc->Instance == ADC1)
    {
        if (adc12_ready != 0U) { adc_pair_error_count++; Error_Handler(); return; }
        sample = adc12_word;
        adc_voltage_raw = (uint16_t)(sample & 0x0FFFU);
        adc_current_raw = (uint16_t)((sample >> 16) & 0x0FFFU);
        adc_voltage_code = ADC_DiffRawToSigned(adc_voltage_raw)- VOLTAGE_ZERO_CODE;
        adc_current_code = ADC_DiffRawToSigned(adc_current_raw)- CURRENT_ZERO_CODE;
        adc_sample_voltage = -(adc_voltage_code / 2048.0f) * vref_voltage;
        adc_sample_current = (adc_current_code / 2048.0f) * vref_voltage;
        true_voltage = adc_sample_voltage * voltage_gain;
        true_current = adc_sample_current * cksr6_current_gain;
        adc_sample_count++;
        adc12_ready = 1U;
    }
    else return;
    if ((adc12_ready == 0U) || (adc34_ready == 0U)) return;
    adc12_ready = 0U;
    adc34_ready = 0U;
    if (adc_sample_count != adc4_sample_count)
    {
        adc_pair_error_count++;
        Error_Handler();
        return;
    }

    /* Master 向上计数：170 MHz/8500=20 kHz；A/B完整三角波=20 kHz。
     * ISR 写下一拍的预装载值，下一谷底装载；每个50 Hz周期约400次更新。
     */
    closed_loop = (voltage_closed_loop_enable != 0U) ? 1U : 0U;
    /* 母线尚未上电时继续遥测，但不把0V送入除法或开启栅极。
     * 低于1V时重置两环；母线恢复后主机从零幅值重新斜坡。
     */
    if (!(dc_bus_voltage_v >= DC_BUS_MIN_CONTROL_V))
    {
        HRTIM1->sCommonRegs.ODISR = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                  HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | SLAVE_OUTPUTS;
        dc_bus_ready = 0U;
        Slave_Stop(0U);
        QPR_ControlReset(&qpr_control);
        QPR_ControlReset(&slave_qpr_control);
        modulation = 0.0f;
    }
    else if (closed_loop != 0U)
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
    if ((dc_bus_voltage_v >= DC_BUS_MIN_CONTROL_V) && (dc_bus_ready == 0U))
    {
        /* 初次启动/恢复：先装载当前的零起步比较值，再打开主机输出。 */
        if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK)
        { Error_Handler(); return; }
        if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                         HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2) != HAL_OK)
        { Error_Handler(); return; }
        dc_bus_ready = 1U;
    }
    /* 两种模式均连续推进相位；幅值斜坡由控制函数共用。 */
    spwm_phase_rad += phase_step_rad;
    if (spwm_phase_rad >= 2.0f * PI) spwm_phase_rad -= 2.0f * PI;
    voltage_reference_v = qpr_control.reference_v;
    spwm_update_count++;

    Slave_ControlUpdate(); /* 本拍主从电流/母线已齐，固定主机先、从机后。 */

    /* ADC ISR 只计算控制和准备遥测；阻塞式串口发送仍在主循环。 */
#if VOFA_OUTPUT_ENABLE
    VOFA_AccumulateVoltageRms(closed_loop);
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
            vofa_ch3 = qpr_control.current_reference_a;  /* CH3：误差DC补偿后虚拟阻抗电流参考，A */
            vofa_ch4 = true_current;         /* CH4：滤波电感电流，A */
            vofa_ch5 = qpr_control.qpr_correction_v;  /* CH5：限幅后QPR输出（校正量），V */
            /* CH6/CH7来自与主机同一触发的ADC3/4快照。 */
            vofa_ch6 = true_aux_current;             /* CH6：辅助电感电流，A */
            vofa_ch7 = dc_bus_voltage_v;             /* CH7：实测直流母线电压，V */
            vofa_ch8 = output_voltage_rms_v;         /* CH8：输出电压一周期有效值，V */
            
            vofa_frame_ready = 1U;
        }
    }
#endif
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC3) || (hadc->Instance == ADC4)) adc4_error_count++;
    if ((hadc->Instance == ADC1) || (hadc->Instance == ADC2) ||
        (hadc->Instance == ADC3) || (hadc->Instance == ADC4))
    {
        /* 母线和反馈均参与控制，任一组ADC/DMA错误都关断全部输出。 */
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
                                          HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | SLAVE_OUTPUTS;
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
