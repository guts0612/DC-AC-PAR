#ifndef QPR_CONTROL_H
#define QPR_CONTROL_H

#include <stdint.h>

/* singlephase.slx / Digital PR，Wc 的单位是 rad/s，不是 Hz。 */
#define QPR_KP                         1.0f
#define QPR_KR                         1.6f
#define QPR_WC_RAD_S                   5.0f
#define QPR_FREQUENCY_HZ               50.0f
#define QPR_VIRTUAL_L_H                0.001f
#define QPR_VIRTUAL_R_OHM              0.01f
#define QPR_CORRECTION_LIMIT_V         5.0f

/* 电压互感器不反馈输出直流：在虚拟阻抗前去掉误差的缓慢直流分量。
 * 每个 50 Hz 周期收集一次全速均值（20 kHz 时为 400 点），以 0.2 s
 * 时间常数平滑更新；不是把每拍误差清零，也不替代电流采样零偏标定。
 */
#define QPR_VOLTAGE_ERROR_DC_COMPENSATION_ENABLE  1U
#define QPR_VOLTAGE_ERROR_DC_TRACK_TIME_S         0.2f

/* 当前默认工作点：48 V 母线，24 Vrms（峰值约 33.9411 V）。 */
#define QPR_DEFAULT_DC_BUS_V           48.0f
#define QPR_DEFAULT_REFERENCE_PEAK_V   (24.0f * 1.41421356237f)
#define QPR_MODULATION_LIMIT           0.95f  /* 32 Vrms / 48 V 所需调制度约 0.9428 */
#define QPR_REFERENCE_SLEW_V_PER_S      60.0f  /* 从 0 到 33.9411 V 峰值约需 0.566 s */

#define QPR_FAULT_NONE                 0U
#define QPR_FAULT_CONFIGURATION        (1UL << 0)
#define QPR_FAULT_INPUT                (1UL << 1)
#define QPR_FAULT_NUMERIC              (1UL << 2)

typedef struct
{
    /* 系数只在初始化时计算；中断中全部使用单精度乘加。 */
    float sample_period_s;
    float virtual_decay;
    float virtual_b;
    float resonant_decay;
    float resonant_b;
    float resonant_g;
    float resonant_h;
    float voltage_error_dc_alpha;
    float voltage_error_dc_mean_scale;
    uint32_t voltage_error_dc_window_samples;

    float voltage_error_previous_v;  /* 上一拍补偿后的误差，供 Tustin 使用。 */
    float current_error_previous_a;
    float resonant_r;
    float resonant_q;

    /* 同一控制拍的观测量，可在 Keil Watch 中展开查看。 */
    float reference_peak_v;
    float reference_v;
    float voltage_error_v;              /* 原始参考-反馈，V；遥测仍保留此量。 */
    float voltage_error_dc_v;           /* 估计的直流误差，V。 */
    float voltage_error_compensated_v;  /* 原始误差-直流估计，虚拟阻抗输入，V。 */
    float voltage_error_dc_sum_v;
    uint32_t voltage_error_dc_count;
    float current_reference_a;
    float current_error_a;
    float qpr_unlimited_v;
    float qpr_correction_v;
    float bridge_reference_v;
    float modulation;
    uint32_t qpr_saturation_count;
    uint32_t modulation_saturation_count;
    uint32_t fault_flags;
    uint8_t initialized;
} QPR_Control;

/* 返回 1 表示系数有效；sample_frequency_hz 来自实际 HRTIM 时钟/周期。 */
uint8_t QPR_ControlInit(QPR_Control *control, float sample_frequency_hz);
/* 保留系数，清除动态状态、斜坡、统计及运行故障。 */
void QPR_ControlReset(QPR_Control *control);
/*
 * 每次 ADC1 DMA 完成调用一次。
 * sine_reference: 50 Hz 单位正弦；reference_peak_v: 目标峰值，非 RMS。
 * voltage_v: 已标定输出电压；inductor_current_a: 电容分流之前的电感电流。
 * dc_bus_v: 实际母线值；本板尚无母线 ADC，main.c 使用用户设定值。
 * 返回值为无量纲调制度 m；本函数不访问 ADC、PWM 或 UART。
 */
float QPR_ControlStep(QPR_Control *control, float sine_reference,
                      float reference_peak_v, float voltage_v,
                      float inductor_current_a, float dc_bus_v);

/* 开环正弦：保留幅值斜坡，m=reference_v/dc_bus_v，不使用电压/电流反馈。
 * 清除闭环动态状态，保留统计及故障；相位由 main.c 连续推进。
 */
float QPR_ControlOpenLoopStep(QPR_Control *control, float sine_reference,
                              float reference_peak_v, float dc_bus_v);

/* 从机只使用电流内环，独立状态；voltage_ff_v为公共输出电压前馈。 */
float QPR_CurrentStep(QPR_Control *control, float current_reference_a,
                      float current_a, float voltage_ff_v, float dc_bus_v);

#endif
