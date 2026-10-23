#include "qpr_control.h"
#include <float.h>
#include <string.h>

#define QPR_PI 3.14159265358979323846

static uint8_t QPR_IsFinite(float value)
{
    return (uint8_t)((value <= FLT_MAX) && (value >= -FLT_MAX));
}

static float QPR_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 从机独立PR电流环：桥电压指令=公共电压+PR电流校正，除母线得调制度。 */
float QPR_CurrentStep(QPR_Control *control, float current_reference_a,
                      float current_a, float voltage_ff_v, float dc_bus_v)
{
    float error, raw, correction, bridge, modulation_raw, modulation;
    struct PR_PATH next;
    if (control == 0) return 0.0f;
    if ((!control->initialized) || (control->fault_flags != QPR_FAULT_NONE))
        return 0.0f;
    if ((!QPR_IsFinite(current_reference_a)) || (!QPR_IsFinite(current_a)) ||
        (!QPR_IsFinite(voltage_ff_v)) || (!QPR_IsFinite(dc_bus_v)) || (dc_bus_v < 1.0f))
    {
        control->fault_flags |= QPR_FAULT_INPUT;
        return 0.0f;
    }
    error = current_reference_a - current_a;
    /* 预计算本拍；从机饱和且误差继续推动饱和时，不提交整组 IIR 历史。 */
    next = control->pr;
    PR_update(&next, error);
    raw = next.Vo;
    bridge = voltage_ff_v + QPR_Clamp(raw, QPR_CORRECTION_LIMIT_V);
    if ((!QPR_IsFinite(raw)) ||
        (!QPR_IsFinite(bridge)) || (!QPR_IsFinite(bridge / dc_bus_v)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        return 0.0f;
    }
    correction = QPR_Clamp(raw, QPR_CORRECTION_LIMIT_V);
    modulation_raw = bridge / dc_bus_v;
    modulation = QPR_Clamp(modulation_raw, QPR_MODULATION_LIMIT);
    /* 保留从机条件冻结策略；直接 I 型的两拍输入/输出需整体提交或冻结。
     * 未实际限幅时直接放行，避免 (bridge/bus)*bus 舍入差误冻结状态。
     */
    if ((error * (raw - correction) <= 0.0f) &&
        ((modulation_raw == modulation) ||
         (error * (bridge - modulation * dc_bus_v) <= 0.0f)))
    {
        control->pr = next;
    }
    control->resonant_r = raw - QPR_KP * error;
    control->current_reference_a = current_reference_a;
    control->current_error_a = error;
    control->reference_v = voltage_ff_v;
    control->qpr_unlimited_v = raw;
    control->qpr_correction_v = correction;
    control->bridge_reference_v = bridge;
    control->modulation = modulation;
    if (raw != correction) control->qpr_saturation_count++;
    if (modulation_raw != modulation) control->modulation_saturation_count++;
    return modulation;
}

/* 开闭环共用同一幅值状态，切换模式时不重置参考幅值。 */
static void QPR_UpdateReference(QPR_Control *control, float sine_reference,
                                float reference_peak_v)
{
    float peak_step;

    /* 参考信号软启动 */
    peak_step = QPR_REFERENCE_SLEW_V_PER_S * control->sample_period_s;
    if (control->reference_peak_v < reference_peak_v)
    {
        control->reference_peak_v += peak_step;
        if (control->reference_peak_v > reference_peak_v)
            control->reference_peak_v = reference_peak_v;
    }
    else if (control->reference_peak_v > reference_peak_v)
    {
        control->reference_peak_v -= peak_step;
        if (control->reference_peak_v < reference_peak_v)
            control->reference_peak_v = reference_peak_v;
    }
    control->reference_v = control->reference_peak_v * sine_reference;
}

/* 一周期均值消除 50 Hz 及其整数次谐波对零偏估计的影响。
 * 斜坡过程中冻结估计并清空窗口，避免把幅值变化当成直流偏置。
 * 平滑系数按窗口时长计算；中断中不进行除法、指数或三角函数计算。
 */
static float QPR_CompensateVoltageError(QPR_Control *control, float raw_error,
                                       uint8_t reference_stable)
{
#if QPR_VOLTAGE_ERROR_DC_COMPENSATION_ENABLE
    {
        if (reference_stable != 0U)
        {
            control->voltage_error_dc_sum_v += raw_error;
            control->voltage_error_dc_count++;
            if (control->voltage_error_dc_count >= control->voltage_error_dc_window_samples)
            {
                /* alpha/window 在初始化时计算，保持实时路径只用乘加。 */
                const float target = control->voltage_error_dc_sum_v *
                                     control->voltage_error_dc_mean_scale;
                control->voltage_error_dc_v += control->voltage_error_dc_alpha *
                                               (target - control->voltage_error_dc_v);
                control->voltage_error_dc_sum_v = 0.0f;
                control->voltage_error_dc_count = 0U;
            }
        }
        else
        {
            control->voltage_error_dc_sum_v = 0.0f;
            control->voltage_error_dc_count = 0U;
        }
        return raw_error - control->voltage_error_dc_v;
    }
#else
    (void)control;
    (void)reference_stable;
    return raw_error;
#endif
}

void QPR_ControlReset(QPR_Control *control)
{
    if (control == 0) return;
    control->voltage_error_previous_v = 0.0f;
    PR_reset(&control->pr);
    control->resonant_r = 0.0f;
    control->reference_peak_v = 0.0f;
    control->reference_v = 0.0f;
    control->voltage_error_v = 0.0f;
    control->voltage_error_dc_v = 0.0f;
    control->voltage_error_compensated_v = 0.0f;
    control->voltage_error_dc_sum_v = 0.0f;
    control->voltage_error_dc_count = 0U;
    control->current_reference_a = 0.0f;
    control->current_error_a = 0.0f;
    control->qpr_unlimited_v = 0.0f;
    control->qpr_correction_v = 0.0f;
    control->bridge_reference_v = 0.0f;
    control->modulation = 0.0f;
    control->qpr_saturation_count = 0U;
    control->modulation_saturation_count = 0U;
    control->fault_flags = control->initialized ? QPR_FAULT_NONE : QPR_FAULT_CONFIGURATION;
}

uint8_t QPR_ControlInit(QPR_Control *control, float sample_frequency_hz)
{
    double ts;
    double virtual_denominator;
    double dc_window_samples;
    double dc_window_period_s;

    if (control == 0) return 0U;
    memset(control, 0, sizeof(*control));
    control->fault_flags = QPR_FAULT_CONFIGURATION;
    if ((!QPR_IsFinite(sample_frequency_hz)) ||
        (sample_frequency_hz <= 2.0f * QPR_FREQUENCY_HZ) ||
        (QPR_VIRTUAL_L_H <= 0.0f) || (QPR_VIRTUAL_R_OHM <= 0.0f) ||
        (QPR_WC_RAD_S <= 0.0f) || (QPR_REFERENCE_SLEW_V_PER_S <= 0.0f) ||
        (QPR_CORRECTION_LIMIT_V <= 0.0f) ||
        (!QPR_IsFinite(QPR_VOLTAGE_ERROR_DC_TRACK_TIME_S)) ||
        (QPR_VOLTAGE_ERROR_DC_TRACK_TIME_S <= 0.0f) ||
        (QPR_MODULATION_LIMIT <= 0.0f) || (QPR_MODULATION_LIMIT >= 1.0f))
    {
        return 0U;
    }

    /* 外环参数使用 double 计算；PR 系数保留压缩包的表达式及 float 接口。 */
    ts = 1.0 / (double)sample_frequency_hz;
    dc_window_samples = (double)sample_frequency_hz / (double)QPR_FREQUENCY_HZ;
    if (dc_window_samples > (double)UINT32_MAX - 0.5) return 0U;
    control->voltage_error_dc_window_samples = (uint32_t)(dc_window_samples + 0.5);
    control->voltage_error_dc_mean_scale = 1.0f / (float)control->voltage_error_dc_window_samples;
    dc_window_period_s = (double)control->voltage_error_dc_window_samples * ts;
    control->voltage_error_dc_alpha = (float)(dc_window_period_s /
        ((double)QPR_VOLTAGE_ERROR_DC_TRACK_TIME_S + dc_window_period_s));
    /* 移植 REG_PFC28379D 的直接 I 型 PR；Ts 来自本机真实控制频率。
     * 原工程 25 us/Kp=7.5/Kr=95 属于它的 PFC 对象，本机使用自己的参数。
     */
    PR_init(&control->pr, (float)ts, QPR_KP, QPR_KR, QPR_WC_RAD_S,
            (float)(2.0 * QPR_PI * (double)QPR_FREQUENCY_HZ));
    if ((!QPR_IsFinite(control->pr.A0)) || (!QPR_IsFinite(control->pr.A1)) ||
        (!QPR_IsFinite(control->pr.A2)) || (!QPR_IsFinite(control->pr.B0)) ||
        (!QPR_IsFinite(control->pr.B1)) || (!QPR_IsFinite(control->pr.B2)))
        return 0U;
    control->sample_period_s = (float)ts;

    /* 模型虚拟阻抗是连续 1/(Lv*s+Rv)；MCU 按 Tustin 离散化。 */
    virtual_denominator = 2.0 * (double)QPR_VIRTUAL_L_H + (double)QPR_VIRTUAL_R_OHM * ts;
    control->virtual_decay = (float)(2.0 * (double)QPR_VIRTUAL_R_OHM * ts / virtual_denominator);
    control->virtual_b = (float)(ts / virtual_denominator);
    control->initialized = 1U;
    QPR_ControlReset(control);
    return 1U;
}

float QPR_ControlStep(QPR_Control *control, float sine_reference,
                      float reference_peak_v, float voltage_v,
                      float inductor_current_a, float dc_bus_v)
{
    float voltage_error;
    float voltage_error_compensated;
    float current_reference;
    float current_error;
    float qpr_raw;
    float correction;
    float bridge_voltage;
    float modulation_raw;

    if (control == 0) return 0.0f;
    if ((!control->initialized) || (control->fault_flags != QPR_FAULT_NONE))
    {
        control->modulation = 0.0f;
        return 0.0f;
    }
    if ((!QPR_IsFinite(sine_reference)) || (sine_reference < -1.001f) ||
        (sine_reference > 1.001f) || (!QPR_IsFinite(reference_peak_v)) ||
        (reference_peak_v < 0.0f) || (!QPR_IsFinite(voltage_v)) ||
        (!QPR_IsFinite(inductor_current_a)) || (!QPR_IsFinite(dc_bus_v)) ||
        (dc_bus_v < 1.0f))
    {
        control->fault_flags |= QPR_FAULT_INPUT;
        control->modulation = 0.0f;
        return 0.0f;
    }

    QPR_UpdateReference(control, sine_reference, reference_peak_v);

    /* 外环：电压误差(V) -> 虚拟阻抗 -> 电感电流指令(A)。 */
    voltage_error = control->reference_v - voltage_v;
    voltage_error_compensated = QPR_CompensateVoltageError(control, voltage_error,
        (uint8_t)(control->reference_peak_v == reference_peak_v));
    current_reference = control->current_reference_a +
                        (control->virtual_b * (voltage_error_compensated + control->voltage_error_previous_v) -
                         control->virtual_decay * control->current_reference_a);
    current_error = current_reference - inductor_current_a;

    /* 内环：电感电流误差(A) -> QPR -> 电压校正量(V)。 */
    PR_update(&control->pr, current_error);
    qpr_raw = control->pr.Vo; /* PR 已含比例项，不能再重复加 Kp*error。 */

    /* 先检查再限幅，防止 NaN/Inf 进入 PWM 比较寄存器的整数转换。 */
    if ((!QPR_IsFinite(control->voltage_error_dc_sum_v)) ||
        (!QPR_IsFinite(control->voltage_error_dc_v)) ||
        (!QPR_IsFinite(current_reference)) || (!QPR_IsFinite(current_error)) ||
        (!QPR_IsFinite(qpr_raw)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        control->modulation = 0.0f;
        return 0.0f;
    }

    /* 保持模型次序：QPR 校正限幅 -> 加参考电压前馈 -> 除母线。 */
    correction = QPR_Clamp(qpr_raw, QPR_CORRECTION_LIMIT_V);
    bridge_voltage = correction + control->reference_v;
    modulation_raw = bridge_voltage / dc_bus_v;
    if ((!QPR_IsFinite(bridge_voltage)) || (!QPR_IsFinite(modulation_raw)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        control->modulation = 0.0f;
        return 0.0f;
    }

    /* 与原模型一致：状态保存未饱和结果。这里没有冒充硬件过流保护。 */
    control->voltage_error_previous_v = voltage_error_compensated;
    control->current_reference_a = current_reference;
    control->resonant_r = qpr_raw - QPR_KP * current_error;
    control->voltage_error_v = voltage_error;
    control->voltage_error_compensated_v = voltage_error_compensated;
    control->current_error_a = current_error;
    control->qpr_unlimited_v = qpr_raw;
    control->qpr_correction_v = correction;
    control->bridge_reference_v = bridge_voltage;
    control->modulation = QPR_Clamp(modulation_raw, QPR_MODULATION_LIMIT);
    if (qpr_raw != correction) control->qpr_saturation_count++;
    if (modulation_raw != control->modulation) control->modulation_saturation_count++;
    return control->modulation;
}

float QPR_ControlOpenLoopStep(QPR_Control *control, float sine_reference,
                              float reference_peak_v, float dc_bus_v)
{
    float modulation_raw;

    if (control == 0) return 0.0f;
    if ((!control->initialized) || (control->fault_flags != QPR_FAULT_NONE))
    {
        control->modulation = 0.0f;
        return 0.0f;
    }
    if ((!QPR_IsFinite(sine_reference)) || (sine_reference < -1.001f) ||
        (sine_reference > 1.001f) || (!QPR_IsFinite(reference_peak_v)) ||
        (reference_peak_v < 0.0f) || (!QPR_IsFinite(dc_bus_v)) ||
        (dc_bus_v < 1.0f))
    {
        control->fault_flags |= QPR_FAULT_INPUT;
        control->modulation = 0.0f;
        return 0.0f;
    }

    QPR_UpdateReference(control, sine_reference, reference_peak_v);
    modulation_raw = control->reference_v / dc_bus_v;
    if ((!QPR_IsFinite(control->reference_v)) || (!QPR_IsFinite(modulation_raw)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        control->modulation = 0.0f;
        return 0.0f;
    }

    /* 开环不累计反馈误差，避免重新闭环时使用之前残留的状态。
     * 保留幅值斜坡、限幅统计及故障；模式切换不保证校正电压无阶跃。
     */
    control->voltage_error_previous_v = 0.0f;
    PR_reset(&control->pr);
    control->resonant_r = 0.0f;
    control->voltage_error_v = 0.0f;
    control->voltage_error_dc_v = 0.0f;
    control->voltage_error_compensated_v = 0.0f;
    control->voltage_error_dc_sum_v = 0.0f;
    control->voltage_error_dc_count = 0U;
    control->current_reference_a = 0.0f;
    control->current_error_a = 0.0f;
    control->qpr_unlimited_v = 0.0f;
    control->qpr_correction_v = 0.0f;
    control->bridge_reference_v = control->reference_v;
    control->modulation = QPR_Clamp(modulation_raw, QPR_MODULATION_LIMIT);
    if (modulation_raw != control->modulation) control->modulation_saturation_count++;
    return control->modulation;
}
