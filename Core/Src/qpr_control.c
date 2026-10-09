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
    float error, r, q, raw, correction, bridge, modulation;
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
    r = control->resonant_r - control->resonant_decay * control->resonant_r +
        control->resonant_b * control->resonant_q +
        control->resonant_g * (error + control->current_error_previous_a);
    q = control->resonant_q + control->resonant_h * (r + control->resonant_r);
    raw = QPR_KP * error + r;
    bridge = voltage_ff_v + QPR_Clamp(raw, QPR_CORRECTION_LIMIT_V);
    if ((!QPR_IsFinite(r)) || (!QPR_IsFinite(q)) || (!QPR_IsFinite(raw)) ||
        (!QPR_IsFinite(bridge)) || (!QPR_IsFinite(bridge / dc_bus_v)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        return 0.0f;
    }
    correction = QPR_Clamp(raw, QPR_CORRECTION_LIMIT_V);
    modulation = QPR_Clamp(bridge / dc_bus_v, QPR_MODULATION_LIMIT);
    /* 误差继续推动饱和时冻结谐振状态，避免从机接入时累计过大校正量。 */
    if ((error * (raw - correction) <= 0.0f) &&
        ((bridge / dc_bus_v == modulation) ||
         (error * (bridge - modulation * dc_bus_v) <= 0.0f)))
    {
        control->resonant_r = r;
        control->resonant_q = q;
    }
    control->current_error_previous_a = error;
    control->current_reference_a = current_reference_a;
    control->current_error_a = error;
    control->reference_v = voltage_ff_v;
    control->qpr_unlimited_v = raw;
    control->qpr_correction_v = correction;
    control->bridge_reference_v = bridge;
    control->modulation = modulation;
    if (raw != correction) control->qpr_saturation_count++;
    if (bridge / dc_bus_v != modulation) control->modulation_saturation_count++;
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

void QPR_ControlReset(QPR_Control *control)
{
    if (control == 0) return;
    control->voltage_error_previous_v = 0.0f;
    control->current_error_previous_a = 0.0f;
    control->resonant_r = 0.0f;
    control->resonant_q = 0.0f;
    control->voltage_resonant_r = 0.0f;
    control->voltage_resonant_q = 0.0f;
    control->voltage_qpr_unlimited_a = 0.0f;
    control->reference_peak_v = 0.0f;
    control->reference_v = 0.0f;
    control->voltage_error_v = 0.0f;
    control->current_reference_a = 0.0f;
    control->current_error_a = 0.0f;
    control->qpr_unlimited_v = 0.0f;
    control->qpr_correction_v = 0.0f;
    control->bridge_reference_v = 0.0f;
    control->modulation = 0.0f;
    control->qpr_saturation_count = 0U;
    control->voltage_qpr_saturation_count = 0U;
    control->modulation_saturation_count = 0U;
    control->fault_flags = control->initialized ? QPR_FAULT_NONE : QPR_FAULT_CONFIGURATION;
}

uint8_t QPR_ControlInit(QPR_Control *control, float sample_frequency_hz)
{
    double ts;
    double theta;
    double damping;
    double denominator;

    if (control == 0) return 0U;
    memset(control, 0, sizeof(*control));
    control->fault_flags = QPR_FAULT_CONFIGURATION;
    if ((!QPR_IsFinite(sample_frequency_hz)) ||
        (sample_frequency_hz <= 2.0f * QPR_FREQUENCY_HZ) ||
        (!QPR_IsFinite(QPR_VOLTAGE_KP_A_PER_V)) || (QPR_VOLTAGE_KP_A_PER_V < 0.0f) ||
        (!QPR_IsFinite(QPR_VOLTAGE_KR_A_PER_V)) || (QPR_VOLTAGE_KR_A_PER_V < 0.0f) ||
        (!QPR_IsFinite(QPR_VOLTAGE_WC_RAD_S)) || (QPR_VOLTAGE_WC_RAD_S <= 0.0f) ||
        (!QPR_IsFinite(QPR_CURRENT_REFERENCE_LIMIT_A)) || (QPR_CURRENT_REFERENCE_LIMIT_A <= 0.0f) ||
        (QPR_WC_RAD_S <= 0.0f) || (QPR_REFERENCE_SLEW_V_PER_S <= 0.0f) ||
        (QPR_CORRECTION_LIMIT_V <= 0.0f) ||
        (QPR_MODULATION_LIMIT <= 0.0f) || (QPR_MODULATION_LIMIT >= 1.0f))
    {
        return 0U;
    }

    /* 初始化使用 double 避免系数生成时的精度损失；实时运算仍是 float。 */
    ts = 1.0 / (double)sample_frequency_hz;
    theta = 2.0 * QPR_PI * (double)QPR_FREQUENCY_HZ * ts;
    damping = (double)QPR_WC_RAD_S * ts;
    denominator = 1.0 + damping + theta * theta * 0.25;

    /*
     * 原模型 Gqpr(s)=Kp+2*Kr*Wc*s/(s*s+2*Wc*s+W0*W0)。
     * 采用与 MATLAB Function 相同的 Tustin 变换，但用等价双状态实现。
     * 连续状态：dr/dt=-2*Wc*r-W0*q+2*Kr*Wc*e；dq/dt=W0*r。
     * 离散状态：rn=r+(-decay*r+B*q+G*(e+e1))；qn=q+H*(rn+r)。
     * 输出 y=Kp*e+rn；小衰减量单独保存，避免把接近 1 的系数量化。
     * 避免原直接二阶差分在 float 中 b0/b1/b2 接近抵消造成的精度损失。
     */
    control->resonant_decay = (float)((2.0 * damping + theta * theta * 0.5) / denominator);
    control->resonant_b = (float)(-theta / denominator);
    control->resonant_g = (float)((double)QPR_KR * damping / denominator);
    control->resonant_h = (float)(theta * 0.5);
    control->sample_period_s = (float)ts;

    /* 电压外环采用同一 Tustin 双状态形式，独立带宽/增益/状态。 */
    damping = (double)QPR_VOLTAGE_WC_RAD_S * ts;
    denominator = 1.0 + damping + theta * theta * 0.25;
    control->voltage_resonant_decay = (float)((2.0 * damping + theta * theta * 0.5) / denominator);
    control->voltage_resonant_b = (float)(-theta / denominator);
    control->voltage_resonant_g = (float)((double)QPR_VOLTAGE_KR_A_PER_V * damping / denominator);
    control->voltage_resonant_h = (float)(theta * 0.5);
    control->initialized = 1U;
    QPR_ControlReset(control);
    return 1U;
}

float QPR_ControlStep(QPR_Control *control, float sine_reference,
                      float reference_peak_v, float voltage_v,
                      float inductor_current_a, float dc_bus_v)
{
    float voltage_error;
    float current_reference;
    float current_error;
    float resonant_next;
    float quadrature_next;
    float qpr_raw;
    float correction;
    float bridge_voltage;
    float modulation_raw;
    float voltage_resonant_next;
    float voltage_quadrature_next;
    float voltage_qpr_raw;

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

    /* 外环：电压误差(V) -> 电压 QPR -> 限幅后的电感电流指令(A)。 */
    voltage_error = control->reference_v - voltage_v;
    voltage_resonant_next = control->voltage_resonant_r +
        (-control->voltage_resonant_decay * control->voltage_resonant_r +
         control->voltage_resonant_b * control->voltage_resonant_q +
         control->voltage_resonant_g * (voltage_error + control->voltage_error_previous_v));
    voltage_quadrature_next = control->voltage_resonant_q +
        control->voltage_resonant_h * (voltage_resonant_next + control->voltage_resonant_r);
    voltage_qpr_raw = QPR_VOLTAGE_KP_A_PER_V * voltage_error + voltage_resonant_next;
    current_reference = QPR_Clamp(voltage_qpr_raw, QPR_CURRENT_REFERENCE_LIMIT_A);
    current_error = current_reference - inductor_current_a;

    /* 内环：电感电流误差(A) -> QPR -> 电压校正量(V)。 */
    resonant_next = control->resonant_r +
                    (-control->resonant_decay * control->resonant_r +
                     control->resonant_b * control->resonant_q +
                     control->resonant_g * (current_error + control->current_error_previous_a));
    quadrature_next = control->resonant_q +
                      control->resonant_h * (resonant_next + control->resonant_r);
    qpr_raw = QPR_KP * current_error + resonant_next;

    /* 先检查再限幅，防止 NaN/Inf 进入 PWM 比较寄存器的整数转换。 */
    if ((!QPR_IsFinite(voltage_error)) || (!QPR_IsFinite(voltage_resonant_next)) ||
        (!QPR_IsFinite(voltage_quadrature_next)) || (!QPR_IsFinite(voltage_qpr_raw)) ||
        (!QPR_IsFinite(current_reference)) || (!QPR_IsFinite(current_error)) ||
        (!QPR_IsFinite(resonant_next)) || (!QPR_IsFinite(quadrature_next)) ||
        (!QPR_IsFinite(qpr_raw)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        control->modulation = 0.0f;
        return 0.0f;
    }

    /* 内环校正限幅 -> 加参考电压前馈 -> 除母线 -> 调制度限幅。 */
    correction = QPR_Clamp(qpr_raw, QPR_CORRECTION_LIMIT_V);
    bridge_voltage = correction + control->reference_v;
    modulation_raw = bridge_voltage / dc_bus_v;
    if ((!QPR_IsFinite(bridge_voltage)) || (!QPR_IsFinite(modulation_raw)))
    {
        control->fault_flags |= QPR_FAULT_NUMERIC;
        control->modulation = 0.0f;
        return 0.0f;
    }

    /* 条件抗饱和：误差继续推动各自输出饱和时冻结该环谐振状态。
     * 误差历史仍更新，避免恢复计算时使用过期误差。
     */
    if (voltage_error * (voltage_qpr_raw - current_reference) <= 0.0f)
    {
        control->voltage_resonant_r = voltage_resonant_next;
        control->voltage_resonant_q = voltage_quadrature_next;
    }
    control->modulation = QPR_Clamp(modulation_raw, QPR_MODULATION_LIMIT);
    if ((current_error * (qpr_raw - correction) <= 0.0f) &&
        ((modulation_raw == control->modulation) ||
         (current_error * (bridge_voltage - control->modulation * dc_bus_v) <= 0.0f)))
    {
        control->resonant_r = resonant_next;
        control->resonant_q = quadrature_next;
    }
    control->voltage_qpr_unlimited_a = voltage_qpr_raw;
    if (voltage_qpr_raw != current_reference) control->voltage_qpr_saturation_count++;
    control->voltage_error_previous_v = voltage_error;
    control->current_error_previous_a = current_error;
    control->current_reference_a = current_reference;
    control->voltage_error_v = voltage_error;
    control->current_error_a = current_error;
    control->qpr_unlimited_v = qpr_raw;
    control->qpr_correction_v = correction;
    control->bridge_reference_v = bridge_voltage;
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
    control->current_error_previous_a = 0.0f;
    control->resonant_r = 0.0f;
    control->resonant_q = 0.0f;
    control->voltage_resonant_r = 0.0f;
    control->voltage_resonant_q = 0.0f;
    control->voltage_qpr_unlimited_a = 0.0f;
    control->voltage_error_v = 0.0f;
    control->current_reference_a = 0.0f;
    control->current_error_a = 0.0f;
    control->qpr_unlimited_v = 0.0f;
    control->qpr_correction_v = 0.0f;
    control->bridge_reference_v = control->reference_v;
    control->modulation = QPR_Clamp(modulation_raw, QPR_MODULATION_LIMIT);
    if (modulation_raw != control->modulation) control->modulation_saturation_count++;
    return control->modulation;
}
