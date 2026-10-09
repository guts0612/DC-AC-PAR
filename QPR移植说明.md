# dcac 开闭环切换与 RMS 调压说明

更新：2026-09-26。

## 当前设定

最新工作点为 **48 V 直流母线、32 V RMS、50 Hz**，沿用已确认的 **220 µH、2.2 µF、25 Ω带载**，不要求空载。220 µH未区分单只还是差模总值，数值验证同时覆盖总电感220 µH和440 µH。

默认上电启用闭环，参考有效值由0以64 V/s上升，约0.5秒到32 V RMS。32 V RMS对应峰值约45.2548 V，25 Ω下理想负载电流为1.28 A RMS、功率40.96 W。

**旧的 `voltage_reference_peak_v` 已移除，用户调压接口统一使用 RMS。** 内部瞬时参考为 `sqrt(2) × RMS斜坡值 × sin(相位)`。没有把瞬时ADC反馈改成RMS闭环；原电压外环和电流内环仍使用瞬时物理量。

## 控制变量与未来按键接口

命令变量定义在 `Core/Src/inverter_control.c`，声明在 `Core/Inc/inverter_control.h`。

| 变量 | 初值 | 含义 |
|---|---:|---|
| `voltage_reference_rms_v` | 32.0 | 目标有效值，单位V RMS |
| `dc_bus_voltage_v` | 48.0 | 母线设定值，不是实时测量值 |
| `voltage_closed_loop_enable` | 1 | 0=开环正弦，1=QPR闭环 |
| `inverter_output_enable` | 1 | 0=清状态并保持零调制度，1=运行所选模式 |

**开环不是停止输出。** 上一版的 `voltage_closed_loop_enable=0` 是暂停，现在它会继续输出开环正弦。暂停改用独立的 `inverter_output_enable=0`。零调制度仍为两桥臂相同的50%互补PWM，不是关断栅极。

未来按键扫描、消抖和事件处理在主循环中调用以下接口：

```c
#include "inverter_control.h"

Inverter_SetClosedLoop(0U);        /* 切到开环 */
Inverter_SetClosedLoop(1U);        /* 切到闭环 */
Inverter_SetVoltageRms(32.0f);     /* 设置32 V RMS */
Inverter_AdjustVoltageRms(1.0f);   /* 加1 V RMS */
Inverter_AdjustVoltageRms(-1.0f);  /* 减1 V RMS */
Inverter_SetOutputEnable(0U);      /* 暂停，m=0，不关栅极 */
Inverter_SetOutputEnable(1U);      /* 从零幅值重新软启动 */
```

这些是独立用法示例，不要把整段顺序粘进主循环。`SetVoltageRms/AdjustVoltageRms`对有限目标钳位到0～32 V RMS；NaN、Inf或计算溢出返回0且不写命令，正常接受返回1（值可能已经钳位）。步进由调用参数决定，也可用0.1f、0.5f。

接口只更新命令，控制状态和PWM由ADC回调统一更新。约定读改写接口在主循环调用，不从多个上下文并发调用 `AdjustVoltageRms()`。本次没有启动 `Hardware/key.c` 扫描，也没有分配PB14/PB15/PB12的按键含义。

## 开闭环与切换

两种模式共用RMS斜坡和持续的50 Hz相位。

稳定开环：`m = sqrt(2) × RMS斜坡 × sin(相位) / Vdc`。不使用反馈修正调制度，所以开环设定32 V RMS不保证实测恰好32 V RMS；采样和遥测仍运行。

闭环保持原 `singlephase.slx` 连接：

```text
e_v = v_ref - v_out
i_ref = [1 / (0.001s + 0.01)] × e_v
e_i = i_ref - i_L
u_qpr = clamp(QPR(e_i), -48 V, +48 V)
m = (v_ref + u_qpr) / Vdc
```

- **开→闭：** 预置虚拟阻抗和谐振状态，使第一拍电流误差约为零、校正量接续上一拍实际值，然后正常闭环计算。不清相位、不重启RMS斜坡。
- **闭→开：** 保留上一拍实际校正电压，在20 ms内线性撤去；过渡期间不再用反馈产生新校正，最终只有开环前馈。
- **改RMS：** 两模式均以64 V RMS/s调整幅值。这是幅值斜坡，不限制正弦瞬时值的自然变化。
- **暂停后再启用：** 清控制状态和幅值斜坡，相位继续推进，再启用从零幅值软启动。

这些处理减少切换额外突变，不代表实物已证明完全无冲击。故障锁存判断先于正常暂停复位；开关输出使能不能绕过故障。主程序发现故障后进入 `Error_Handler()` 禁用四路输出并停住，需要复位。

## 限幅与硬件余量

旧±0.90在48 V下的理想上限约30.55 V RMS，达不到32 V RMS。本版改为 **±0.96**：

```text
所需调制度 ≈ 32 × sqrt(2) / 48 = 0.942809
理想RMS上限 ≈ 48 × 0.96 / sqrt(2) = 32.5835 V
```

峰值调节余量约0.825 V，实际死区、压降和母线下跌会消耗它。`INVERTER_MAX_REFERENCE_RMS_V=32`是本次按键/UI设定上限；扩展时需重新核对功率级。

每拍还按 `min(32, 0.96 × Vdc / sqrt(2))` 限制有效参考，`inverter_control.reference_limited`标记目标被限制。母线降低时可用上限立即收窄。目前没有母线ADC，此计算使用用户设定的 `dc_bus_voltage_v`，不能自动检测实际母线跌落。

±0.96处HRTIM比较值为85和4165，超过原6 tick端点余量；20 kHz下最短理想命令脉冲约1 µs，配置死区约147 ns。计数核查不等于驱动器/MOSFET最小脉宽或自举能力验证。

## QPR参数与文件

`Core/Inc/qpr_control.h`中保留：Kp=2、Kr=20、Wc=5 rad/s、W0=2π×50 rad/s，虚拟阻抗 `1/(0.001s+0.01)`，采样50 µs。虚拟电感0.001 H是算法参数，不等于实物220 µH。

QPR保持原MATLAB Function的Tustin传递函数，使用等价增量双状态形式减少float误差；初始化系数用double、实时计算用float。±48 V校正限幅在QPR之后，状态保存未饱和值。虚拟阻抗原为连续模块，MCU采用Tustin离散。

RMS、斜坡和切换已移到 `inverter_control.c`；`QPR_ControlStep()`现在直接接收瞬时参考，`QPR_ControlTrack()`仅在开转闭时配平状态。

| 文件 | 职责 |
|---|---|
| `Core/Inc/inverter_control.h` | RMS/切换配置、未来按键接口 |
| `Core/Src/inverter_control.c` | 命令、共用斜坡、开闭环与接续 |
| `Core/Inc/qpr_control.h` | QPR/虚拟阻抗配置 |
| `Core/Src/qpr_control.c` | 双环数值算法和状态预置 |
| `Core/Src/main.c` | 原ADC校准、持续相位、唯一PWM提交、遥测 |
| `MDK-ARM/dcac.uvprojx` | 已加入两个控制源文件 |
| `Tests/verify_qpr.py` | 实际C算法、切换及平均LC验证 |

## Watch与VOFA

| 变量 | 含义 |
|---|---|
| `voltage_reference_rms_v` | 请求的有效值 |
| `inverter_control.reference_rms_v` | 当前斜坡有效值，非实测RMS |
| `voltage_reference_v` | 瞬时参考电压 |
| `true_voltage` / `true_current` | 瞬时实际电压/电感电流 |
| `inverter_control.closed_loop_active` | 回调已应用的模式 |
| `inverter_control.reference_limited` | 参考目标受到限制 |
| `inverter_control.qpr.current_reference_a` | 电感电流指令 |
| `inverter_control.qpr.voltage_error_v` / `current_error_a` | 两环误差 |
| `inverter_control.qpr.qpr_unlimited_v` / `qpr_correction_v` | QPR限幅前/后校正 |
| `modulation_command` | 最终实际提交的调制度 |
| `inverter_control.modulation_saturation_count` | 两模式下总调制度限幅次数 |
| `inverter_control.qpr.qpr_saturation_count` | QPR校正限幅次数 |
| `inverter_control.fault_flags` | bit0配置、bit1输入、bit2数值异常 |

VOFA保持UART4、115200、8N1、JustFloat、500帧/s：CH1瞬时参考电压(V)，CH2瞬时实际电压(V)，CH3电感电流(A)。设定改为RMS，不代表三个遥测通道改成RMS测量。

## 验证与边界

修改前文件备份位于 `.codex-backups/before-rms-mode-20260926-150306/`。没有修改模型，没有烧录MCU。

Keil结果见 `MDK-ARM/qpr_build.log`；数值结果见 `Tests/results/qpr_verification.json`，重现方式见 `Tests/README.md`。测试直接编译实际C文件，覆盖原QPR精度、RMS换算、斜坡、目标边界、开闭环切换、暂停重启、数值异常，以及带一拍50 µs延迟的理想LC平均模型。

该验证不是原Simulink开关模型仿真，也不是实机反馈极性、开关纹波、驱动能力或中断最坏耗时测试。原增益不自动保证轻载/空载稳定；本次使用范围为25 Ω。首次上板降低RMS目标核对反馈与波形后，再升到32 V RMS，并关注限幅次数和实际母线电压。

数值检查和调制度限幅不构成硬件过流保护。`Error_Handler()`关闭四路输出；ADC错误回调只处理已上报错误，ADC1_2 IRQ当前未启用，不能据此声称覆盖全部overrun或HardFault。CubeMX重新生成后需检查Keil仍包含两个控制源文件。
