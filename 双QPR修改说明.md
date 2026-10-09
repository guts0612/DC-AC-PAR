# 双 QPR 闭环修改（2026-10-08）

当前工程：`D:\study\hal\DC-AC-PAR`。

修改前完整备份：`D:\study\hal\backups\DC-AC-PAR_before_dual_qpr_20261008_203345`。
已逐一比对 595 个文件的 SHA-256；编辑器正在写入的 `keil-assistant.log` 已复制，未参与哈希校验。
恢复时先关闭 Keil/编辑器，在另一个目录打开备份中的 `MDK-ARM/dcac.uvprojx` 即可使用修改前工程。

## 控制链

电压参考减实测输出电压 → 电压 QPR → 电流参考限幅 →
减实测滤波电感电流 → 电流 QPR → 校正电压限幅 →
加参考电压前馈 → 除母线电压 → 调制度限幅。

两个环都使用 `G(s)=Kp+2*Kr*Wc*s/(s²+2*Wc*s+W0²)`，
以真实采样频率进行 Tustin 离散化，分别保存系数、误差历史和谐振状态。
原 `1/(Lv*s+Rv)` 外环已移除。外环输出单位 A，内环输出单位 V。
开环运行清理两个环的动态状态，保留参考幅值斜坡；切换不保证校正量无阶跃。
从机继续使用独立电流 QPR。

## 参数入口

`Core/Inc/qpr_control.h`：

| 参数 | 当前值 | 单位 |
| --- | ---: | --- |
| QPR_VOLTAGE_KP_A_PER_V | 0.1 | A/V |
| QPR_VOLTAGE_KR_A_PER_V | 0.5 | A/V |
| QPR_VOLTAGE_WC_RAD_S | 5 | rad/s |
| QPR_CURRENT_REFERENCE_LIMIT_A | 5 | A，峰值指令 |
| QPR_CURRENT_KP_V_PER_A | 1.0 | V/A |
| QPR_CURRENT_KR_V_PER_A | 1.6 | V/A |
| QPR_CURRENT_WC_RAD_S | 5 | rad/s |
| QPR_FREQUENCY_HZ | 50 | Hz |

内环增益、±5 V 校正限幅、0.95 调制度上限、48 V 母线设定、24 Vrms 参考和
60 V/s 峰值斜坡沿用当前工程。外环增益和 ±5 A 电流指令上限是初始调试值，
未依据硬件额定电流或实物频响整定；电流指令限幅不是硬件过流保护。
各环有条件冻结谐振状态以避免继续推动自身输出饱和；外环不根据内环饱和进行跟踪回算。

## 观测与检查

正常 VOFA 七通道顺序保留。均值模式 CH6 从虚拟阻抗残差改为平均电流跟踪误差（A）。
Watch 可观察 `voltage_qpr_unlimited_a`、`voltage_qpr_saturation_count`、
`current_reference_a`、`qpr_unlimited_v`、`qpr_saturation_count`、`modulation_saturation_count`。

`Tests/verify_dual_qpr.py` 直接编译执行控制 C，独立比较两环在 49/50/51 Hz
的解析 Tustin 频率响应，检查指令/校正限幅、条件抗饱和、复位、开环状态清理及故障锁存。
从机、VOFA 均值和 JustFloat 帧回归检查仍可运行。旧虚拟阻抗测试及旧结果保留为历史资料，
不适用于当前算法。

以上属于编译及主机数值验证，未进行闭环对象仿真、烧录、示波器测试或 MCU 中断执行时间测量，
不能据此认定实际负载下稳定。后续应在确认反馈极性/比例及电流额定值后，从低参考幅值调试。
