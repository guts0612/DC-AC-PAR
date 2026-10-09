# DC-AC-PAR 三通道零点校正（乘增益后扣零偏）

适配修改后的 `Core/Src/main.c`：三个通道都是 `true = adc_sample * gain - ZERO`。CH2、CH4、CH7 对应 CSV 的 `I1`、`I3`、`I6`，单位分别为 V、A、A。CSV 已乘增益，均值不再乘或除增益。两个电流零偏宏虽然带 `_V`，实际单位是 A。

## 运行

```matlab
cd('D:/study/hal/DC-AC-PAR/MDK-ARM/MATLAB');
clear vofa_zero_calibrate;
result = vofa_zero_calibrate('D:/download/vofa/0v.csv');
```

默认 `OldZero=[0 0 0]`，用于本次提供的原始零输入 CSV。这不是自动读取当前源码宏值：当前宏值可能在采集后才修改，不能直接作为历史采集参数累加。

新零偏 = 采集时实际扣除的输出侧旧零偏 + CSV 有符号均值。

本次 49,802 点原始零输入数据的三个输出侧零偏为：

```c
#define VOLTAGE_ZERO_V (0.657916399f)
#define CURRENT_ZERO_V (-0.024201973f)
#define AUX_CURRENT_ZERO_V (0.033564952f)
```

将报告中的宏手动替换到 `main.c` 后编译、下载。脚本不修改固件。

## 重复校正

若新 CSV 在乘增益后扣零偏的公式下采集，且已启用过校正，应填写采集时的旧零偏：

```matlab
result = vofa_zero_calibrate('second_zero.csv', ...
    'OldZero', [0.657916399 -0.024201973 0.033564952], ...
    'SampleRange', [1001 40000]);
```

顺序始终为 `[CH2 CH4 CH7]`，单位分别为 V、A、A。若 CSV 是在旧版乘增益前扣零偏公式下采集，则 `OldZero` 应填写那次 ADC 端旧零偏乘以那次增益后的值；旧 ADC 端零偏为 0 时仍填写 0。不要把乘增益前的 ADC 端零偏直接传给此版本。

本版本使用参数 `OldZero`，移除旧版 `Gains`、`OldZeroV`，避免单位混用。

## 结果

`results/` 保存带时间戳的 TXT、MAT 和三通道校正前后 PNG。新版文件名包含 `three_output_zero`。此前 `three_zero` 文件是乘增益前的 ADC 域结果，不能用于当前公式。

- `result.new_zero_output`：三个新的固件零偏，单位 V、A、A。
- `result.residual_output`：本次残余均值，单位 V、A、A。
- `result.voltage_corrected_v`、`result.current_corrected_a`、`result.aux_current_corrected_a`：原文件减去固定残余均值后的数据。

支持 `'Plot', false`、`'OutputDir', '目录'` 和 `'SampleRange', [起点 终点]`。行号不含表头；默认使用全部有效数据。含 NaN/Inf 的三通道数据行会排除并报告。统计包含标准差、峰峰值和前后半段均值差。

采集时保持 `VOFA_MEAN_OUTPUT_ENABLE=0`，测量端实际为 0 V、两路电流实际为 0 A，采样电路供电并预热稳定。校正消除平均偏置，不消除噪声、纹波或增益误差。下载新参数后重新采集独立零输入数据验证；已使用新零偏固件的数据不应再重复扣除这次残余均值。
