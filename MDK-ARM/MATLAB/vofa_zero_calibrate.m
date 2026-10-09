function result = vofa_zero_calibrate(csvFile, varargin)
%VOFA_ZERO_CALIBRATE DC-AC-PAR 工程 CH2/CH4/CH7 三通道零点校正。
% MATLAB R2020b+，无需额外工具箱。无参数运行时弹窗选择 CSV。
%   r = vofa_zero_calibrate('D:\download\vofa\0v.csv');
%   r = vofa_zero_calibrate('0v.csv', 'SampleRange', [1001 40000]);
%   r = vofa_zero_calibrate('0v.csv', ...
%       'OldZero', [0 0 0]);
%
% 参数顺序始终为 [CH2 电压, CH4 主电流, CH7 辅助电流]。
% OldZero：采集时实际扣除的输出侧零偏，单位分别为 V / A / A。
% 默认 [0 0 0] 适用于本次原始零输入 CSV，不代表当前源码宏值。
% SampleRange：CSV 数据行号 [起点 终点]，不含表头；空表示全部。
% Plot：是否绘图并保存 PNG（默认 true）。OutputDir：结果保存目录。
%
% 当前工程公式（零偏在乘增益之后扣除）：
%   CH2 = 负号处理后的 ADC 电压 * voltage_gain - VOLTAGE_ZERO_V;
%   CH4 = ADC 电压 * cksr6_current_gain - CURRENT_ZERO_V;
%   CH7 = ADC 电压 * aux_current_gain - AUX_CURRENT_ZERO_V;
% 所以：新输出侧零偏 = 采集时旧输出侧零偏 + mean(CSV 通道)。
% CSV 已乘增益，不再乘或除增益。两个电流宏的实际单位为 A。
% CH2 的负号已经在固件中处理；这里不能再额外取反。
% CH2=I1 [V]，CH4=I3 [A]，CH7=I6 [A]。采集时均值模式必须关闭。
% 必须在实际 0 V / 两路 0 A 下采集；零偏校正不校正增益或噪声。

if nargin < 1 || isempty(csvFile)
    [name, folder] = uigetfile('*.csv', '选择 CH2/CH4/CH7 零输入数据');
    if isequal(name, 0)
        result = [];
        return;
    end
    csvFile = fullfile(folder, name);
end
p = inputParser;
p.addRequired('csvFile', @(x) ischar(x) || (isstring(x) && isscalar(x)));
threeValues = @(x) isnumeric(x) && isreal(x) && numel(x) == 3 && all(isfinite(x(:)));
p.addParameter('OldZero', [0 0 0], threeValues);
p.addParameter('SampleRange', [], @(x) isnumeric(x) && isreal(x) && ...
    (isempty(x) || (numel(x) == 2 && all(isfinite(x(:))) && ...
    all(x(:) >= 1) && all(x(:) == fix(x(:))) && x(1) <= x(2))));
p.addParameter('Plot', true, @(x) islogical(x) && isscalar(x));
p.addParameter('OutputDir', fullfile(fileparts(mfilename('fullpath')), 'results'), ...
    @(x) ischar(x) || (isstring(x) && isscalar(x)));
p.parse(csvFile, varargin{:});
cfg = p.Results;
oldZero = double(reshape(cfg.OldZero, 1, 3));
if ~isfile(csvFile)
    error('vofa_zero_calibrate:MissingFile', '文件不存在：%s', csvFile);
end
data = readtable(csvFile, 'VariableNamingRule', 'preserve');
columns = {'I1', 'I3', 'I6'};
if ~all(ismember(columns, data.Properties.VariableNames))
    error('vofa_zero_calibrate:Channels', 'CSV 必须包含 I1、I3、I6（CH2、CH4、CH7）。');
end
for c = 1:3
    if ~isnumeric(data.(columns{c}))
        error('vofa_zero_calibrate:NonNumeric', '%s 必须为数值列。', columns{c});
    end
end
raw = double(data{:, columns});
n = height(data);
range = cfg.SampleRange;
if isempty(range)
    range = [1 n];
end
if n == 0 || range(2) > n
    error('vofa_zero_calibrate:Range', '采样区间超出 %d 行数据。', n);
end
selected = (range(1):range(2)).';
valid = all(isfinite(raw(selected, :)), 2);
rows = selected(valid);
if numel(rows) < 2
    error('vofa_zero_calibrate:TooFewSamples', '至少需要两组三通道有效数据。');
end
if any(~valid)
    warning('vofa_zero_calibrate:InvalidRows', '已排除 %d 行 NaN/Inf 数据。', sum(~valid));
end
if numel(rows) < 1000
    warning('vofa_zero_calibrate:ShortCapture', '有效点数少于 1000，建议延长采集。');
end
samples = raw(rows, :);
residual = mean(samples, 1);       % 输出侧有符号均值，单位 V / A / A
increment = residual;            % 零偏位于输出侧，直接使用 V / A / A 均值
newZero = oldZero + increment;
corrected = raw - residual;       % CSV 已扣过旧零偏，只减本次输出侧残余
mid = floor(numel(rows) / 2);
drift = mean(samples(mid+1:end, :), 1) - mean(samples(1:mid, :), 1);
macroNames = {'VOLTAGE_ZERO_V', 'CURRENT_ZERO_V', 'AUX_CURRENT_ZERO_V'};
channelNames = {'CH2', 'CH4', 'CH7'};
units = {'V', 'A', 'A'};
summary = table(channelNames.', columns.', units.', oldZero.', ...
    residual.', increment.', newZero.', std(samples, 0, 1).', ...
    (max(samples, [], 1)-min(samples, [], 1)).', drift.', ...
    'VariableNames', {'Channel','CSVColumn','OutputUnit','OldZeroOutput', ...
    'ResidualOutput','ZeroIncrementOutput','NewZeroOutput','StdOutput', ...
    'PeakToPeakOutput','HalfDriftOutput'});
disp(summary);
report = sprintf(['DC-AC-PAR three-channel zero calibration\nSource: %s\n' ...
    'Rows: %d:%d; valid: %d; excluded NaN/Inf: %d\n' ...
    'Formula: output = ADC_voltage * gain - zero_output\n' ...
    'New zero_output = old zero_output + mean(output); units: V / A / A\n\n'], ...
    char(csvFile), range(1), range(2), numel(rows), sum(~valid));
for c = 1:3
    report = [report sprintf([ ...
        '%s / %s: old zero=%.12g %s\n' ...
        '  Residual mean=%+.12g %s; zero increment=%+.12g %s\n' ...
        '  Std=%.9g %s; peak-to-peak=%.9g %s; half drift=%+.9g %s\n' ...
        '  Corrected calibration mean=%.9g %s\n'], ...
        channelNames{c}, columns{c}, oldZero(c), units{c}, residual(c), units{c}, ...
        increment(c), units{c}, std(samples(:,c)), units{c}, ...
        max(samples(:,c))-min(samples(:,c)), units{c}, drift(c), units{c}, ...
        mean(corrected(rows,c)), units{c})]; %#ok<AGROW>
end
report = [report sprintf('\nReplace these macros (CH2 in V; CH4/CH7 in A):\n')];
for c = 1:3
    report = [report sprintf('#define %s (%.9ff)\n', macroNames{c}, newZero(c))]; %#ok<AGROW>
end
report = [report sprintf(['\nCollect with VOFA_MEAN_OUTPUT_ENABLE = 0.\n' ...
    'Calibration requires actual 0 V and both currents at 0 A.\n' ...
    'OldZero must be the output offset actually subtracted during this capture.\n' ...
    'After programming new offsets, verify with an independent zero-input capture.\n'])];
fprintf('%s', report);

result.source_csv = char(csvFile);
result.sample_range = range;
result.valid_samples = numel(rows);
result.excluded_rows = sum(~valid);
result.calibration_row_indices = rows;
result.summary = summary;
result.old_zero_output = oldZero;
result.residual_output = residual;
result.zero_increment_output = increment;
result.new_zero_output = newZero;
result.voltage_corrected_v = corrected(:,1);
result.current_corrected_a = corrected(:,2);
result.aux_current_corrected_a = corrected(:,3);
result.report = report;
outputDir = char(cfg.OutputDir);
if ~isfolder(outputDir)
    [ok, message] = mkdir(outputDir);
    if ~ok, error('vofa_zero_calibrate:OutputDir', '%s', message); end
end
[~, stem] = fileparts(csvFile);
tag = [stem '_three_output_zero_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'))];
result.mat_file = fullfile(outputDir, [tag '.mat']);
result.report_file = fullfile(outputDir, [tag '.txt']);
result.plot_file = '';
if cfg.Plot
    stride = max(1, ceil(numel(rows) / 10000));
    shown = 1:stride:numel(rows);   % 仅绘图抽点，计算使用所有有效点
    fig = figure('Name', 'CH2 / CH4 / CH7 零点校正', ...
        'Color', 'w', 'Position', [100 100 1100 850]);
    for c = 1:3
        subplot(3, 1, c);
        plot(rows(shown), samples(shown,c), '-', ...
            rows(shown), corrected(rows(shown),c), '-');
        yline(0, 'k:'); grid on;
        xlabel('CSV 数据行号'); ylabel(['测量值 / ' units{c}]);
        title(sprintf('%s / %s：残余均值 %+.6f %s，新零偏 %+.9f %s', ...
            channelNames{c}, columns{c}, residual(c), units{c}, newZero(c), units{c}));
        legend('校正前', '校正后', '零线', 'Location', 'best');
    end
    result.plot_file = fullfile(outputDir, [tag '.png']);
    exportgraphics(fig, result.plot_file, 'Resolution', 150);
end
fid = fopen(result.report_file, 'w', 'n', 'UTF-8');
if fid < 0, error('vofa_zero_calibrate:WriteReport', '无法写入报告。'); end
closeFile = onCleanup(@() fclose(fid));
fprintf(fid, '%s', report);
clear closeFile;
save(result.mat_file, 'result');
fprintf('\n报告：%s\n数据：%s\n', result.report_file, result.mat_file);
end
