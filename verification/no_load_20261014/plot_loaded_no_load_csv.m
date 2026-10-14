% Read-only scientific plots of supplied VOFA data. Original CSV is untouched.
sourcePath = 'D:\download\vofa\带载和空载对比.csv';
outputPath = 'D:\study\hal\DC-AC-PAR\verification\no_load_20261014';
data = readmatrix(sourcePath, 'NumHeaderLines', 1);
assert(size(data, 2) == 8 && size(data, 1) == 49802);
assert(all(isfinite(data), 'all'));
fsTelemetry = 2000;
n = size(data, 1);
time = (0:n-1)' / fsTelemetry;
railPositive = 38.34375;
railNegative = -38.4375;
rails = abs(data(:,2)-railPositive)<1e-5 | abs(data(:,2)-railNegative)<1e-5;
set(groot,'defaultAxesFontName','Microsoft YaHei');
set(groot,'defaultTextFontName','Microsoft YaHei');
set(groot,'defaultAxesFontSize',12);
set(groot,'defaultLineLineWidth',1.25);
loadedColor = [0.10 0.45 0.72];
emptyColor = [0.83 0.29 0.15];

figureOverview = figure('Visible','off','Color','white', ...
    'Position',[100 100 1260 980]);
layout = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
title(layout,'12 Ω 带载 → 空载：实测记录对比', 'FontSize',19,'FontWeight','bold');
subtitle(layout,'主机闭环、从机关断；时间由约 2 kHz 遥测帧率估计', 'FontSize',12);
ax1 = nexttile;
plot(time,data(:,8),'Color',loadedColor);
hold on;
yline(24,'--','24 V 目标','Color',[0.25 0.25 0.25], ...
    'LabelHorizontalAlignment','left');
ylabel('通道 8 有效值 / V');
ylim([23.9 25.15]);
xlim([0 time(end)]);
grid on;
shadeSegments(ax1);
xline(13.3,'--','卸载约 13.3 s','Color',emptyColor,'LabelVerticalAlignment','top');
text(6,25.02,sprintf('带载稳定段：23.976 V，标准差 0.0074 V'), ...
    'HorizontalAlignment','center','Color',loadedColor,'FontSize',12);
text(19,25.02,sprintf('空载稳定段：24.276 V，标准差 0.1226 V'), ...
    'HorizontalAlignment','center','Color',emptyColor,'FontSize',12);

block = 40;
count = floor(n/block);
masterCurrent = reshape(data(1:count*block,4),block,count);
currentRms = sqrt(mean(masterCurrent.^2,1));
currentTime = ((0:count-1)*block+(block-1)/2)/fsTelemetry;
ax2 = nexttile;
plot(currentTime,currentRms,'Color',loadedColor);
ylabel('主电感电流 RMS / A');
xlim([0 time(end)]);
ylim([0 2.6]);
grid on;
shadeSegments(ax2);
xline(13.3,'--','Color',emptyColor);
text(24,2.42,'每 40 个遥测点合成（约 20 ms）', ...
    'HorizontalAlignment','right','FontSize',11,'Color',[.35 .35 .35]);

railBlock = 400;
railCount = floor(n/railBlock);
railBins = reshape(rails(1:railCount*railBlock),railBlock,railCount);
railPercent = 100*mean(railBins,1);
railTime = ((0:railCount-1)*railBlock+(railBlock-1)/2)/fsTelemetry;
ax3 = nexttile;
stairs(railTime,railPercent,'Color',emptyColor);
ylabel('电压采样触轨比例 / %');
xlabel('估计时间 / s');
xlim([0 time(end)]);
ylim([0 max(4,ceil(max(railPercent))+0.5)]);
grid on;
shadeSegments(ax3);
xline(13.3,'--','Color',emptyColor);
text(24,ax3.YLim(2)*0.9,'每 400 个遥测点统计（约 200 ms）', ...
    'HorizontalAlignment','right','FontSize',11,'Color',[.35 .35 .35]);
annotation(figureOverview,'textbox',[.05 .003 .90 .03], ...
    'String','浅蓝/浅橙色区域为稳定取段；通道 8 来自 20 kHz 全速采样。波形遥测抽点到约 2 kHz，高频频谱会混叠。', ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',10);
layout.OuterPosition = [.025 .055 .95 .93];
exportgraphics(figureOverview,fullfile(outputPath,'loaded_no_load_capture_overview.png'),'Resolution',170);
close(figureOverview);

windowLength = 120;
loadedStart = 10001;
emptyCandidates = 28001:48000-windowLength+1;
railDensity = conv(double(rails),ones(windowLength,1),'valid');
[~,maximumIndex] = max(railDensity(emptyCandidates));
emptyStart = emptyCandidates(maximumIndex);
loadedIndices = loadedStart:loadedStart+windowLength-1;
emptyIndices = emptyStart:emptyStart+windowLength-1;
relativeTime = (0:windowLength-1)*1000/fsTelemetry;

figureWaveforms = figure('Visible','off','Color','white', ...
    'Position',[100 100 1400 930]);
layoutWave = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
title(layoutWave,'带载与空载：约 3 个工频周期的采样波形', 'FontSize',19,'FontWeight','bold');
subtitle(layoutWave,'空载选取截顶明显的片段；曲线为约 2 kHz 遥测点，不能据此唯一判定实际高频振荡频率', ...
    'FontSize',12);

axLoadedVoltage = nexttile;
plot(relativeTime,data(loadedIndices,1),'--','Color',[.25 .25 .25]);
hold on;
plot(relativeTime,data(loadedIndices,2),'Color',loadedColor);
title(sprintf('12 Ω 带载（约 %.3f–%.3f s）',time(loadedIndices(1)),time(loadedIndices(end))));
ylabel('电压 / V');
ylim([-43 43]);
xlim([0 60]);
grid on;
legend('电压参考','电压反馈','Location','southoutside','Orientation','horizontal');

axEmptyVoltage = nexttile;
plot(relativeTime,data(emptyIndices,1),'--','Color',[.25 .25 .25]);
hold on;
plot(relativeTime,data(emptyIndices,2),'Color',emptyColor);
railSlice = rails(emptyIndices);
plot(relativeTime(railSlice),data(emptyIndices(railSlice),2),'o', ...
    'Color',[.65 0 0],'MarkerFaceColor',[.65 0 0],'MarkerSize',5);
yline(railPositive,':','ADC 上端点','Color',[.55 .25 .25], ...
    'LabelHorizontalAlignment','left','FontSize',10);
yline(railNegative,':','ADC 下端点','Color',[.55 .25 .25], ...
    'LabelHorizontalAlignment','left','FontSize',10);
title(sprintf('空载（约 %.3f–%.3f s；%d 个触轨点）', ...
    time(emptyIndices(1)),time(emptyIndices(end)),sum(railSlice)));
ylabel('电压 / V');
ylim([-43 43]);
xlim([0 60]);
grid on;
legend('电压参考','电压反馈','ADC 触轨点','Location','southoutside','Orientation','horizontal');

axLoadedCurrent = nexttile;
plot(relativeTime,data(loadedIndices,3),'--','Color',[.25 .25 .25]);
hold on;
plot(relativeTime,data(loadedIndices,4),'Color',loadedColor);
title('带载：电流参考与主电感实际电流');
ylabel('电流 / A');
xlabel('片段内时间 / ms');
xlim([0 60]);
limitLoaded = max(abs(data(loadedIndices,3:4)),[],'all')*1.10;
ylim([-limitLoaded limitLoaded]);
grid on;
legend('电流参考','实际主电感电流','Location','southoutside','Orientation','horizontal');

axEmptyCurrent = nexttile;
plot(relativeTime,data(emptyIndices,3),'--','Color',[.25 .25 .25]);
hold on;
plot(relativeTime,data(emptyIndices,4),'Color',emptyColor);
title('空载：电流存在明显额外波动');
ylabel('电流 / A');
xlabel('片段内时间 / ms');
xlim([0 60]);
limitEmpty = max(abs(data(emptyIndices,3:4)),[],'all')*1.12;
ylim([-limitEmpty limitEmpty]);
grid on;
legend('电流参考','实际主电感电流','Location','southoutside','Orientation','horizontal');
annotation(figureWaveforms,'textbox',[.05 .003 .90 .027], ...
    'String','空载电压反馈触及 +38.34375 V / −38.43750 V，分别对应 ADC raw=0 / 4095；触轨以外的真实幅值无法从记录恢复。', ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',10);
layoutWave.OuterPosition = [.025 .05 .95 .935];
exportgraphics(figureWaveforms,fullfile(outputPath,'loaded_no_load_waveforms.png'),'Resolution',170);
close(figureWaveforms);
fprintf('Plots exported. Loaded start zero-based: %d. Empty start zero-based: %d. Empty rail count: %d.\n', ...
    loadedStart-1,emptyStart-1,sum(railSlice));

function shadeSegments(ax)
    limits = ylim(ax);
    hold(ax,'on');
    h1 = patch(ax,[2.5 13 13 2.5],[limits(1) limits(1) limits(2) limits(2)], ...
        [.65 .83 .94],'EdgeColor','none','FaceAlpha',.12,'HandleVisibility','off');
    h2 = patch(ax,[14 24 24 14],[limits(1) limits(1) limits(2) limits(2)], ...
        [.97 .76 .61],'EdgeColor','none','FaceAlpha',.12,'HandleVisibility','off');
    uistack([h1 h2],'bottom');
end
