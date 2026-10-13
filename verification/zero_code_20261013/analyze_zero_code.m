function analyze_zero_code(csvPath, outDir)
% MATLAB analysis of powered analog front-end zero capture. Source CSV is read only.
if nargin < 1
    csvPath = 'D:\download\vofa\对源码的0点矫正.csv';
end
if nargin < 2
    outDir = fileparts(mfilename('fullpath'));
end
if ~exist(outDir,'dir'), mkdir(outDir); end
data = readmatrix(csvPath, 'NumHeaderLines', 1);
assert(size(data,2) == 7, 'Expected seven VOFA columns.');
assert(all(isfinite(data(:))), 'Missing/nonfinite data: inspect before calibration.');
columns = [2 4 6 7];
names = {'ADC1 voltage','ADC2 current','ADC4 auxiliary current','ADC3 DC bus'};
macros = {'VOLTAGE_ZERO_CODE','CURRENT_ZERO_CODE','AUX_CURRENT_ZERO_CODE','DC_BUS_ZERO_CODE'};
x = data(:,columns);
assert(all(x(:) == round(x(:))) && all(x(:)>=-2048 & x(:)<=2047), ...
    'Capture is not valid centered 12-bit ADC integer codes.');
n = size(x,1);
mu = mean(x,1);
sigma = std(x,0,1);
med = median(x,1);
q = sort(x,1);
percentiles = q(max(1,min(n,round([0.01 0.05 0.95 0.99]*n))),:);
zeroCode = round(mu);
corrected = x-zeroCode;
blocks = 20;
edges = round(linspace(0,n,blocks+1));
blockMeans = zeros(blocks,4);
blockStd = zeros(blocks,4);
for b = 1:blocks
    idx = edges(b)+1:edges(b+1);
    blockMeans(b,:) = mean(x(idx,:),1);
    blockStd(b,:) = std(x(idx,:),0,1);
end
report = struct('source',csvPath,'matlab_version',version,'samples',n, ...
    'csv_columns',columns,'channels',{names},'macro_names',{macros}, ...
    'mean_code',mu,'median_code',med,'std_code',sigma,'min_code',min(x), ...
    'max_code',max(x),'percentiles_1_5_95_99',percentiles, ...
    'integer_offset',zeroCode,'corrected_mean_code',mean(corrected), ...
    'centered_input_mV_per_code',3000/2048, ...
    'block_means',blockMeans,'block_std',blockStd,'correlation',corrcoef(x), ...
    'fullscale_hits',sum(x==-2048|x==2047,1), ...
    'note','Offsets are valid only if physical signal is zero. DC bus common-mode compliance must be checked separately.');
window = 0.5-0.5*cos(2*pi*(0:n-1)'/(n-1));
spectrum = abs(fft((x-mu).*window));
positive = 2:floor(n/2);
peakBins = zeros(1,4);
for j=1:4
    [~, peak] = max(spectrum(positive,j));
    peakBins(j) = positive(peak)-1;
end
report.dominant_cycles_per_recorded_sample = peakBins/n;
report.dominant_Hz_if_exact_2000_samples_per_second = peakBins/n*2000;
report.nominal_frequency_caution = 'CSV has no timestamps. Frequencies in Hz assume exact 2 kHz delivery with no missing samples.';
fprintf('Dominant cycles per recorded sample:\n');disp(peakBins/n);
fprintf('Indicative Hz ONLY if exact 2 kHz capture:\n');disp(peakBins/n*2000);
fid = fopen(fullfile(outDir,'zero_code_results.json'),'w','n','UTF-8');
assert(fid>=0);fwrite(fid,jsonencode(report),'char');fclose(fid);
save(fullfile(outDir,'zero_code_analysis.mat'),'report','blockMeans','blockStd');
fid = fopen(fullfile(outDir,'zero_code_results.txt'),'w','n','UTF-8');
assert(fid>=0);
fprintf(fid,'Source: %s\nMATLAB: %s\nSamples: %d\n',csvPath,version,n);
for j=1:4
    line = sprintf('%s: mean=%.9f median=%.1f std=%.6f min=%g max=%g offset=%d residual=%.9f', ...
        names{j},mu(j),med(j),sigma(j),min(x(:,j)),max(x(:,j)),zeroCode(j),mean(corrected(:,j)));
    fprintf('%s\n',line);fprintf(fid,'%s\n',line);
    fprintf(fid,'#define %s (%d)\n',macros{j},zeroCode(j));
end
fprintf(fid,'Offsets are candidates until all physical inputs are confirmed zero.\n');
fclose(fid);
fprintf('Samples=%d; correlation matrix:\n',n);disp(corrcoef(x));
fprintf('Block mean minima/maxima:\n');disp([min(blockMeans);max(blockMeans)]);
% Deterministic symptom check on the supplied captured trace, not a hardware reproduction.
ratio = sigma(2:3)/sigma(1);
fprintf('Captured current/voltage noise ratios: %.3f %.3f\n',ratio);
fprintf('The recording alone cannot isolate ADC role from differences in the analog input circuits.\n');
f = figure('Visible','off','Color','w','Position',[100 100 1400 1100]);
layout = tiledlayout(4,2,'Padding','compact','TileSpacing','compact');
colors = lines(4);
displayIdx = unique(round(linspace(1,n,min(n,5000))));
for j=1:4
    nexttile;
    plot(displayIdx,x(displayIdx,j),'Color',colors(j,:));hold on;
    yline(mu(j),'k-','Mean');grid on;
    title(sprintf('%s | mean %.3f, std %.3f codes',names{j},mu(j),sigma(j)));
    xlabel('Recorded sample index');ylabel('ADC raw - 2048 (code)');
    nexttile;
    histogram(x(:,j),'BinMethod','integers','FaceColor',colors(j,:));
    xline(mu(j),'k-','Mean');grid on;
    xlabel('ADC raw - 2048 (code)');ylabel('Sample count');
end
title(layout,'Powered front-end zero capture: trace and code distribution');
exportgraphics(f,fullfile(outDir,'zero_code_distribution.png'),'Resolution',150);
close(f);
f = figure('Visible','off','Color','w','Position',[100 100 1250 620]);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile;
zoomIdx=1:min(n,400);
plot(zoomIdx,x(zoomIdx,2),'LineWidth',1);hold on;
plot(zoomIdx,x(zoomIdx,3),'LineWidth',1);grid on;
legend('ADC2 current','ADC4 auxiliary current','Location','best');
xlabel('Recorded sample index');ylabel('ADC raw - 2048 (code)');
title('Current channels: first 400 samples');
nexttile;
plot(x(displayIdx,2)-mu(2),x(displayIdx,3)-mu(3),'.');grid on;
xlabel('ADC2 code minus its mean');ylabel('ADC4 code minus its mean');
title(sprintf('Current-channel correlation: %.4f',report.correlation(2,3)));
exportgraphics(f,fullfile(outDir,'current_channel_detail.png'),'Resolution',150);
close(f);
end
