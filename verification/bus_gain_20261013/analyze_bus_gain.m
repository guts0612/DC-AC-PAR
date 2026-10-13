function analyze_bus_gain
% Calibrate only bus scale, keeping the current raw-code zero offset unchanged.
csvPath = 'D:/download/vofa/母线电压.csv';
outDir = fileparts(mfilename('fullpath'));
data = readmatrix(csvPath, 'NumHeaderLines', 1);
assert(size(data,2)==7 && all(isfinite(data(:))), 'Invalid seven-channel CSV.');
voltage = data(:,7); % Current firmware: CH7 / I6 = measured DC bus, in V.
referenceVoltage = 47.849;
oldGain = 24.0;
meanVoltage = mean(voltage);
assert(meanVoltage>0, 'Cannot calibrate gain against a zero/negative mean.');
newGain = oldGain * referenceVoltage / meanVoltage;
result = struct('source',csvPath,'matlab_version',version,'channel',7, ...
    'samples',numel(voltage),'mean_bus_v',meanVoltage,'median_bus_v',median(voltage), ...
    'std_bus_v',std(voltage),'min_bus_v',min(voltage),'max_bus_v',max(voltage), ...
    'reference_bus_v',referenceVoltage,'old_gain',oldGain,'new_gain',newGain, ...
    'gain_delta',newGain-oldGain,'gain_change_percent',(newGain/oldGain-1)*100, ...
    'raw_zero_code_preserved',16,'vref_v',3.0, ...
    'corrected_mean_bus_v',mean(voltage*(newGain/oldGain)));
edges = round(linspace(0,numel(voltage),21));
result.block_means_v = zeros(1,20);
for k=1:20
    result.block_means_v(k) = mean(voltage(edges(k)+1:edges(k+1)));
end
fid = fopen(fullfile(outDir,'bus_gain_results.json'),'w','n','UTF-8');
assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);
fid = fopen(fullfile(outDir,'bus_gain_results.txt'),'w','n','UTF-8');
assert(fid>=0);
fprintf(fid,'Samples: %d\nCH7 mean: %.9f V\nCH7 standard deviation: %.9f V\n', ...
    result.samples,meanVoltage,result.std_bus_v);
fprintf(fid,'Reference: %.6f V\nOld gain: %.9f\nNew gain: %.9f\nDelta: %+.9f\nChange: %+.6f %%\n', ...
    referenceVoltage,oldGain,newGain,newGain-oldGain,result.gain_change_percent);
fprintf(fid,'Keep DC_BUS_ZERO_CODE=16 and vref_voltage=3.0 V unchanged.\n');
fclose(fid);
disp(result);
end
