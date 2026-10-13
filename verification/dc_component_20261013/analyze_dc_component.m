function analyze_dc_component
% Read-only diagnosis from decimated telemetry. No unmeasured output DC is invented.
outDir=fileparts(mfilename('fullpath'));
csvPath='D:/download/vofa/直流分量.csv';
a=readmatrix(csvPath,'NumHeaderLines',1);
assert(size(a,2)==7 && all(isfinite(a(:))));
n=size(a,1);ref=a(:,1);v=a(:,2);ir=a(:,3);i=a(:,4);bridge=a(:,5);bus=a(:,7);
rawError=ref-v;correction=bridge-ref;
idx=find(ref(1:end-1)<0 & ref(2:end)>=0);
crossing=idx-ref(idx)./(ref(idx+1)-ref(idx));
periods=diff(crossing);
assert(numel(periods)>20,'Need several complete reference cycles.');
% Each whole reference cycle supplies raw-sample means/RMS plus a phase-based LS fit.
nc=numel(periods);means=zeros(nc,7);rmsValues=zeros(nc,7);
fund=zeros(nc,7);phase=zeros(nc,7);fitDC=zeros(nc,7);errorMean=zeros(nc,1);
correctionMean=zeros(nc,1);satCorrection=zeros(nc,1);satModulation=zeros(nc,1);
cycleSampleCounts=zeros(nc,1);
cycleMinBus=zeros(nc,1);cycleMaxIref=zeros(nc,1);
for c=1:nc
    rows=ceil(crossing(c)):ceil(crossing(c+1))-1;
    y=a(rows,:);p=2*pi*(rows'-crossing(c))/periods(c);
    design=[ones(numel(rows),1),sin(p),cos(p),sin(2*p),cos(2*p),sin(3*p),cos(3*p)];
    coefficients=design\y;
    means(c,:)=mean(y,1);rmsValues(c,:)=sqrt(mean(y.^2,1));
    fitDC(c,:)=coefficients(1,:);
    fund(c,:)=hypot(coefficients(2,:),coefficients(3,:))/sqrt(2);
    phase(c,:)=atan2(coefficients(3,:),coefficients(2,:))*180/pi;
    errorMean(c)=mean(rawError(rows));
    correctionMean(c)=mean(correction(rows));
    satCorrection(c)=mean(abs(correction(rows))>=4.9999);
    satModulation(c)=mean(abs(bridge(rows)./bus(rows))>=0.95);
    cycleSampleCounts(c)=numel(rows);
    cycleMinBus(c)=min(bus(rows));cycleMaxIref(c)=max(abs(ir(rows)));
end
% Explicitly classify the final supply-collapse event; retain it in the saved data.
loss=find(bus<40,1,'first');
if isempty(loss),loss=n+1;end
steady=(crossing(2:end)<loss) & rmsValues(:,2)>20;
steadyCycleIdx=find(steady);
assert(numel(steadyCycleIdx)>100,'Not enough normal-supply cycles.');
steadyRows=ceil(crossing(steadyCycleIdx(1))):ceil(crossing(steadyCycleIdx(end)+1))-1;
fprintf('First bus <40V: sample %d (~%.6f nominal seconds); last 10 samples:\n',loss,(loss-1)/2000);
disp(a(max(1,n-9):n,[2 3 4 7]));
% 25 complete cycles per block (~0.5 s at 50 Hz), avoiding arbitrary phase windows.
blockLength=25;nb=floor(nc/blockLength);
blockDC=zeros(nb,7);blockRMS=zeros(nb,7);blockFund=zeros(nb,7);
blockError=zeros(nb,1);blockCorrection=zeros(nb,1);
for b=1:nb
    q=(b-1)*blockLength+1:b*blockLength;
    blockDC(b,:)=mean(fitDC(q,:),1);
    blockRMS(b,:)=sqrt(mean(rmsValues(q,:).^2,1));
    blockFund(b,:)=mean(fund(q,:),1);
    blockError(b)=mean(errorMean(q));
    blockCorrection(b)=mean(correctionMean(q));
end
report=struct('source',csvPath,'matlab_version',version,'samples',n,'complete_cycles',nc, ...
    'assumed_sample_rate_hz',2000,'mean_reference_period_samples',mean(periods), ...
    'min_reference_period_samples',min(periods),'max_reference_period_samples',max(periods), ...
    'nominal_duration_s',n/2000,'whole_record_mean',mean(a), ...
    'whole_record_rms',sqrt(mean(a.^2)), ...
    'cycle_mean_channel_dc',mean(fitDC), ...
    'cycle_mean_fundamental_rms',mean(fund),'mean_v_phase_relative_ref_deg',mean(phase(:,2)-phase(:,1)), ...
    'correction_saturation_fraction',mean(abs(correction)>=4.9999), ...
    'modulation_saturation_fraction',mean(abs(bridge./bus)>=0.95), ...
    'correction_range_v',[min(correction),max(correction)], ...
    'max_abs_command_modulation',max(abs(bridge./bus)), ...
    'block_dc',blockDC,'block_rms',blockRMS,'block_fundamental_rms',blockFund, ...
    'block_raw_error_mean_v',blockError,'block_correction_mean_v',blockCorrection, ...
    'block_output_rms_range_v',[min(blockRMS(:,2)),max(blockRMS(:,2))], ...
    'block_current_reference_dc_range_a',[min(blockDC(:,3)),max(blockDC(:,3))], ...
    'cycle_current_reference_dc_range_a',[min(fitDC(:,3)),max(fitDC(:,3))], ...
    'cycle_output_rms_range_v',[min(rmsValues(:,2)),max(rmsValues(:,2))], ...
    'note','CSV omits DC estimator state and timestamps. Sensor-side voltage DC is not real output voltage DC; current-reference bias alone cannot prove insufficient compensation.');
% Virtual impedance DC equation provides a coarse, cycle-averaged inferred residual.
% Derivative includes dynamics; this is NOT a replay of the 20 kHz firmware filter.
dt=1/50;L=0.001;R=0.01;
irDC=fitDC(:,3);
estimatedResidual=L*gradient(irDC,dt)+R*irDC;
report.inferred_cycle_compensated_error_v=estimatedResidual;
report.inferred_cycle_estimator_v=errorMean-estimatedResidual;
report.mean_cycle_raw_error_v=mean(errorMean);
report.mean_inferred_compensated_error_v=mean(estimatedResidual);
report.mean_inferred_estimator_v=mean(errorMean-estimatedResidual);
report.output_rms_bus_correlation=corr(blockRMS(:,2),blockDC(:,7));
report.current_reference_dc_output_rms_correlation=corr(blockDC(:,3),blockRMS(:,2));
report.first_bus_below_40v_sample=loss;
report.steady_complete_cycles=numel(steadyCycleIdx);
report.steady_mean_voltage_sensor_dc_v=mean(fitDC(steady,2));
report.steady_mean_reference_dc_a=mean(fitDC(steady,3));
report.steady_mean_measured_current_dc_a=mean(fitDC(steady,4));
report.steady_reference_cycle_dc_range_a=[min(fitDC(steady,3)),max(fitDC(steady,3))];
report.steady_raw_voltage_error_mean_v=mean(errorMean(steady));
report.steady_current_error_dc_a=mean(fitDC(steady,3)-fitDC(steady,4));
report.steady_mean_correction_v=mean(correctionMean(steady));
report.steady_correction_saturation_fraction=mean(abs(correction(steadyRows))>=4.9999);
report.steady_modulation_saturation_fraction=mean(abs(bridge(steadyRows)./bus(steadyRows))>=0.95);
report.steady_max_command_modulation=max(abs(bridge(steadyRows)./bus(steadyRows)));
report.steady_cycle_rms_range_v=[min(rmsValues(steady,2)),max(rmsValues(steady,2))];
report.steady_mean_rms_v=sqrt(mean(rmsValues(steady,2).^2));
report.steady_mean_fundamental_rms_v=mean(fund(steady,2));
report.steady_mean_reference_fundamental_rms_v=mean(fund(steady,1));
report.steady_reference_rms_a=mean(fund(steady,3));
report.steady_mean_phase_deg=mean(phase(steady,2)-phase(steady,1));
report.steady_virtual_dc_error_estimate_v=0.01*mean(fitDC(steady,3));
report.steady_inferred_estimator_v=mean(errorMean(steady))-report.steady_virtual_dc_error_estimate_v;
report.normal_condition_note='Normal-supply statistics use complete cycles before the first bus<40V sample and voltage RMS>20V. Final supply collapse is reported separately, not deleted.';
fprintf('NORMAL-SUPPLY ONLY: %d cycles\n',numel(steadyCycleIdx));
fprintf('RMS %.9f V; cycle range %.9f..%.9f V\n',report.steady_mean_rms_v,report.steady_cycle_rms_range_v);
fprintf('Voltage sensor DC %.9f V; current ref DC %.9f A; measured current DC %.9f A\n', ...
    report.steady_mean_voltage_sensor_dc_v,report.steady_mean_reference_dc_a,report.steady_mean_measured_current_dc_a);
fprintf('Current ref cycle DC range %.9f..%.9f A\n',report.steady_reference_cycle_dc_range_a);
fprintf('Steady correction saturation %.9f%%, max modulation %.9f\n', ...
    100*report.steady_correction_saturation_fraction,report.steady_max_command_modulation);
fprintf('Steady inferred mean DC estimator %.9f V; compensated DC equivalent %.9f V\n', ...
    report.steady_inferred_estimator_v,report.steady_virtual_dc_error_estimate_v);
save(fullfile(outDir,'dc_component_analysis.mat'),'a','report','means','rmsValues','fund','phase','fitDC', ...
    'periods','cycleSampleCounts','errorMean','correctionMean','satCorrection','satModulation');
fid=fopen(fullfile(outDir,'dc_component_results.json'),'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Samples=%d, complete cycles=%d, mean period=%.6f samples\n',n,nc,mean(periods));
fprintf('Whole-record means [ref,v,iref,i,bridge,iaux,bus]:\n');disp(mean(a));
fprintf('Mean cycle fitted DC:\n');disp(mean(fitDC));
fprintf('Mean fundamental RMS:\n');disp(mean(fund));
fprintf('Output %.6f..%.6f Vrms (25-cycle blocks)\n',report.block_output_rms_range_v);
fprintf('Current ref DC %.6f..%.6f A (25-cycle blocks)\n',report.block_current_reference_dc_range_a);
fprintf('Measured current DC %.6f..%.6f A (25-cycle blocks)\n',min(blockDC(:,4)),max(blockDC(:,4)));
fprintf('Raw error mean %.9f V; inferred compensated residual %.9f V\n',mean(errorMean),mean(estimatedResidual));
fprintf('Correction range %.6f..%.6f V, saturation %.6f%%, modulation max %.6f\n', ...
    min(correction),max(correction),100*report.correction_saturation_fraction,report.max_abs_command_modulation);
fprintf('Voltage phase relative reference %.6f deg\n',report.mean_v_phase_relative_ref_deg);
fprintf('RMS/bus correlation %.6f; RMS/irefDC correlation %.6f\n',report.output_rms_bus_correlation,report.current_reference_dc_output_rms_correlation);
f=figure('Visible','off','Color','w','Position',[100 100 1350 1050]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
time=(1:nc)'/50;
nexttile;plot(time,rmsValues(:,2),'Color',[.75 .75 .75]);hold on;
plot((1:nb)'*blockLength/50,blockRMS(:,2),'LineWidth',1.6);yline(24,'k--');grid on;
ylim([23.8 24.3]);xlim([0 max(time)]);
ylabel('Output RMS (V)');title('Sensor-side voltage: cycle RMS and 25-cycle block RMS');
nexttile;plot(time,fitDC(:,3));hold on;plot(time,fitDC(:,4));grid on;
ylim([-.12 .12]);xline((loss-1)/2000,'k--','Supply collapse');
legend('Current reference DC','Measured current DC','Location','best');ylabel('Cycle fitted DC (A)');
nexttile;plot(time,errorMean);hold on;plot(time,estimatedResidual);grid on;
ylim([-.5 .15]);
legend('Raw voltage-error cycle mean','Inferred compensated residual','Location','best');ylabel('Voltage error (V)');
nexttile;plot((1:nb)'*blockLength/50,blockDC(:,7));grid on;
ylabel('Mean DC bus (V)');xlabel('Nominal time (s; 50 Hz reference assumed)');
exportgraphics(f,fullfile(outDir,'dc_component_trends.png'),'Resolution',150);close(f);
end
