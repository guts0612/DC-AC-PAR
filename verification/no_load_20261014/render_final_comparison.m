output_dir=fileparts(mfilename('fullpath'));cd(output_dir);
names={'extended_440uH_Rc0_no_load','delayed_440uH_Rc0_no_load'};
labels={'Original model timing','Added 50 us modulation delay'};
colors=[.05 .35 .70;.85 .25 .10];
fig=figure('Visible','off','Color','w','Position',[100 100 1100 720]);
tl=tiledlayout(fig,2,1,'TileSpacing','compact');
nexttile(tl);hold on;
for k=1:numel(names)
 d=load([names{k} '.mat']);
 plot(d.vr.Time,double(d.vr.Data),'Color',colors(k,:),'LineWidth',1.1);
end
yline(24,'--','24 V reference');grid on;
xlabel('Time (s)');ylabel('True RMS (V)');xlim([0 1]);legend(labels,'Location','best');
title('Master only: 440 uH, no explicit C series resistor, slave gates off; both capacitors retained');
nexttile(tl);hold on;
for k=1:numel(names)
 d=load([names{k} '.mat']);t=d.v.Time;mask=t>=.96;
 plot(t(mask),double(d.v.Data(mask)),'Color',colors(k,:),'LineWidth',.7);
end
grid on;xlabel('Time (s)');ylabel('Output voltage (V)');xlim([.96 1]);
title('Continuous outer loop retained; delayed case also samples full modulation at 20 kHz');
exportgraphics(fig,'master_delay_comparison.png','Resolution',150);close(fig);
names={'original_parallel_12ohm','original_parallel_no_load','extended_440uH_Rc0_no_load','delayed_440uH_Rc0_no_load'};
analyses=struct([]);
for k=1:numel(names)
 d=load([names{k} '.mat']);t=d.v.Time;mask=t>=.8&t<1;t=double(t(mask));v=double(d.v.Data(mask));
 v=v(:);t=t(:);n=numel(v);
 a=2*sum(v.*sin(2*pi*50*t))/n;b=2*sum(v.*cos(2*pi*50*t))/n;
 dc=mean(v);residual=v-a*sin(2*pi*50*t)-b*cos(2*pi*50*t)-dc;
 spec=abs(fft(residual))/n;fs=1/median(diff(t));f=(0:n-1)'*fs/n;
 band=f>=500&f<=20000;indices=find(band);[peak,j]=max(spec(band));
 r=struct('case_name',names{k},'window_s',[.8,1], ...
    'raw_trace_rms',sqrt(mean(v.^2)),'fundamental_50Hz_rms',hypot(a,b)/sqrt(2), ...
    'residual_rms',sqrt(mean(residual.^2)),'dominant_500Hz_to_20kHz',f(indices(j)), ...
    'dominant_sinusoidal_peak_V',2*peak,'trace_sample_period_s',1/fs);
 fprintf('TRACE_ANALYSIS %s rawRMS=%.9g fundamental=%.9g residual=%.9g dominantHz=%.9g\n', ...
    r.case_name,r.raw_trace_rms,r.fundamental_50Hz_rms,r.residual_rms,r.dominant_500Hz_to_20kHz);
 analyses=[analyses;r]; %#ok<AGROW>
end
fid=fopen('trace_spectral_analysis.json','w');fwrite(fid,jsonencode(analyses,'PrettyPrint',true));fclose(fid);
