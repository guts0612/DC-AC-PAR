output_dir=fileparts(mfilename('fullpath'));cd(output_dir);
d=load('original_parallel_no_load.mat');
windows=[0.08 0.10;0.18 0.20;0.38 0.40;0.58 0.60;0.78 0.80;0.98 1.00];
rows=[];
for k=1:size(windows,1)
 t=d.vr.Time;mask=t>=windows(k,1)&t<=windows(k,2);
 row=[windows(k,:) mean(d.vr.Data(mask)) min(d.vr.Data(mask)) max(d.vr.Data(mask))];
 rows=[rows;row]; %#ok<AGROW>
 fprintf('ORIGINAL_NO_LOAD_WINDOW %.2f-%.2fs RMS_mean=%.9g RMS_min=%.9g RMS_max=%.9g\n',row);
end
writematrix(rows,'original_no_load_windows.csv');
fig=figure('Visible','off','Color','w','Position',[100 100 1000 650]);
tl=tiledlayout(fig,2,1,'TileSpacing','compact');
nexttile(tl);plot(d.vr.Time,double(d.vr.Data),'Color',[.85 .25 .10],'LineWidth',1.2);
yline(24,'--','24 V reference');xline(.1,':','Saved model stop time');
grid on;xlabel('Time (s)');ylabel('True RMS (V)');xlim([0 1]);
title('Saved model: both inverters active, no load, 600 uH per inverter, capacitor series R = 1 ohm');
nexttile(tl);t=d.v.Time;mask=t>=.96;
plot(t(mask),double(d.v.Data(mask)),'Color',[.85 .25 .10],'LineWidth',.8);
grid on;xlabel('Time (s)');ylabel('Output voltage (V)');xlim([.96 1]);
title('Last two cycles show large high-frequency content');
exportgraphics(fig,'original_parallel_no_load.png','Resolution',150);close(fig);
