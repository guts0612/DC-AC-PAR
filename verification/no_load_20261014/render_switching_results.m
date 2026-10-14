output_dir=fileparts(mfilename('fullpath'));cd(output_dir);
names={'priority_440uH_Rc0_12ohm','priority_440uH_Rc0_no_load'};
fig=figure('Visible','off','Color','w','Position',[100 100 1000 650]);
layout=tiledlayout(fig,2,1,'TileSpacing','compact');
colors=[0.05 0.35 0.70;0.85 0.25 0.10];
labels={'12 ohm load','No load (1 Gohm)'};
nexttile(layout);hold on;
for k=1:2
 d=load(fullfile(output_dir,[names{k} '.mat']));
 plot(d.vr.Time,double(d.vr.Data),'LineWidth',1.4,'Color',colors(k,:));
end
grid on;xlabel('Time (s)');ylabel('Model true RMS (V)');
xlim([0.02 0.3]);ylim([23.7 24.2]);legend(labels,'Location','best');
title('Original switching model: 440 uH, C ESR = 0, slave gates off');
nexttile(layout);hold on;
for k=1:2
 d=load(fullfile(output_dir,[names{k} '.mat']));t=d.v.Time;
 select=t>=0.26 & t<=0.30;
 plot(t(select),double(d.v.Data(select)),'LineWidth',0.8,'Color',colors(k,:));
end
grid on;xlabel('Time (s)');ylabel('Output voltage (V)');xlim([0.26 0.30]);
title('Last two cycles; continuous outer loop and no MCU/PWM delay retained');
exportgraphics(fig,fullfile(output_dir,'switching_loaded_vs_no_load.png'),'Resolution',150);
close(fig);
fprintf('FIGURE_SAVED switching_loaded_vs_no_load.png\n');
