% Prioritized realistic wiring experiments; original model remains unchanged.
output_dir=fileparts(mfilename('fullpath'));
cd(output_dir);
mdl='BINLIANSinglePhase';
cases=struct('name',{'priority_440uH_Rc0_12ohm','priority_440uH_Rc0_no_load','priority_440uH_Rc1_no_load'}, ...
 'Rload',{12,1e9,1e9},'Rc',{0,0,1});
results=struct([]);
for idx=1:numel(cases)
 c=cases(idx);
 fprintf('\nPRIORITY_CASE_START %s %s\n',c.name,datestr(now,31));
 load_system('D:/study/MATLAB-study/BINLIANSinglePhase.slx');
 set_param(mdl,'StopTime','0.3','SaveTime','off','SaveOutput','off','SignalLogging','off','SaveState','off');
 set_param(sidpath(mdl,'195'),'Resistance',num2str(c.Rload,17));
 for sid={'29','30','156','157'}
  set_param(sidpath(mdl,sid{1}),'Inductance','220e-6');
 end
 for sid={'36','160'}
  set_param(sidpath(mdl,sid{1}),'Resistance',num2str(c.Rc,17));
 end
 add_block('simulink/Sources/Constant',[mdl '/AuditSlaveGateZero'],'Value','0');
 add_block('simulink/Signal Routing/Goto',[mdl '/AuditSlaveGateZeroTag'],'GotoTag','AuditSlaveGateZero','TagVisibility','global');
 add_line(mdl,'AuditSlaveGateZero/1','AuditSlaveGateZeroTag/1');
 for sid={'137','138','139','140'}
  set_param(sidpath(mdl,sid{1}),'GotoTag','AuditSlaveGateZero');
 end
 taps={'198','236','234','235'};vars={'v_trace','v_rms','master_rms','slave_rms'};
 samples={'2e-6','5e-5','5e-5','5e-5'};
 for k=1:numel(taps)
  src=sidpath(mdl,taps{k});sink=[mdl '/AuditLog_' vars{k}];
  add_block('simulink/Sinks/To Workspace',sink,'VariableName',vars{k},'SaveFormat','Timeseries','SampleTime',samples{k},'MaxDataPoints','1000000');
  ph=get_param(src,'PortHandles');sh=get_param(sink,'PortHandles');
  add_line(mdl,ph.Outport(1),sh.Inport(1),'autorouting','on');
 end
 started=tic;
 out=sim(mdl,'ReturnWorkspaceOutputs','on');
 vr=out.get('v_rms');v=out.get('v_trace');mr=out.get('master_rms');sr=out.get('slave_rms');
 tail=vr.Time>=0.2;
 r=c;r.elapsed_s=toc(started);r.v_rms_mean=mean(vr.Data(tail));r.v_rms_std=std(vr.Data(tail));
 r.v_rms_min=min(vr.Data(tail));r.v_rms_max=max(vr.Data(tail));
 vt=v.Time>=0.2;r.v_mean=mean(v.Data(vt));r.v_peak=max(abs(v.Data(vt)));
 r.master_current_rms=mean(mr.Data(mr.Time>=0.2));r.slave_current_rms=mean(sr.Data(sr.Time>=0.2));
 save(fullfile(output_dir,[c.name '.mat']),'v','vr','mr','sr','r','-v7');
 fprintf('PRIORITY_CASE_RESULT %s Vrms=%.9g std=%.9g min=%.9g max=%.9g Vmean=%.9g I1=%.9g I2=%.9g runtime=%.3fs\n', ...
 c.name,r.v_rms_mean,r.v_rms_std,r.v_rms_min,r.v_rms_max,r.v_mean,r.master_current_rms,r.slave_current_rms,r.elapsed_s);
 results=[results;r]; %#ok<AGROW>
 fid=fopen(fullfile(output_dir,'priority_results.json'),'w');fwrite(fid,jsonencode(results,'PrettyPrint',true));fclose(fid);
 close_system(mdl,0);
end
function p=sidpath(mdl,sid)
 p=getfullname(Simulink.ID.getHandle([mdl ':' sid]));
end
