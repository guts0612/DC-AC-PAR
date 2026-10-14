output_dir = fileparts(mfilename('fullpath'));
cd(output_dir);
mdl = 'BINLIANSinglePhase';
load_system('D:/study/MATLAB-study/BINLIANSinglePhase.slx');
fprintf('MATLAB_VERSION=%s\n', version);
fprintf('SOLVER=%s, SOLVER_TYPE=%s\n',get_param(mdl,'Solver'),get_param(mdl,'SolverType'));
rt=sfroot;
ch=rt.find('-isa','Stateflow.EMChart');
for k=1:numel(ch)
 fprintf('CHART=%s SAMPLE_TIME=%s UPDATE_METHOD=%s\n',ch(k).Path,ch(k).SampleTime,ch(k).ChartUpdate);
end
feval(mdl,[],[],[],'compile');
for k=1:numel(ch)
 fprintf('COMPILED=%s ',ch(k).Path);
 disp(get_param(ch(k).Path,'CompiledSampleTime'));
end
feval(mdl,[],[],[],'term');
sid_list={'4','127','88','92','124','29','30','36','160','240'};
for k=1:numel(sid_list)
 b=Simulink.ID.getHandle([mdl ':' sid_list{k}]);
 fprintf('SID=%s PATH=%s\n',sid_list{k},getfullname(b));
end
close_system(mdl,0);
