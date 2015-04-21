%% Start logging

%% get system structures
% [sys_def,sys_prop,sys_abs,sys_opt] = AbstractFuelControl;
[sys_def,sys_prop,sys_abs,sys_opt] = vanderpol1();
% [sys_def,sys_prop,sys_abs,sys_opt] = vanderpol1_sml();

%% start logging
clock_str = num2str(clock(), '%02.0f');
unique_str = ['_' regexprep(clock_str,' *','_') '_'];

diary([sys_def.str unique_str '.diary']);
diary on;

%% record it
disp('****************************')
fprintf('sys_abs.grid_eps ')
fprintf('%f ',sys_abs.grid_eps);
fprintf('\n');
fprintf('sys_abs.numSamples %f\n',sys_abs.numSamples);
fprintf('sys_abs.Delta %f\n',sys_abs.Delta);
disp('****************************')


%% run in a loop N times
N = 1;

for i = 1:N
    close all
    disp('BEGINS')
    time_total = tic;
    ret_val = S3CAM(sys_def,sys_prop,sys_abs,sys_opt);
    
    if ret_val == 0
        fprintf('Total Running Time');toc(time_total);
    else
        fprintf('DNF Time');toc(time_total);
        
    end
    disp('ENDS')
end



diary off;
