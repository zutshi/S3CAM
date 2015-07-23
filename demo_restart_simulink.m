mdl = 'AbstractFuelControl_M1';
mdl_path = '/home/zutshi/work/RA/cpsVerification/HyCU/releases/S3CAM/benchmarks/AbstractFuelControl/AbstractFuelControl_M1.slx';
load_system(mdl_path)

% mdl = 'o_AbstractFuelControl_M1';

simTime  = 50;

M1 = '/Model 1';
AFC_FC = '/AF_Controller/fuel_controller';
IM = '/Intake Manifold';
CnE = '/Cylinder and Exhaust';

fcm10 = '/fuel_controller_mode_10ms';
fc10 = '/fuel_controller_10ms';

NUM_STATES = 11;

x_str = cell(NUM_STATES,1);

x_str{1} = [mdl M1 '/Throttle delay'];

x_str{2} = [mdl M1 CnE '/Integrator'];

x_str{3} = [mdl M1 IM '/p0 = 0.543 (bar)'];

x_str{4} = [mdl M1 AFC_FC fcm10 '/sensor_failure_detection/Unit Delay'];
x_str{5} = [mdl M1 AFC_FC fcm10 '/normal_mode_detection/Unit Delay2'];
x_str{6} = [mdl M1 AFC_FC fcm10 '/normal_mode_detection/Unit Delay1'];
x_str{7} = [mdl M1 AFC_FC fcm10 '/power_mode_detection/Unit Delay1'];
x_str{8} = [mdl M1 AFC_FC fc10 '/air_estimation/UnitDelay1'];
x_str{9} = [mdl M1 AFC_FC fc10 '/feedback_PI_controller/UnitDelay1'];

x_str{10} = [mdl M1 '/Wall wetting/Integrator'];
x_str{11} = [mdl '/V&V stub system/Calcuate Error/RMS error/Integrator'];

% DSMs
% dsm_str = find_system('AbstractFuelControl_M1/Model 1/AF_Controller/','BlockType', 'DataStoreMemory')
dsm_str = cell(7,1);

% dsm_wrkspace_str = mySimOut.find;
dsm_workspace_str = cell(7,1);

dsm_str{1} = 'AbstractFuelControl_M1/Model 1/AF_Controller/commanded_fuel';
dsm_workspace_str{1} = 'commanded_fuel';

dsm_str{2} = 'AbstractFuelControl_M1/Model 1/AF_Controller/mode_fb';
dsm_workspace_str{2} = 'mode_fb';

dsm_str{3} = 'AbstractFuelControl_M1/Model 1/AF_Controller/mode_fb1';
dsm_workspace_str{3} = 'mode_fb1';

dsm_str{4} = 'AbstractFuelControl_M1/Model 1/AF_Controller/fuel_controller/DataStoreMemory';
dsm_workspace_str{4} = 'DataStoreMemory';

dsm_str{5} = 'AbstractFuelControl_M1/Model 1/AF_Controller/fuel_controller/DataStoreMemory1';
dsm_workspace_str{5} = 'DataStoreMemory1';

dsm_str{6} = 'AbstractFuelControl_M1/Model 1/AF_Controller/fuel_controller/DataStoreMemory2';
dsm_workspace_str{6} = 'DataStoreMemory2';

dsm_str{7} = 'AbstractFuelControl_M1/Model 1/AF_Controller/fuel_controller/DataStoreMemory3';
dsm_workspace_str{7} = 'DataStoreMemory3';


StartTime = 0.0;
% BreakTime = 19.997; fixes the issue of state mismatch.
% Reason is not entirely clear
% % Possible culprits:
%   pulse generator block or
%   Step block in
%   AbstractFuelControl_M1/Model 1/AF_Controller

BreakTime = 20.0;
StopTime = 50.0;

%% Initialize
X0_val = [0.000000  0.000000  0.982000  0.000000  0.000000  0.000000  0.000000  0.982000  0.000000  0.011200  0.000000];
set_param(x_str{1}, 'X0', num2str(X0_val(1)));
for i = 2:NUM_STATES
    set_param(x_str{i}, 'InitialCondition', num2str(X0_val(i)));
end
dsm_init_val = [0.1726 1 14.7 0.0 0.0 0.0 0.0];
for i = 1:7
    set_param(dsm_str{i}, 'InitialValue', num2str(dsm_init_val(i)));
end

%% RUN 1 - phase 1
% simulate for 20 s
mySimOut = sim(mdl, 'StartTime', num2str(StartTime),'StopTime', num2str(BreakTime), 'SaveFinalState', 'on', 'FinalStateName',[mdl 'SimState'], 'SaveCompleteFinalSimState', 'on', 'SaveFormat','Structure');
xf = mySimOut.get([mdl 'SimState']);
X1_str = cell(NUM_STATES, 1);
X1 = zeros(NUM_STATES, 1);
% get states
for i = 1:NUM_STATES
    X1(i) = xf.loggedStates(i).values;
    X1_str{i} = xf.loggedStates(i).blockName;
end
% get data store mem states
dsm_val_1 = cell(7, 1);
for i = 1:7
    dsm_val_1{i} = num2str(mySimOut.get(dsm_workspace_str{i}).Data);
end

%% RUN 1 - phase 2

% load states

set_param(X1_str{1}, 'InitialCondition', num2str(X1(1)))
set_param(X1_str{2}, 'X0', num2str(X1(2)))
for i = 3:NUM_STATES
    set_param(X1_str{i}, 'InitialCondition', num2str(X1(i)))
end

% load data store mem states

for i = 1:7
    set_param(dsm_str{i}, 'InitialValue', dsm_val_1{i});
end

% simulate
mySimOut = sim(mdl, 'StartTime', num2str(BreakTime), 'StopTime', num2str(StopTime), 'SaveFinalState', 'on', 'FinalStateName',[mdl 'SimState'], 'SaveCompleteFinalSimState', 'on', 'SaveFormat','Structure');

xf = mySimOut.get([mdl 'SimState']);
X2_ = zeros(NUM_STATES, 1);
for i = 1:NUM_STATES
    X2_(i) = xf.loggedStates(i).values;
end


%% Initialize
set_param(x_str{1}, 'X0', num2str(X0_val(1)));
for i = 2:NUM_STATES
    set_param(x_str{i}, 'InitialCondition', num2str(X0_val(i)));
end
for i = 1:7
    set_param(dsm_str{i}, 'InitialValue', num2str(dsm_init_val(i)));
end

%% RUN 2

mySimOut = sim(mdl, 'StartTime', num2str(StartTime),'StopTime', num2str(StopTime), 'SaveFinalState', 'on', 'FinalStateName',[mdl 'SimState'], 'SaveCompleteFinalSimState', 'on', 'SaveFormat','Structure');
xf = mySimOut.get([mdl 'SimState']);
X2 = zeros(NUM_STATES, 1);
% save final states
for i = 1:NUM_STATES
    X2(i) = xf.loggedStates(i).values;
end


%% compare both states
X2_
X2
if all(X2_ == X2) ~= 1
    X2_ - X2
    error('states not exactly the same')
else
    display('state match perfectly')
end



% dsm_str_2_ = mySimOut.find;
% dsm_val_2_ = cell(7, 1);
% for i = 2:9
%     dsm_val_2_{i} = num2str(mySimOut.get(dsm_str_2_{i}).Data);
% end

%
% X = zeros(NUM_STATES,1);
% Y = zeros(NUM_STATES,1);
%
% for i = 2:NUM_STATES
% % display([num2str(i) ' = ' num2str(get_param(x{i}, 'InitialCondition'))])
% Y(i) = get_param(x_str{i}, 'InitialCondition');
% end
%
% for i = 2:NUM_STATES
% set_param(x_str{i}, 'InitialCondition', X(i));
% end
%
% set_param([model_name '/int1'],'InitialCondition')
%
% % Change the parameter values in the model
% set_param([model,'/Pedal Angle (deg)'],'Amplitude',num2str(X0(1)));
% set_param([model,'/Pedal Angle (deg)'],'Period',num2str(X0(2)));
%
% out = sim(model, 'StopTime', num2str(simTime), 'SaveFinalState', 'on', 'FinalStateName','xFinal', 'SaveCompleteFinalSimState', 'on', 'SaveFormat','Structure');



% % %% retrieve all the parameters of a block and their corresponding values
% % pss = get_param('AbstractFuelControl_M1/V&V stub system', 'ObjectParameters');
% % fn = fieldnames(pss);
% % for i = 1:length(fn)
% % ['=========================' fn{i} '=========================']
% % get_param('AbstractFuelControl_M1/V&V stub system', fn{i})
% % end

