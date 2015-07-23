function [sys_def,sys_prop,sys_abs,sys_opt] = bball()

sys_def.str= 'bball';
sys_def.hybrid = 1;

% # of state vars to the system
sys_def.NUM_STATE_VARS = 4;
% # of exogenous inputs to the system
sys_def.NUM_CINPUT_VARS = 0;
% # of parameters to the system
sys_def.NUM_PARAMS = 0;
% list of modes
sys_def.MODE_LIST = [1];

%% System description
% sys_def.SIM = @SIM;
sys_def.modeInvMap = getModeInvMap();
sys_prop.initConstraints = initConstraints();
sys_prop.finalConstraints = finalConstraints();

%% Aux functions
sys_opt.plotFun = @plotFun;
sys_opt.initFun = @initFun;
sys_opt.drawProp = @drawProp;

%% simulation parameters
% Time horizon (T)
sys_prop.TH = 40;

%% select initial abstraction parameters
sys_abs.grid_eps = [1 1 1 1];
sys_abs.numSamples = 2;
sys_abs.Delta = 10;

sys_opt.runType = 'analyzeInit';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


sys_def.transitionMap = getTransitionMap(sys_def);
sys_def.modeDynMap = getModeDynMap();
sys_def.getModeTransitions = getModeTransitionMap();
% sys_maps.minMaxDwellTimesMap = getMinMaxDwellTimesMap();
% sys_maps.minMaxStateVarsMap = getMinMaxStateVarsMap();
% sys_def.maxDwellTimesMap = getMaxDwellTimesMap();
% sys_def.minDwellTimesMap = getMinDwellTimesMap();

% sys_def.maxStateVarsMap = getMaxStateVarsMap();
% sys_def.minStateVarsMap = getMinStateVarsMap();


sys_opt.samplingType = 'UniformRand';
%sys_opt.samplingType = 'UniformAll';
%sys_opt.samplingType = 'Halton';

% sys_opt.ScatterSim.MAX_SAMPLE_AGE = 40;
% sys_opt.ScatterSim.MAX_COSINESS = 0.4;
% sys_opt.ScatterSim.TOO_CLOSE_FOR_COMFORT = 0.3;
% 
% % used for result compilation
% sys_opt.ScatterSim.NUM_CHILDREN = 1;
% 
% % should be tried
% % sys_opt.ScatterSim.NUM_CHILDREN = 2;
% 
sys_opt.ScatterSim.MAX_SWITCHINGS = 7;
% sys_opt.MODE_TIME_HORIZON = 12;
% 
% sys_opt.ScatterSim.PERIODIC = 1;
% sys_opt.ScatterSim.GUARDS = 0;

% should be tried
% sys_opt.ScatterSim.GAP_ARR = 1*[1 1 1 1];

% % used for result compilation
% sys_opt.ScatterSim.GAP_ARR = 0.4*[1 1 1 1];
% sys_opt.ScatterSim.SCALE = [1 1 1 1];

% sys_opt.minLengthTrajThresh = 0.5;
% sys_opt.useGrad = 1;
sys_opt.figID = 2;

sys_opt.MAX_ITER = 10;

sys_opt.GuardDetectRoundDigits = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if nargin == 1
%     if get_structs == 1
%         sys_opt.Abstraction.Delta = 10;
%         return;
%     else
%         error('bball_hycu: unknown value')
%     end
% end



% disp('****************************')
% fprintf('sys_opt.Abstraction.grid_eps ')
% fprintf('%f ',sys_abs.grid_eps);
% fprintf('\n');
% fprintf('sys_opt.Abstraction.refinementFactor %f\n',sys_opt.Abstraction.refinementFactor);
% fprintf('sys_opt.Abstraction.numSamples %f\n',sys_opt.Abstraction.numSamples);
% fprintf('sys_opt.Abstraction.Delta %f\n',sys_opt.Abstraction.Delta);
% disp('****************************')


% sys_opt.phase = 24;

sim_fn = hybrid_system_simulator(sys_def,sys_prop,sys_opt);
sys_def.SIM = sim_fn;

%% test simulator
test_sim = 0;
if test_sim == 1
    figure(1)
    hold on
    t = [0];
    XArr = [0 0 5 10];
    XCumuArr = XArr;
    delta = 1;
    for i = 0:delta:30
        [simtimeArr, XArr, MArr] = sim_fn(i, i+delta, XArr, [1], [0]);
        t = [t simtimeArr];
        XCumuArr = [XCumuArr; XArr];
    end
    plot(t, XCumuArr(:,2))
end
end

function plotFun(~,X,~,~)
hold on
plot(X(:,1),X(:,2),'r-');
%hold off
%drawnow;
end

function initFun()
end

function drawProp()
cons_f = finalConstraints();
cons_f = cons_f.cube;
line([cons_f(1,1) cons_f(1,2) cons_f(1,2) cons_f(1,1) cons_f(1,1)],[cons_f(2,1) cons_f(2,1) cons_f(2,2) cons_f(2,2) cons_f(2,1)],'color','b');
end

function cons = initConstraints()
initial = [-0.1 0.1
    3 4
    5 10
    -5 5];
%initial = [-4 4; -inf inf; -inf inf;-inf inf];
% Al = -eye(sysInfo.NUM_STATE_VARS);
% bl = -initial(:,1);
% Ah = eye(sysInfo.NUM_STATE_VARS);
% bh = initial(:,2);
% cons.A = [Al;Ah];
% cons.b = [bl;bh];
cons.cube = initial;
cons.isCube = 1;
cons.mode = 1;
cons.isNonLinear = 0;
end

function cons = finalConstraints()
% property for linear bball
final = [330    330.1
    1      1.1
    -Inf    Inf
    -Inf    Inf];


% property with air resistance
finalAR1 = [334.5    334.6
    0.5      0.52
    -Inf    Inf
    -Inf    Inf];

% property with air resistance
finalAR2 = [339    339.1
    0.55      0.6
    -Inf    Inf
    -Inf    Inf];

% property with air resistance
finalAR3 = [65    66
    9.1      8.3
    -Inf    Inf
    -Inf    Inf];

% Al = -eye(sysInfo.NUM_STATE_VARS);
% bl = -final(:,1);
% Ah = eye(sysInfo.NUM_STATE_VARS);
% bh = final(:,2);
% cons.A = [Al;Ah];
% cons.b = [bl;bh];
cons.cube = final;
cons.isCube = 1;
cons.mode = 1;
cons.isNonLinear = 0;
end


function cons = finalConstraints_BIG()
p = 0.5/100;
% property for linear bball
final = [330-330*p    330.1+330.1*p
    1-1*p      1.1+1.1*p
    -Inf    Inf
    -Inf    Inf];


cons.cube = final;
cons.isCube = 1;
cons.mode = 1;
cons.isNonLinear = 0;
end

function transitionMap = getTransitionMap(sys_def)
% guard
guard.isNonLinear = 0;
guard.A = [0 1 0 0
    0 -1 0 0];
guard.b = [0 0]';
% reset
reset.isNonLinear = 0;
reset.A = eye(sys_def.NUM_STATE_VARS);
reset.A(2,2) = 0;
reset.A(4,4) = -0.75;
reset.b = zeros(sys_def.NUM_STATE_VARS,1);
reset.b(2) = 1e-7;

% transition from 1 -> 1
fromModeID = 1;toModeID = 1;
transitionMap{fromModeID,toModeID}{1}.guard = guard;
transitionMap{fromModeID,toModeID}{1}.reset = reset;
end

function modeInvMap = getModeInvMap()
modeID = 1;
modeInvMap(modeID).A = [-1 0 0 0
    1 0 0 0
    0 -1 0 0
    0 1 0 0
    0 0 -1 0
    0 0 1 0
    0 0 0 -1
    0 0 0 1];

modeInvMap(modeID).b = [1 400 0 20 -5 10 10 10]';
modeInvMap(modeID).isNonLinear = 0;
end

% easier than checking for every possible transition!
function modeTransitionMap = getModeTransitionMap()
% empty
modeTransitionMap = [];
end

function Y = airResDyn(~,X,~,~)
g = -1;
vx = X(3);
vy = X(4);

A = 0.5;
Cd = 0.7;
r = 1.22;

dx = vx;
dy = vy;
dvx = 0;
airRes = (1/50)*Cd*r*A*(vy^2)/2;
dvy = g - sign(vy)*airRes;
Y = [dx;dy;dvx;dvy];
end

function modeDynMap = getModeDynMap()
g = -1;
modeID = 1;

modeDynMap(modeID).isNonLinear = 0;
modeDynMap(modeID).A = [0 0 1 0
    0 0 0 1
    0 0 0 0
    0 0 0 0];
modeDynMap(modeID).b = [0 0 0 g]';

% modeDynMap(modeID).isNonLinear = 1;
% modeDynMap(modeID).F = @airResDyn;

end

% % approximate bounds on the state variables in every mode
% function minMaxStateVarsMap = getMinMaxStateVarsMap()
% modeID = 1;
% minMaxStateVarsMap(modeID).XMin = [-1 -0.01 -10 -10]';
% minMaxStateVarsMap(modeID).XMax = [400 20 10 10]';
% end
% 
% 
% % approximate bounds on the dwell time in every mode
% function  minMaxDwellTimesMap = getMinMaxDwellTimesMap()
% modeID = 1;
% minMaxDwellTimesMap(modeID).TMin = 0.01;
% minMaxDwellTimesMap(modeID).TMax = 10;
% end
% 

% approximate bounds on the state variables in every mode
function minStateVarsMap = getMinStateVarsMap()
for currMode = 1:NUM_MODES
    minStateVarsMap(currMode,:) = [-1 -0.01 -10 -10];
%     minStateVarsMap(currMode,:) = [-inf -inf -inf -inf];
end
end

% approximate bounds on the state variables in every mode
function maxStateVarsMap = getMaxStateVarsMap()
for currMode = 1:NUM_MODES
    maxStateVarsMap(currMode,:) = [400 20 10 10];
%     maxStateVarsMap(currMode,:) = [inf inf inf inf];
end
end


% approximate bounds on the dwell time in every mode
function  ret = getMaxDwellTimesMap()
% ignore the currmode arg
    function maxDwellTimesMap = retTimeBounds(~)
        maxDwellTimesMap = 30;
    end
ret = @retTimeBounds;
end

% approximate bounds on the dwell time in every mode
function  ret = getMinDwellTimesMap()
% ignore the currmode arg
    function minDwellTimesMap = retTimeBounds(~)
        minDwellTimesMap = 0.1;
    end
ret = @retTimeBounds;
end

function y = NUM_MODES
y = 1;
end
