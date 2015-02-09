function [sys_def,sys_prop,sys_abs,sys_opt] = vanderpol1()

% System description string
sys_def.str = 'vanderpol1';

%% System description
sys_def.SIM = @SIM;
sys_def.modeInvMap = getModeInvMap();
sys_prop.initConstraints = initConstraints();
sys_prop.finalConstraints = finalConstraints();

% # of state vars to the system
sys_def.NUM_STATE_VARS = NUM_STATE_VARS;
% # of exogenous inputs to the system
sys_def.NUM_CINPUT_VARS = 0;
% # of parameters to the system
sys_def.NUM_PARAMS = 0;


%% Aux functions
sys_opt.plotFun = @plotFun;
sys_opt.initFun = @initFun;
sys_opt.drawProp = @drawProp;

%% simulation parameters
% Time horizon (T)
sys_prop.TH = 1;

%% select initial abstraction parameters
sys_abs.grid_eps = [.1 .1];
sys_abs.numSamples = 3;
sys_abs.Delta = .4;

sys_opt.runType = 'analyzeInit';

return
end

%% Sim Function
function [ret_t,ret_X,currMode] = SIM(Tspan,XX,~,currMode)

optionsODE = odeset('Refine',4,'RelTol',1e-6);
    function Y = dyn(~,X,~,~)
        Y(1) = X(2);
        Y(2) = 5 * (1 - X(1)^2) * X(2) - X(1);
        Y = Y';
    end

simTime = 0;
ret_t = [];
ret_X = [];
T = Tspan(1,2);

[tt,YY] = ode45(@dyn,[0 T - simTime],XX,optionsODE);
ret_t = [ret_t; tt+simTime];
ret_X = [ret_X; YY];
end

%% use it to plot the desired properties
function plotFun(~,X,currMode,plotOpts)
if nargin == 4
    plot(X(:,1),X(:,2),plotOpts);
else
    plot(X(:,1),X(:,2),'-r');
end

end

%% called everytime a figure is created. Can use to draw properties
function drawProp()
cons_i = initConstraints();
cons_i_cube = cons_i.cube;
line([cons_i_cube(1,1) cons_i_cube(1,2) cons_i_cube(1,2) cons_i_cube(1,1) cons_i_cube(1,1)],[cons_i_cube(2,1) cons_i_cube(2,1) cons_i_cube(2,2) cons_i_cube(2,2) cons_i_cube(2,1)],'color','g');

cons_f = finalConstraints();
cons_f_cube = cons_f.cube;
line([cons_f_cube(1,1) cons_f_cube(1,2) cons_f_cube(1,2) cons_f_cube(1,1) cons_f_cube(1,1)],[cons_f_cube(2,1) cons_f_cube(2,1) cons_f_cube(2,2) cons_f_cube(2,2) cons_f_cube(2,1)],'color','g');
end

%% called once in the begining
function initFun()
%figure(1)
%hold on
% illustrate porp 1(easy)
%line([-1 -0.7 -0.7 -1 -1],[-6.5 -6.5 -5.6 -5.6 -6.5])
end

function cons = initConstraints()
initial = [-0.4 0.4 %x
    -0.4 0.4 %y
    ];
cons.cube = initial;
cons.isCube = 1;
cons.mode = 1;
cons.isNonLinear = 0;
end

function cons = finalConstraints()

final = [-1 -0.7
    -6.5 -5.6];

cons.cube = final;
cons.isCube = 1;
cons.mode = -1;
cons.isNonLinear = 0;
end

function modeInvMap = getModeInvMap()
H = helper(NUM_STATE_VARS);
cons1 = H.getConstraint([X1 -1],10);
cons2 = H.getConstraint([X1 1],10);
cons3 = H.getConstraint([X2 -1],10);
cons4 = H.getConstraint([X2 1],10);

modeInvMap(1) = H.getConsMat([cons1 cons2 cons3 cons4]);

end

%% global vars

function y = NUM_STATE_VARS()
y = 2;
end
function y = X1()
y = 1;
end
function y = X2()
y = 2;
end
