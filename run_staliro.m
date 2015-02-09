function run_hycu_files_using_staliro()

clear;



% addpath('../nav_sim/')
% plotmap(1,'../nav_sim/nav30.mat')
% hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%  Call the benchmark  %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[sys_maps,sys_info,sys_opt] = constrPend(1);
% [sys_maps,sys_info,sys_opt] = brusselator(1);
% [sys_maps,sys_info,sys_opt] = idle_speed_engine_control_v2_hycu(1);
% [sys_maps,sys_info,sys_opt] = bball_hycu(1);
% [sys_maps,sys_info,sys_opt] = diabeticFisher(1);
% [sys_maps,sys_info,sys_opt] = navNew(1);
% [sys_maps,sys_info,sys_opt] = navNewQ(1);
% [sys_maps,sys_info,sys_opt] = navNewR(1);
% [sys_maps,sys_info,sys_opt] = navNewS(1);
% [sys_maps,sys_info,sys_opt] = bball_sinusiod(1);
% [sys_maps,sys_info,sys_opt] = vpt1(1);
% [sys_maps,sys_info,sys_opt] = vpt2(1);
% [sys_maps,sys_info,sys_opt] = vpt3(1);
% [sys_maps,sys_info,sys_opt] = mpc_engine(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sys_opt.phase = 100;

if sys_info.NUM_PARAMS > 0
    error('params unhandled')
end

initMode = sys_maps.initConstraints.mode;
inputSignalStore = [];

% ignore simulation time
    function [T, XT, YT, LT, CLG, GRD] = hycu_wrapper(XPoint,~)
        [~,T, XT] = HyCu(sys_maps,sys_info,sys_opt,XPoint,initMode,[],eventToTransitionInvMap,eventSet,terminalSet,eventInfo);
        YT = [];
        LT = [];
        CLG = [];
        GRD = [];
    end

% ignore simulation time
    function [T, XT, YT, LT, CLG, GRD] = hycu_inputs_wrapper(XPoint,~, steptime,InpSignal)
        inputSignalStore = InpSignal;
        [~,T, XT] = HyCu(sys_maps,sys_info,sys_opt,XPoint,initMode,InpSignal');
        YT = [];
        LT = [];
        CLG = [];
        GRD = [];
    end

if sys_info.NUM_CINPUT_VARS > 0
    ni = sys_info.NUM_CINPUT_VARS;
    input_range = sys_info.CINPUT_BOUNDS;
    cp_array = 2*ones(1,ni);
    interpolationtype = {};
    for i = 1:ni
        interpolationtype = [interpolationtype 'UR'];
    end
    SampTime = 0.0067;
    model = @hycu_inputs_wrapper;
else
    SampTime = 0.05 ;% default in staliro_options
    input_range = [];
    cp_array =[];
    interpolationtype = {};
    model = @hycu_wrapper;
end


diary([sys_info.str 'P2__staliro_results']);
diary on;

initial_cond = sys_maps.initConstraints.cube;

phi = '!<>p';

preds(1).str = 'p';

poly = cubeToPoly(sys_maps.finalConstraints.cube);
preds(1).A = poly.A;
preds(1).b = poly.b;

[sys_maps.initConstraints,sys_maps.finalConstraints] = computePoly(sys_maps.initConstraints,sys_maps.finalConstraints);
if sys_info.BB == 1
    eventToTransitionInvMap = [];
    eventSet = [];
    terminalSet = [];
    eventInfo = [];
else
    [eventToTransitionInvMap,eventSet,terminalSet,eventInfo] = prepEventDetection(sys_info,sys_maps);
end


simTime = sys_opt.ScatterSim.MAX_SAMPLE_AGE;
opt = staliro_options();
opt.SampTime = SampTime;
opt.black_box = 1;
opt.taliro_metric = 'none';
opt.optimization_solver= 'SA_Taliro' ;
opt.spec_space = 'X';
opt.interpolationtype = interpolationtype;

% opt.runs = 1;
% opt.n_tests = 1;

opt.runs = 10;
opt.n_tests = 5000;

figure(1)
hold on

disp('Running S-TaLiRo with chosen solver ...')
opt
staliroTime = tic;
results = staliro(model,initial_cond,input_range,cp_array,phi,preds,simTime,opt);
runtime = toc(staliroTime);
results.run(results.optRobIndex).bestRob

% disp('inputSignal')
% disp(inputSignalStore')
% disp('X0')
% disp(results.run(results.optRobIndex).bestSample')

% [T1,XT1,YT1,IT1] = SimSimulinkMdl(model,initial_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample(:,1),simTime,opt);
% subplot(1,1,1) 
% plot(T1,YT1(:,1)) 
% 
% 



% 
% [T1,XT1,YT1,IT1] = SimFunctionMdl(model,initial_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample,simTime,opt);
% 
% figure(1)
% hold on
% plot(T1(:,1),XT1(:,2))

% cons_i = [-0.4 0.4; -0.4 0.4 ];
% line([cons_i(1,1) cons_i(1,2) cons_i(1,2) cons_i(1,1) cons_i(1,1)],[cons_i(2,1) cons_i(2,1) cons_i(2,2) cons_i(2,2) cons_i(2,1)],'color','g');
% cons_f = [-0.7  -1 ; -5.6  -6.5 ];
% line([cons_f(1,1) cons_f(1,2) cons_f(1,2) cons_f(1,1) cons_f(1,1)],[cons_f(2,1) cons_f(2,1) cons_f(2,2) cons_f(2,2) cons_f(2,1)],'color','b');

diary off;
end

function poly = cubeToPoly(cube)
NUM_STATE_VARS = size(cube,1);

Al = -eye(NUM_STATE_VARS);
bl = -cube(:,1);
Ah = eye(NUM_STATE_VARS);
bh = cube(:,2);

% collect indices of elements in b with infinity as the value
arrInf = [];
% remove all constraints such as x > -inf
for i = 1:NUM_STATE_VARS
    if bl(i) == inf %|| poly.b(i) == -inf, redundant. will never occur
        arrInf = [arrInf i];
    end
end

Al(arrInf,:) = 0;
bl(arrInf) = 0;

arrInf = [];
% remove all constraints such as x < inf
for i = 1:NUM_STATE_VARS
    if bh(i) == inf %|| poly.b(i) == -inf, redundant. will never occur
        arrInf = [arrInf i];
    end
end
Ah(arrInf,:) = 0;
bh(arrInf) = 0;

poly.A = [Al;Ah];
poly.b = [bl;bh];
end

function [initConstraints,finalConstraints] = computePoly(initConstraints,finalConstraints)
if initConstraints.isNonLinear == 0
    if initConstraints.isCube == 1
        poly = cubeToPoly(initConstraints.cube);
        initConstraints.A = poly.A;
        initConstraints.b = poly.b;
    end
else
    error('computePoly: init constraints neither lin nor nonLin!!')
end
% repeat with final constraints
if finalConstraints.isNonLinear == 0
    if finalConstraints.isCube == 1
        poly = cubeToPoly(finalConstraints.cube);
        finalConstraints.A = poly.A;
        finalConstraints.b = poly.b;
    end
else
    error('computePoly: final constraints neither lin nor nonLin!!')
end
end


function [eventToTransitionInvMap,eventSet,terminalSet,eventInfo] = prepEventDetection(sys_info,sys_maps)
eventToTransitionInvMap = {};
eventSet = {};
terminalSet = {};
eventID = 0;
eventInfo = {};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
transitionMap = sys_maps.transitionMap;
MODE_LIST = sys_info.MODE_LIST;
modeInvMap = sys_maps.modeInvMap;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for currMode = MODE_LIST
    %    fprintf('currmode = %d', currMode);
    P = [];
    q = [];
    F_arr = [];
    %transMap = modeToOutGoingTransitionLinGuardMap(currMode);
    % TODO: too inefficient, use mode2transition map
    eventToTransitionInvMap{currMode} = [];
    terminalArr = [];
    addPropDetect = 1;
    %% ADD Property detection
    if addPropDetect == 1
        errCons = sys_maps.finalConstraints;
        % error lies in the current mode, and hence add event detection for the error
        if currMode == errCons.mode || errCons.mode == -1
            if errCons.isNonLinear == 1
                F_arr = [F_arr;errCons.f_arr];
                numEvents = length(errCons.f_arr);
                error('prepEventDetection: NonLin error constraints unhandled:')
            elseif errCons.isNonLinear == 0
                P = [P;errCons.A];
                q = [q;errCons.b];
                numEvents = length(errCons.b);
            else
                error('prepEventDetection: error constraints neither lin nor nonLin!!')
            end
            % num of conjuncts, each conjunct equivalent to an event
            %numEvents = length(errCons.b) + length(errCons.f_arr);
            %TODO: change this to 0 or 1 later
            % new structure
            event = [];
            event.type = PROP_SAT;
            event.cons = errCons;
            eventID = eventID + 1;
            eventIDArr = eventID*ones(numEvents,1);
            eventInfo{eventID} = event;
            eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
            
            if numEvents > 1
                % do not terminate on event detection
                % instead do a post evaluation and decide if the transition was enabled or not
                terminalArr = [terminalArr;zeros(numEvents,1)];
            else
                % because there is only one transition, terminate immediately
                terminalArr = [terminalArr;1];
            end
            
        end
    end
    
    if sys_info.hybrid == 1
        %% ADD Transitions
        for toMode = MODE_LIST
            % inf implies that get all transitions
            %TODO: handle all transitions and not just 1!
            allTransitions = createTransitionID(currMode,toMode,inf);
            transitionCell = getTransition(allTransitions,transitionMap);
            for k = 1:length(transitionCell)
                transition = transitionCell{k};
                %transition = transition{1};
                guard = transition.guard;
                % else there is only one transition at idx 1
                if guard.isNonLinear == 0
                    P = [P;guard.A];
                    q = [q;guard.b];
                    numEvents = length(guard.b);
                elseif guard.isNonLinear == 1
                    F_arr = [F_arr;guard.f_arr];
                    numEvents = length(guard.f_arr);
                else
                    % TODO: print the mode name
                    error('simulate: guard.isNonLinear is neither 0 or 1!')
                end
                % num of conjuncts, each conjunct equivalent to an event
                %numEvents = length(guard.b) + length(guard.f_arr);
                %TODO: change this to 0 or 1 later
                % new structure
                event = [];
                event.type = TRANSITION;
                event.transitionID = createTransitionID(currMode,toMode,k);
                eventID = eventID + 1;
                eventIDArr = eventID*ones(numEvents,1);
                eventInfo{eventID} = event;
                eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
                if numEvents > 1
                    % do not terminate on event detection
                    % instead do a post evaluation and decide if the transition was enabled or not
                    terminalArr = [terminalArr;zeros(numEvents,1)];
                else
                    terminalArr = [terminalArr;zeros(numEvents,1)];
                    % disable special case handling because non-det still needs to be considered...use it when absolutely
                    % sure that non-det is not there and the transition is forcing....
                    
                    %                 disp(numEvents)
                    %                 error('transition:numEvents <= 1')
                    %                 % because there is only one transition, terminate immediately
                    %                 % TODO: this means non determinism is yet not handled!
                    %                 terminalArr = [terminalArr;1];
                end
            end
        end
        
        %% ADD Mode Invariants
        % TODO: enable with checks, if modeinv == guard
        modeInv = modeInvMap(currMode);
        if modeInv.isNonLinear == 0
            % negate the invariants because we want to detect when they are not
            % satisfied
            P = [P;-modeInv.A];
            q = [q;-modeInv.b];
            numEvents = length(modeInv.b);
        elseif modeInv.isNonLinear == 1
            F_arr = [F_arr;modeInv.f_arr];
            numEvents = length(modeInv.f_arr);
        else
            % TODO: print the mode name
            error('simulate: guard.isNonLinear is neither 0 or 1!')
        end
        %numEvents = length(modeInv.b) + length(modeInv.f_arr);
        % new structure
        event = [];
        event.type = MODE_INVARIANT;
        % should it be uniform?
        event.mode = currMode;
        eventID = eventID + 1;
        eventIDArr = eventID*ones(numEvents,1);
        eventInfo{eventID} = event;
        eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
        if numEvents > 1
            % do not terminate on event detection
            % instead do a post evaluation and decide if the transition was enabled or not
            terminalArr = [terminalArr;zeros(numEvents,1)];
        else
            %error('modeInv:numEvents <= 1')
            % because there is only one transition, terminate immediately
            % TODO: this means non determinism is yet not handled!
            terminalArr = [terminalArr;zeros(numEvents,1)];
        end
    end
    
    %% DONE
    eventSet{currMode}.Ab.A = P;
    eventSet{currMode}.Ab.b = q;
    eventSet{currMode}.f_arr = F_arr;
    terminalSet{currMode} = terminalArr;
    
end
end

function y = TRANSITION()
y = 0;
end

function y = MODE_INVARIANT()
y = 1;
end

function y = NO_EVENT_FOUND()
y = 2;
end

function y = PROP_SAT()
y = 3;
end


function transitionID = createTransitionID(fromMode,toMode,num)
transitionID.fromMode = fromMode;
transitionID.toMode = toMode;
transitionID.num = num;
end

function transitionCell = getTransition(transitionID,transitionMap)
transitionCell = transitionMap{transitionID.fromMode,transitionID.toMode};
% if not return {}
if isempty(transitionCell)
    transitionCell = {};
    return
end
if length(transitionCell) > 1
    warning('more than one transition to the same mode. EXPERIMENTAL');
end

if transitionID.num == inf
    % give back all transitions as a cell matrix
else
    % not really a cell!
    transitionCell = transitionCell{transitionID.num};
end

end
