
function [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedMERGEDBUGGYFLATIP(sys_def,sys_prop,sys_abs,sys_opt,eventToTransitionInvMap,eventSet,terminalSet,eventInfo)

time_sim = tic;
numViolations = 0;
plotTrajs = 0;

if sys_def.NUM_PARAMS ~= 0
    error('parameter handlind disabled: code them in as regular state variables')
end

MAX_SWITCHINGS = sys_opt.ScatterSim.MAX_SWITCHINGS;
MAX_SAMPLE_AGE = sys_prop.TH;


grid_eps = sys_abs.grid_eps;

modeDynMap = sys_maps.modeDynMap;
transitionMap = sys_maps.transitionMap;
MODE_LIST = sys_def.MODE_LIST;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;


exploredCells = sys_def.initCells;

cons_i = sys_prop.initConstraints;
cons_f = sys_prop.finalConstraints;
finalArr = [];

numSamplesPerCell = sys_abs.numSamples;

% hack to fix mode issue!
initSet = cell(1,sys_def.NUM_MODES);
initSet{cons_i.mode} = sys_def.initSet{cons_i.mode};

options = odeset('Events',@event_fun,'Refine',4,'RelTol',1e-6);%,'MaxStep',1e-4);

% This assumes that the max dwell time in a mode is 100s
% MODE_TIME_HORIZON = sys_opt.MODE_TIME_HORIZON;
Delta = sys_abs.Delta;
trajsCellMatXi = cell(1,sys_def.NUM_MODES);
trajsCellMatXf = cell(1,sys_def.NUM_MODES);
timeCellMat = cell(1,sys_def.NUM_MODES);
trajMat = [];

% [eventToTransitionInvMap,eventSet,terminalSet,eventInfo] = prepEventDetection(sys_def,sys_maps);

NUM_CINPUT_VARS = sys_def.NUM_CINPUT_VARS;

XArr = [];
ageArr = [];
samples_numSwitches =[];
currModeArr = [];
THArr = [];
for currMode = MODE_LIST
    if ~isempty(initSet{currMode})
        numSamples = rows(initSet{currMode});
        XArr = [XArr; initSet{currMode}];
        ageArr = [ageArr; zeros(numSamples,1)];
        samples_numSwitches =[samples_numSwitches; zeros(numSamples,1)];
        disp(rows(XArr))
        currModeArr = [currModeArr; repmat(currMode,numSamples,1)];
        THArr = [THArr; Delta*ones(numSamples,1)];
        if NUM_CINPUT_VARS ~= 0
            randPoints = rand(numSamples,NUM_CINPUT_VARS);
            samples_Inp = genRandVectors(randPoints,sys_def.CINPUT_BOUNDS);
        else
            samples_Inp = [];
        end
    end
end


while ~isempty(XArr)
    if NUM_CINPUT_VARS ~= 0
        InpArr = samples_Inp;
    else
        InpArr = [];
    end
    
    numInitStates = rows(XArr);
    fprintf('simulating %d samples\n',numInitStates);
    
    
    trajsMatXi = zeros(numInitStates,NUM_STATE_VARS);
    trajsMatXf = zeros(numInitStates,NUM_STATE_VARS);
    timeMat = zeros(numInitStates,1);
    newModeArr = [];
    
    plotSet = cell(1,numInitStates);
    
    switchFlagArr = zeros(numInitStates,1);
    
    % parfor here
    parfor i = 1:numInitStates
        vio = 0;
        X0 = XArr(i,1:NUM_STATE_VARS);
        currMode = currModeArr(i);
        eventToTransitionInv = eventToTransitionInvMap{currMode};
        modeDynStruct = modeDynMap(currMode);
        if NUM_CINPUT_VARS ~= 0
            if modeDynStruct.isNonLinear == 0
                modeDyn = @(~,X,~,U,~,~,~) modeDynStruct.A*X + modeDynStruct.b + modeDynStruct.B*U;
            else
                modeDyn = @(t,X,~,U,currMode,~,~) modeDynStruct.F(t,X,U,currMode);
            end
        else
            if modeDynStruct.isNonLinear == 0
                modeDyn = @(~,X,~,~,~,~,~) modeDynStruct.A*X + modeDynStruct.b;
            else
                modeDyn = @(t,X,~,~,currMode,~,~) modeDynStruct.F(t,X,[],currMode);
            end
        end
        TH = THArr(i,1);
        
        if NUM_CINPUT_VARS == 0
            [t,X,te,ye,ie] = ode45(modeDyn,[0 TH],X0,options,[],[],currMode,eventSet,terminalSet);
        else
            cip = InpArr(i,1);
            [t,X,te,ye,ie] = ode45(modeDyn,[0 TH],X0,options,[],cip,currMode,eventSet,terminalSet);
        end
        [teout, Y, eventFound, event] = checkNreturnEvent(te,ye,ie,eventToTransitionInv,eventInfo,transitionMap,sys_opt);
        if isempty(te) || ~eventFound
            newMode = currMode;
            Xf = X(end,:);
            tf = t(end);
            if tf ~= TH
                error('internal: not possible! floating point errors?')
            end
            THArr(i,1) = 0;
        elseif event.type == TRANSITION
            %             warning('TRANSITION')
            newMode = event.transitionID.toMode;
            transition = getTransition(event.transitionID,transitionMap);
            if transition.reset.isNonLinear == 1
                Xf = transition.reset.F(Y')';
            elseif transition.reset.isNonLinear == 0
                Xf = (transition.reset.A*Y' + transition.reset.b)';
            else
                error('simulate: reset.isNonLinear is neither 0 or 1!')
            end
            tf = teout;
            switchFlagArr(i) = 1;
            THArr(i,1) = THArr(i,1) - tf;
        elseif event.type == MODE_INVARIANT
            newMode = -1;
            Xf = Y;
            tf = teout;
            %             warning('invariant hit!')
        elseif event.type == PROP_SAT
            newMode = -1;
            Xf = Y;
            tf = teout;
            numViolations = numViolations+1;
            %error('prop-sat')
            finalArr = [finalArr; Xf];
            vio = 1;
        else
            error('simulateSystem: not possible to reach here')
        end
        % TODO: make transition objects carry from and to info
        trajsMatXi(i,:) = X0;
        trajsMatXf(i,:) = Xf;
        timeMat(i,:) = tf;
        
        %% check if landed on an error cell
        if vio == 0%
            X_snapped = snapToGrid(Xf,grid_eps);
            if checkIntersection(getCellRange(X_snapped,grid_eps),cons_f.cube) == 1
                finalArr = [finalArr; Xf];
                newMode = -1;
            end
        end
        
        %TODO: better detection of a new mode is required for non-deterministic transitions
        %% debug
        if plotTrajs == 1
            idx = (t <= tf);
            t = [t(idx); tf];
            X = [X(idx,:); Xf];
            plotSet{i} = [X t+ageArr(i,1)];
        end
        newModeArr(i,1) = newMode;
        ageArr(i,1) = ageArr(i,1) + tf;
    end
    %% debug
    if plotTrajs == 1
        for j = 1:length(plotSet)
            sys_opt.plotFun(plotSet{j}(:,NUM_STATE_VARS+1),plotSet{j}(:,1:NUM_STATE_VARS),currModeArr(j));
        end
        drawnow;
    end
    
    for currMode = MODE_LIST
        idx = currModeArr == currMode;
        trajsCellMatXi{currMode} = [trajsCellMatXi{currMode}; trajsMatXi(idx,:)];
        trajsCellMatXf{currMode} = [trajsCellMatXf{currMode}; trajsMatXf(idx,:)];
        timeCellMat{currMode} = [timeCellMat{currMode}; timeMat(idx)];
    end
    
    numSwitchArr = samples_numSwitches + switchFlagArr;
    
    filterIdx = (newModeArr < 0 | ageArr >= MAX_SAMPLE_AGE | numSwitchArr >= MAX_SWITCHINGS);
    
    %fprintf('suspending %d\n', length(find(filterIdx)))
    % invert the indices and assign only the values which are not suspended
    filtered_newModeArr = newModeArr(~filterIdx);
    filtered_ageArr = ageArr(~filterIdx);
    filtered_XArr = trajsMatXf(~filterIdx,:);
    filtered_swichArr = numSwitchArr(~filterIdx);
    filtered_THArr = THArr(~filterIdx);
    if NUM_CINPUT_VARS ~= 0
        filtered_InpArr = InpArr(~filterIdx);
    else
        filtered_InpArr = [];
    end
    newX_arr = [];
    newNewModeArr = [];
    newAgeArr = [];
    newNumSwitchesArr = [];
    newTHArr = [];
    newInpArr = [];
    numStates = length(filtered_newModeArr);
    
    %% new code for scattering:
    % - does not scatter at guards
    % - scatters more uniformly
    
    
    % TBD: parallelize
    for k = 1:numStates
        X = filtered_XArr(k,:);
        newMode = filtered_newModeArr(k);
        age = filtered_ageArr(k);
        Sw = filtered_swichArr(k);
        TH = filtered_THArr(k);
        if NUM_CINPUT_VARS ~= 0
            Inp = filtered_InpArr(k);
        else
            Inp = [];
        end
        
        if TH ~= 0
            newX_arr = [newX_arr;X];
            newNewModeArr = [newNewModeArr;newMode];
            newAgeArr = [newAgeArr;age];
            newNumSwitchesArr = [newNumSwitchesArr;Sw];
            newTHArr = [newTHArr;TH];
            newInpArr = [newInpArr;Inp];
        else
            modeInv = sys_def.modeInvMap(newMode);
            % snap X to grid
            X_snapped = snapToGrid(X,grid_eps);
            
            if isempty(exploredCells) || ~ismember(X_snapped,exploredCells,'rows')
                % scatter only inside the cell
                scatteredX = scatterX(X_snapped,sys_def,numSamplesPerCell,grid_eps, modeInv);
                n = rows(scatteredX);
                
                newX_arr = [newX_arr; scatteredX];
                newNewModeArr = [newNewModeArr; repmat(newMode,n,1)];
                % all the new scattered points inherit the age of their father/mother
                newAgeArr = [newAgeArr; repmat(age,n,1)];
                newNumSwitchesArr = [newNumSwitchesArr;repmat(Sw,n,1)];
                newTHArr = [newTHArr;Delta*ones(n,1)];
                if NUM_CINPUT_VARS ~= 0
                    randPoints = rand(n,NUM_CINPUT_VARS);
                    sInp = genRandVectors(randPoints,sys_def.CINPUT_BOUNDS);
                    newInpArr = [newInpArr;sInp];
                end
                % TBD: cells need to be cateogorized as by modes!! this
                % impl. is wrong for non-paritioned state space Hybrid
                % systems!
                exploredCells = [exploredCells;X_snapped];
            end
        end
    end
    
    XArr = newX_arr;
    ageArr = newAgeArr;
    samples_numSwitches = newNumSwitchesArr;
    currModeArr = newNewModeArr;
    THArr = newTHArr;
    
    if NUM_CINPUT_VARS ~= 0
        samples_Inp = newInpArr;
    end
    
    
end

% create remaining matrices
NUM_MODES = length(MODE_LIST);
trajOffsetMat = zeros(NUM_MODES+1,1);
trajMat = [];
for currMode = 1:NUM_MODES
    numTrajsInCurrMode = length(timeCellMat{currMode});
    trajOffsetMat(currMode+1) = numTrajsInCurrMode;
    trajMat = [trajMat; trajsCellMatXi{currMode} trajsCellMatXf{currMode} timeCellMat{currMode} repmat(currMode,numTrajsInCurrMode,1)];
end
fprintf(2,'number of recorded violations %d\n',numViolations);
simTime = toc(time_sim);
end
