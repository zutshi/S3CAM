
function [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedBB(sys_def,sys_prop,sys_abs,sys_opt)
time_sim = tic;
numViolations = 0;
plotTrajs = 0;
if plotTrajs == 1
    hold on;
end
ONE = 1;
MAX_SAMPLE_AGE = sys_prop.TH;
grid_eps = sys_abs.grid_eps;

NUM_STATE_VARS = sys_def.NUM_STATE_VARS;

exploredCells = sys_def.initCells;

cons_i = sys_prop.initConstraints;
cons_f = sys_prop.finalConstraints;
finalArr = [];

numSamplesPerCell = sys_abs.numSamples;

% hack to fix mode issue!
initSet = cell(1,ONE);
initSet{cons_i.mode} = sys_def.initSet{cons_i.mode};

Delta = sys_abs.Delta;

trajsCellMatXi = cell(1,ONE);
trajsCellMatXf = cell(1,ONE);
timeCellMat = cell(1,ONE);

trajMat = [];

modeInv = sys_def.modeInvMap(ONE);

if ~isempty(initSet{ONE})
    numSamples = rows(initSet{ONE});
    XArr = initSet{ONE};
    propHitArr = zeros(numSamples,1);
    StateArr = ones(numSamples,1);
    disp(rows(XArr))
else
    return
end
AGE = 0;

SIM = sys_def.SIM;
while AGE < MAX_SAMPLE_AGE && rows(XArr) ~= 0
    
    numInitStates = rows(XArr);
    fprintf('simulating %d samples\n',numInitStates);
    
    
    trajsMatXi = zeros(numInitStates,NUM_STATE_VARS);
    trajsMatXf = zeros(numInitStates,NUM_STATE_VARS);
    timeMat = zeros(numInitStates,1);
    
    
    plotSet = cell(1,numInitStates);
    
    % parfor here
    parfor i = 1:numInitStates
%     for i = 1:numInitStates
        vio = 0;
        X0 = XArr(i,1:NUM_STATE_VARS);
        [t,X,StateArr(i)] = SIM([0 Delta],X0,[],StateArr(i));
        Xf = X(end,:);
        tf = t(end);
        
        %         if tf ~= Delta
        %             error('tf~=delta for BB, something wrong with SIM??')
        %         end
        propHit = any(checkConstraintSat(cons_f,X'));
        if propHit == 1
            numViolations = numViolations+1;
            %             warning('prop-sat')
            finalArr = [finalArr; Xf];
            vio = 1;
        end
        % TODO: make transition objects carry from and to info
        trajsMatXi(i,:) = X0;
        trajsMatXf(i,:) = Xf;
        timeMat(i,:) = tf;
        
        %% check if landed on an error cell
        if vio == 0
            X_snapped = snapToGrid(Xf,grid_eps);
            if checkIntersection(getCellRange(X_snapped,grid_eps),cons_f.cube) == 1
                finalArr = [finalArr; Xf];
                propHit = 1;
            end
        end
        %TODO: better detection of a new mode is required for non-deterministic transitions
        %% debug
        if plotTrajs == 1
            plotSet{i} = [X t+AGE];
        end
        propHitArr(i,1) = propHit;
    end
    AGE = AGE + Delta;
    %% debug
    if plotTrajs == 1
        for j = 1:length(plotSet)
            sys_opt.plotFun(plotSet{j}(:,NUM_STATE_VARS+1),plotSet{j}(:,1:NUM_STATE_VARS),ONE);
        end
        drawnow;
    end
    
    trajsCellMatXi{ONE} = [trajsCellMatXi{ONE}; trajsMatXi];
    trajsCellMatXf{ONE} = [trajsCellMatXf{ONE}; trajsMatXf];
    timeCellMat{ONE} = [timeCellMat{ONE}; timeMat];
    
    filterIdx = propHitArr == 1;
    filtered_XArr = trajsMatXf(~filterIdx,:);
    filteredStateArr = StateArr(~filterIdx,:);
    newX_arr = [];
    newState_arr = [];
    
    numStates = rows(filtered_XArr);
    
    % TBD: parallelize
    for k = 1:numStates
        X = filtered_XArr(k,:);
        
        % snap X to grid
        X_snapped = snapToGrid(X,grid_eps);
        
        if isempty(exploredCells) || ~ismember(X_snapped,exploredCells,'rows')
            % scatter only inside the cell
            scatteredX = scatterX(X_snapped,sys_def,numSamplesPerCell,grid_eps, modeInv);
            n = rows(scatteredX);
            newX_arr = [newX_arr; scatteredX];
            newState_arr = [newState_arr; repmat(filteredStateArr(k),n,1)];
            exploredCells = [exploredCells;X_snapped];
        end
    end
    XArr = newX_arr;
    StateArr = newState_arr;
    propHitArr = zeros(rows(XArr),1);
end

% create remaining matrices
% NUM_MODES = length(MODE_LIST);
NUM_MODES = ONE;
trajOffsetMat = zeros(NUM_MODES+1,1);
trajMat = [];
% for ONE = 1:NUM_MODES
numTrajsInCurrMode = length(timeCellMat{ONE});
trajOffsetMat(ONE+1) = numTrajsInCurrMode;
trajMat = [trajMat; trajsCellMatXi{ONE} trajsCellMatXf{ONE} timeCellMat{ONE} repmat(ONE,numTrajsInCurrMode,1)];
% end
fprintf(2,'number of recorded violations %d\n',numViolations);
simTime = toc(time_sim);
end


