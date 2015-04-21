function [ret_status,ret_T,ret_X] = S3CAM(sys_def,sys_prop,sys_abs,sys_opt)
silent = 1;
ret_status = 0;
sys_opt.simType = 'simulateInitSet';
sys_def.FILEPATH = [sys_def.str '-hycu-'];
FILEPATH = [sys_def.FILEPATH 'scatterSim'];
sys_opt.Abstraction.refinementFactor = 2;
sys_opt.ScatterSim.SCALE = ones(1,sys_def.NUM_STATE_VARS);

[sys_prop.initConstraints,sys_prop.finalConstraints] = computePoly(sys_prop.initConstraints,sys_prop.finalConstraints);

if silent == 0
    disp('initial')
    sys_prop.initConstraints.cube
    disp('property')
    sys_prop.finalConstraints.cube
    %%%%%%%%%%%%%%%%%
    % print run info
    fprintf(2,'%s\n','%%%%%%%%%%%%%%%%%%%%%%%% RUN INFO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    fprintf(2,sys_def.str);
    fprintf(2,'%s\n','%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    %%%%%%%%%%%%%%%%%
    
    CLOCK = clock;
    sprintf('Starting Time: %02d:%02d', CLOCK(4), CLOCK(5))
    % cleanup function, add ina func which user can call
    cleanupObj = onCleanup(@cleanupFun);
end


sys_def.transitionMap = [];
sys_def.NUM_MODES = 1;

% compute mode and time indexes
sys_def.TIME_IDX = sys_def.NUM_STATE_VARS*2 + 1;
sys_def.MODE_IDX = sys_def.TIME_IDX + 1;

runType = sys_opt.runType;
sys_opt.initFun();

% simulate
%% beeter, faster code than (phase == 10) and is more fair to DoD computations
if strcmp(runType, 'simulate')%phase == 11
    %     myFigure(1)
    %     hold on
    sys_abs.numSamples = 1e5;
    sys_opt.samplingType = 'UniformRand';
    [initCellMat,~] = generateSamples(sys_def,sys_prop,sys_abs,sys_opt);
    time_dod = tic;
    disp('simulating...');
    
    
    numViolations = simulateSystemFromInitArrBB(sys_def,sys_prop,sys_abs,sys_opt,initCellMat);
    
    
    % %     for currMode = MODE_LIST
    % %         fprintf('mode %d\n',currMode);
    % %         initSet = sys_info.stateSamples{currMode};
    % %         parfor i = 1:size(initSet,1)
    % %             [~,~,eventCode] = simulateSystemFromInit(sys_maps,sys_info,sys_opt,initSet(i,:),currMode);
    % %             if eventCode == PROP_SAT
    % %                 numViolations = numViolations + 1;
    % %             end
    % %         end
    % %     end
    
    fprintf(2,'number of violations: %d\n',numViolations);
    fprintf('time taken for DoD simulation: ');toc(time_dod);fprintf('\n');
    
    %% Interface with S-Taliro
    % This phase just uses the built in simulator to output values for
    % S-Taliro
elseif strcmp(runType, 'STaliro')%phase == 100
    
    
    
    [ret_T,ret_X] = simulateForSTaliroBB(sys_def,sys_prop,sys_abs,sys_opt,X0,M0,InputArr);
    
    
    %% cell based but the refinement is done only on initial set (cells):
    %% old version, does not used modes properly. won't work with systems with non-partitioned states
elseif strcmp(runType, 'analyzeInit')%phase == 24
    sys_opt.inputHandling.saveInputs = 1;
    sys_opt.inputHandling.scatterInputs = 0;
    
    myFigure(100)
    hold on;
    %     plotmap(100,'../nav_sim/nav30.mat')
    
    %     drawPropBox(sys_opt);
    grid_eps = sys_abs.grid_eps;
    %     inp_grid_eps = sys_opt.Abstraction.inp_grid_eps;
    NUM_CINPUT_VARS = sys_def.NUM_CINPUT_VARS;
    cons_i = sys_prop.initConstraints;
    %% get 0 measure initial conditions
    zeroMeasureDims = (cons_i.cube(:,2) - cons_i.cube(:,1) == 0);
    % create a 0 measure grid to reflect that: used for scatterX function, pins
    % down the the 0 measure states to X0
    sampling_grid_eps = grid_eps;
    sampling_grid_eps(zeroMeasureDims) = 0;
    
    
    initCells = generateCellsFromRange(cons_i.cube,sampling_grid_eps);
    sys_def.initCells = initCells;
    
    drawCells(initCells,grid_eps);
    drawPropBox(sys_opt);
    
    numSamplesPerCell = sys_abs.numSamples;
    initModeInv = sys_def.modeInvMap(sys_prop.initConstraints.mode);
    initArr = [];
    fprintf('generating init samples...');
    parfor i = 1:rows(initCells)
        cellToSample = initCells(i,:);
        % the scattered points are checked against the mode invariant
        cellSamples = scatterX(cellToSample,sys_def,numSamplesPerCell,sampling_grid_eps, initModeInv);
        initArr = [initArr;cellSamples];
    end
    fprintf('%d generated\n',rows(initArr));
    sys_def.initSet{cons_i.mode} = initArr;
    
    fprintf('cells to simulate %d\n',rows(initCells));
    
    
    if NUM_CINPUT_VARS == 0
        [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedBB(sys_def,sys_prop,sys_abs,sys_opt);
    else
        [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedMERGEDBUGGYFLATIP(sys_def,sys_prop,sys_abs,sys_opt,eventToTransitionInvMap,eventSet,terminalSet,eventInfo);
    end
    fprintf(2,'time taken for simulations: %f\n',simTime);
    disp('saving sim file')
    save(FILEPATH,'trajsCellMatXi','trajsCellMatXf','timeCellMat','trajMat');
    
    [Xi_actual,Xf_actual] = getXiXf(trajMat,sys_def,sys_prop,sys_abs,sys_opt);
    ret_status = (isempty(initArr) || isempty(finalArr));
    if ret_status == 0
        Xi = find(ismember(trajMat(:,1:sys_def.NUM_STATE_VARS),initArr,'rows'));
        Xf = find(ismember(trajMat(:,sys_def.NUM_STATE_VARS+1:2*sys_def.NUM_STATE_VARS),finalArr,'rows'));
        
        %     ret_status = checkXiXfEmpty(Xi,Xf,0);
        
        dumpSimData(trajsCellMatXi,trajsCellMatXf,timeCellMat,Xi,Xf,sys_def,sys_prop,sys_abs,sys_opt,1,sys_opt.ScatterSim.SCALE,FILEPATH);
        %TODO: temp fix to not get 0 as answer
        cg_ksp(FILEPATH,norm(grid_eps.*sys_opt.ScatterSim.SCALE,2));
    else
        % error('no candidate trajectory')
        fprintf(2,'DNF\n');
        return
    end
    prev_grid_eps = grid_eps;
    sampling_prev_grid_eps = sampling_grid_eps;
    %% load previous iteration results
    FILEPATHOrig = FILEPATH;
    maxIter = 20;
    maxNumPaths = 10000;
    %         maxNumPaths = 200;
    k = 0;
    for iterNum = 1:maxIter
        myFigure(iterNum)
        %         plotmap(iterNum,'../nav_sim/nav30.mat')
        hold on;
        %         drawPropBox(sys_opt);
        
        k = k + 1;
        if k == 1
            FILEPATH_r = FILEPATHOrig;
            FILEPATH_w = [FILEPATHOrig '-finer-iter' num2str(k)];
        else
            FILEPATH_r = [FILEPATHOrig '-finer-iter' num2str(k-1)];
            FILEPATH_w = [FILEPATHOrig '-finer-iter' num2str(k)];
        end
        fprintf('loading sim file...%s\n',FILEPATH_r)
        loadedMats = load(FILEPATH_r, '-mat');
        p = readKPaths([FILEPATH_r '.paths']);
        [costMat,inputCellMat] = prepMat4Opt(p,loadedMats.trajMat);
        uniqueSolCellMat = pruneSimilarTraj2(inputCellMat,sys_def);
        maxNumPaths = min(length(uniqueSolCellMat),maxNumPaths);
        fprintf(2,'maxNumPaths: %d\n',maxNumPaths);
        
        %% refine grid
        curr_grid_eps = prev_grid_eps/sys_opt.Abstraction.refinementFactor;
        sampling_curr_grid_eps = sampling_prev_grid_eps/sys_opt.Abstraction.refinementFactor;
        sys_abs.grid_eps = curr_grid_eps;
        
        %% get init cells from k-shortest paths of past iteration
        exploredCells = [];
        for pathIdx = 1:maxNumPaths
            solMat = uniqueSolCellMat{pathIdx};
            [X0,M0] = getX0AndM0(sys_def,sys_prop,sys_abs,sys_opt,solMat);
            if M0 ~= sys_prop.initConstraints.mode
                error('internal: M0 ~= sys_prop.initConstraints.mode');
            end
            oldGridCell = snapToGrid(X0,sampling_prev_grid_eps);
            if ~isempty(exploredCells) && ismember(oldGridCell,exploredCells,'rows')
                continue;
            else
                % add old cell to the list of explored cells
                exploredCells = [exploredCells;oldGridCell];
            end
            
        end
        fprintf('num. cells extracted from paths: %d\n',rows(exploredCells));
        
        %% subdivide successfull initial cells
        initCellsToSimulate = [];
        
        %% get 0 measure initial conditions
        fprintf('generating init samples...');
        initArr = [];
        for j = 1:rows(exploredCells)
            oldGridCell = exploredCells(j,:);
            newGridCells = subDivideCells(oldGridCell,sampling_prev_grid_eps,sampling_curr_grid_eps);
            for i = 1:rows(newGridCells)
                newGridCell = newGridCells(i,:);
                if checkIntersection(getCellRange(newGridCell,sampling_curr_grid_eps),cons_i.cube) == 1
                    initCellsToSimulate = [initCellsToSimulate;newGridCell];
                    cellSamples = scatterX(newGridCell,sys_def,numSamplesPerCell,sampling_curr_grid_eps, cons_i);
                    initArr = [initArr;cellSamples];
                end
            end
        end
        
        sys_def.initCells = initCellsToSimulate;
        
        drawCells(initCellsToSimulate,sampling_curr_grid_eps);
        drawPropBox(sys_opt);
        
        fprintf('cells to simulate %d\n',rows(initCellsToSimulate));
        
        
        fprintf('%d generated\n',rows(initArr));
        sys_def.initSet{cons_i.mode} = initArr;
        if isempty(initArr)
            error('empty init set')
        end
        
        
        %% check
        finalTraceMat = [];
        finalTraceMatFlag = [];
        
        
        
        sys_opt.inputHandling.saveInputs = 1;
        sys_opt.inputHandling.scatterInputs = 0;
        
        sys_def.initSet{cons_i.mode} = initArr;
        
        % initArr = [0.39887 -0.39216];
        %        0.3999     -0.39606
        
        fprintf('checking for errors trajs: %d\n',rows(initArr));
        %% simulate only cells
        
        tmpInitSet = cell(1,sys_def.NUM_MODES);
        
        % tmpInitSet{cons_i.mode} = initCellsToSimulate;
        tmpInitSet{cons_i.mode} = initArr;
        
        %% simulate cell samples
        % validInitIdxArr = checkConstraintSat(cons_i,initArr');
        % tmpInitSet = cell(1,sys_def.NUM_MODES);
        % tmpInitSet{cons_i.mode} = initArr(validInitIdxArr,:);
        % maxChecks = min(100,length(find(validInitIdxArr)));
        % tmpInitSet{cons_i.mode} = tmpInitSet{cons_i.mode}(1:maxChecks,:);
        
        if NUM_CINPUT_VARS == 0
            N = 1;
        else
            N = 4;
        end
        
        for r = 1:N
            
            numViolations = simulateSystemFromInitArrBB(sys_def,sys_prop,sys_abs,sys_opt,tmpInitSet);
            
            if numViolations > 0
                break;
            end;
        end
        if numViolations > 0
            fprintf(2,'%%%%%%%%%%%%%%%%%%%%%%\n');
            fprintf(2,'error trajectory/ies found!\n');
            fprintf(2,'%%%%%%%%%%%%%%%%%%%%%%\n');
            return
        else
            disp('bad luck!');
        end
        
        
        % % NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
        % % fprintf('checking for errors trajs. num samples: %d\n',size(initArr,1));
        % % M0 = cons_i.mode;
        % % parfor Idx = 1:size(initArr,1)
        % %     X0 = initArr(Idx,1:NUM_STATE_VARS);
        % %     if checkConstraintSat(cons_i,X0')
        % %         [~,~,eventCode] = simulateSystemFromInit(sys_maps,sys_info,sys_opt,X0,M0);
        % %         if eventCode == PROP_SAT
        % %             finalTraceMat(Idx,:) = [X0 M0];
        % %             finalTraceMatFlag(Idx) = 1;
        % %             fprintf(2,'%%%%%%%%%%%%%%%%%%%%%%\n');
        % %             fprintf(2,'error trajectory/ies found!\n');
        % %             fprintf(2,'%%%%%%%%%%%%%%%%%%%%%%\n');
        % %         end
        % %     end
        % % end
        % %
        % % if isempty(finalTraceMatFlag==1)
        % %     fprintf(2,'Bad Luck!!');
        % % else
        % %     disp(finalTraceMat(finalTraceMatFlag==1,:));
        % %     return;
        % % end
        
        
        
        
        
        %% repeats
        if NUM_CINPUT_VARS == 0
            [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedBB(sys_def,sys_prop,sys_abs,sys_opt);
        else
            [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedMERGEDBUGGYFLATIP(sys_def,sys_prop,sys_abs,sys_opt,eventToTransitionInvMap,eventSet,terminalSet,eventInfo);
        end
        
        fprintf(2,'time taken for simulations: %f\n',simTime);
        
        disp('saving sim file')
        save(FILEPATH_w,'trajsCellMatXi','trajsCellMatXf','timeCellMat','trajMat');
        
        ret_status = (isempty(initArr) || isempty(finalArr));
        [Xi_actual,Xf_actual] = getXiXf(trajMat,sys_def,sys_prop,sys_abs,sys_opt);
        if ret_status == 0
            Xi = find(ismember(trajMat(:,1:sys_def.NUM_STATE_VARS),initArr,'rows'));
            Xf = find(ismember(trajMat(:,sys_def.NUM_STATE_VARS+1:2*sys_def.NUM_STATE_VARS),finalArr,'rows'));
            
            %         ret_status = checkXiXfEmpty(Xi,Xf,0);
            
            dumpSimData(trajsCellMatXi,trajsCellMatXf,timeCellMat,Xi,Xf,sys_def,sys_prop,sys_abs,sys_opt,1,sys_opt.ScatterSim.SCALE,FILEPATH_w);
            cg_ksp(FILEPATH_w,norm(curr_grid_eps.*sys_opt.ScatterSim.SCALE,2));
        else
            %             error('no candidate trajectory')
            fprintf(2,'DNF\n');
            return
        end
        %% prev grid = current grid
        prev_grid_eps = curr_grid_eps;
        sampling_prev_grid_eps = sampling_curr_grid_eps;
    end
else
    error('unkown phase...what is it that you want to do?')
end
if silent == 0
    CLOCK = clock;
    sprintf('Ending Time: %02d:%02d', CLOCK(4), CLOCK(5))
end
end

%% snaps any state to the center of the appropriate cell in the current grid
function X_snapped = snapToGrid(X,grid_eps)
% X_snapped = truncate(X./grid_eps).*grid_eps + grid_eps/2;
grid_eps_mat = repmat(grid_eps,rows(X),1);
% why use truncate when fix is there?

% doesn't handle sign(0) properly! returns 0!!
% X_snapped = sign(X).*(fix(abs(X)./grid_eps_mat).*grid_eps_mat + grid_eps_mat/2);

% this one returns sign(0) = 1

X_snapped = mySign(X).*(fix(abs(X)./grid_eps_mat).*grid_eps_mat + grid_eps_mat/2);


% X_snapped = truncate(X./grid_eps_mat).*grid_eps_mat + grid_eps_mat/2;

%% restore the states that were not supposed to be snapped
nanIdx = isnan(X_snapped);
X_snapped(nanIdx) = X(nanIdx);

% X_snapped = truncate(X_snapped.*1e8)./1e8;
X_snapped = round(X_snapped.*1e10)./1e10;
end

%% snaps any state to the center of the appropriate cell in the current grid
% For some reasons, there is some residue remaining after computations!
function cellRange = getCellRange(X,grid_eps)
cellRange = [(X - grid_eps/2)' (X + grid_eps/2)'];
end

function gridCells = subDivideCells(oldCell, oldGrid_eps, newGrid_eps)
range(:,1) = (oldCell - oldGrid_eps/2 + newGrid_eps/2)';
range(:,2) = (oldCell + oldGrid_eps/2 - newGrid_eps/2)';

dummyGridEps = newGrid_eps;
zeroGridEpsIdx = (newGrid_eps == 0);
dummyGridEps(zeroGridEpsIdx) = 1;
%% sanity check
if range(zeroGridEpsIdx,1) ~= range(zeroGridEpsIdx,2)
    error('unhandled condition, grid_eps is 0 but range is non-zero measure')
end

numDim = rows(range);
H = cell(1,numDim);
rangeMat = cell(1,numDim);
for i = 1:numDim
    rangeMat{i} = range(i,1):dummyGridEps(i):range(i,2);
end
[H{:}] = ndgrid(rangeMat{:});
gridCells = [];
for i = 1:numDim
    gridCells = [gridCells H{i}(:)];
end
% if size(gridCells,1) > 4
%     error('sadasdas')
% end
gridCells = round(gridCells.*1e10)./1e10;
end

function gridCells = generateCellsFromRange(range, grid_eps)
% range(:,1) = snapToGrid(range(:,1)',grid_eps)';
% range(:,2) = snapToGrid(range(:,2)',grid_eps)';

% range(1,:) = snapToGrid(range(1,:),grid_eps);
% range(2,:) = snapToGrid(range(2,:),grid_eps);

% zeroMeasureDims = (range(:,2) - range(:,1) == 0);
cellRange = snapToGrid(range',grid_eps)';
% cellRange(zeroMeasureDims,:) = range(zeroMeasureDims,:);


% range = round(range.*1e10)./1e10;

dummyGridEps = grid_eps;
zeroGridEpsIdx = (grid_eps == 0);
dummyGridEps(zeroGridEpsIdx) = 1;
%% sanity check
if range(zeroGridEpsIdx,1) ~= range(zeroGridEpsIdx,2)
    error('unhandled condition, grid_eps is 0 but range is non-zero measure')
end
%%
numDim = rows(cellRange);
H = cell(1,numDim);
rangeMat = cell(1,numDim);
for i = 1:numDim
    rangeMat{i} = cellRange(i,1):dummyGridEps(i):cellRange(i,2);
end
[H{:}] = ndgrid(rangeMat{:});
gridCells = [];
for i = 1:numDim
    gridCells = [gridCells H{i}(:)];
end

gridCells = round(gridCells.*1e10)./1e10;

% gridCells = truncate(gridCells.*1e8)./1e8;

%
% sampling_grid_eps = grid_eps;
% sampling_grid_eps(zeroMeasureDims) = 0;
end

function cg_ksp(fileName,NNthresh)

% infoFileName = [fileName '-info.dat'];
pathFileName = [fileName '.paths'];
%graphFileName = [fileName '-' num2str(NNthresh) '.graph'];
graphFileName = [fileName '.graph'];

%disp(infoFileName)
%disp(graphFileName)
%disp(pathFileName)


numPaths = 10000;

% KTEST = './graphNpaths/grahel/ktest';
KTEST = './graphNpaths/k_paths/ktest'
CREATE_GRAPH = './graphNpaths/createGraph';

cgcmd = [CREATE_GRAPH ' ' fileName ' ' num2str(NNthresh)];
disp(cgcmd)

time_graph_build = tic;
unix(cgcmd);
fprintf('time taken to create Graph: ');toc(time_graph_build);fprintf('\n');

kspCmd = [KTEST ' ' num2str(numPaths) ' <' './' graphFileName '>' './' pathFileName];

disp(kspCmd)
time_k_short = tic;
unix(kspCmd);
fprintf('time taken to run Dijkstra: ');toc(time_k_short);fprintf('\n');

unix(['head -n 2 ' pathFileName]);
end

function [Xi_,Xf_] = getXiXf(trajMat_,sys_def,sys_prop,sys_abs,sys_opt)
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
MODE_IDX = sys_def.MODE_IDX;
initConstraints = sys_prop.initConstraints;
finalConstraints = sys_prop.finalConstraints;

disp('computing Xi Xf...')
cons_i = initConstraints;
cons_f = finalConstraints;
%         disp(cons_i)
Xi_ = find(checkConstraintSat(cons_i,trajMat_(:,1:NUM_STATE_VARS)'));
%         disp(Xi_)
%         disp(trajMat_(Xi_,MODE_IDX))
%disp('================================================')
%length(Xi_)
% don't check mode if its not applicable
% usefull when the hybrid state is defined over all modes
if cons_i.mode >= 0
    Xi_ = Xi_(find(trajMat_(Xi_,MODE_IDX) == cons_i.mode));
end
%length(Xi_)
Xf_ = find(checkConstraintSat(cons_f,trajMat_(:,NUM_STATE_VARS+1:2*NUM_STATE_VARS)'));
% don't check mode if its not applicable
% usefull when the hybrid state is defined over all modes
if cons_f.mode >= 0
    Xf_ = Xf_(find(trajMat_(Xf_,MODE_IDX) == cons_f.mode));
end
%disp('================================================')
Xi_ = Xi_';
Xf_ = Xf_';

end

function P = scatterX(X,sys_def,numPoints,scatterRadius, constraints)
% disp('=====================')
% numPoints
% disp('=====================')
%numPoints = 5;
numRPoints = numPoints*100;
% hyper Cube Side
s = scatterRadius/2;
a = (X - s);
%b = (X + s);
range = 2*s;
% scatter by picking random points
%P = rand(numRPoints,sys_def.NUM_STATE_VARS).*(repmat(b,numRPoints,1)-repmat(a,numRPoints,1))+repmat(a,numRPoints,1);
R = rand(numRPoints,sys_def.NUM_STATE_VARS);
P = scale_cols(R,range);
P = bsxfun(@plus, P, a);

zeroWidthIdx = range == 0;
P(:,zeroWidthIdx) = repmat(a(zeroWidthIdx),numRPoints,1);

% P
% if system is not hybrid, ignore invariants (this might stil be usefull if there is a Region Of Interest)
% if sys_info.hybrid == 1

%send constraints directly to scatterX
%modeInv = sys_def.modeInvMap(newMode);
%isSat = checkConstraintSat(modeInv,P');


isSat = checkConstraintSat(constraints,P');

%      error('lala')
% return only the satisfactory points
% P = P(isSat,:);

%seems redundant
%     actualNumPoints = min(numPoints,length(find(isSat)));
%     if actualNumPoints < numPoints/2

actualNumPoints = length(find(isSat));

% % if actualNumPoints < numPoints/2
% %     %     warning('<actualNumPoints < numPoints>')
% %     %     fprintf(2,'actualNumPoints:%d < numPoints:%d\n',actualNumPoints,numPoints);
% %     %     warning('\<actualNumPoints < numPoints>')
% %     %     P
% %     %disp(newMode)
% %     %disp(X)
% %     %     disp(maxJmp)
% %     %     disp(a)
% %     %     disp(b)
% % end


% % if actualNumPoints == 0
% %     warning('actualNumPoints = 0!')
% % end

if actualNumPoints < numPoints
    P = P(isSat,:);
else
    P = P(isSat,:);
    P = P(1:numPoints,:);
end
% else
%     P = P(1:numPoints,:);
% end

%P = [P; X];


%fprintf('scattered %d points within a Linf of %f\n',size(P,1),maxJmp);
end

%% treats cubes who share boundaries as a non-empty intersection
%TBD: VECTORIZE
function result = checkIntersection(cube1,cube2)

I = [max(cube1(:,1),cube2(:,1)) min(cube1(:,2),cube2(:,2))];
diff = I(:,2) - I(:,1);
% diff = fix(diff*1e8)/1e8;
result = all(diff >= 0);
% result = all((I(:,2) - I(:,1))>=0);
end

function [value, isterminal, direction] = events(~,X,~,~,currMode,eventSet,terminalSet)
%TODO: assumes all the events are described by a linear or all by non linear function
% Its a bit too strong when watching for both mode invariant and guards
% if non linear guards

% if ~isempty(eventSet{currMode}.Ab.b) && ~isempty(eventSet{currMode}.f_arr)
%     error('mix of non-lin and lin guards is UNHANDLED! sorry!')
% end
%
% if isempty(eventSet{currMode}.Ab.b)
%     f_arr = eventSet{currMode}.f_arr;
%     X_rep_arr = repmat({X},size(f_arr));
%     value = cellfun(@feval,f_arr,X_rep_arr);
% else
%     value = eventSet{currMode}.Ab.A*X-eventSet{currMode}.Ab.b;
% end
% numEvents = length(value);
% isterminal = terminalSet{currMode};
% direction = -1*ones(numEvents,1);

value_nlin = [];
value_lin = [];

if ~isempty(eventSet{currMode}.f_arr)
    %if isfield(eventSet{currMode},'f_arr')
    f_arr = eventSet{currMode}.f_arr;
    X_rep_arr = repmat({X},size(f_arr));
    value_nlin = cellfun(@feval,f_arr,X_rep_arr);
end
if ~isempty(eventSet{currMode}.Ab.b)
    %if isfield(eventSet{currMode},'Ab')
    value_lin = eventSet{currMode}.Ab.A*X-eventSet{currMode}.Ab.b;
end

value = [value_lin; value_nlin];
numEvents = length(value);
isterminal = terminalSet{currMode};
direction = -1*ones(numEvents,1);
% disp('a')



end

function [teout, Y, eventFound, event] = checkNreturnEvent(te,ye,ie,eventToTransitionInvMap,getEvent,transitionMap,sys_opt)
teout = [];Y = [];event = [];
eventFound = 0;
%disp(ie')
for i = 1:length(te)
    % TODO: round these off to prevent numerical issues from cropping up...specially when the guards are equality
    %  round to two digits
    %  Only for detection purposes...but note down the true values..
    %  Bring this option as a parameter outside
    Y = ye(i,:);
    Yr = round(Y*(10^sys_opt.GuardDetectRoundDigits))/(10^sys_opt.GuardDetectRoundDigits);
    teout = te(i);
    ieout = ie(i);
    % get the guard whose conjunct might have triggererd the event
    %     disp('--------')
    %     disp(eventToGuardMap{currMode}')
    eventID = eventToTransitionInvMap(ieout);
    % % ignore event id and check all guards going out from the current mode
    % % to find the guard that triggered the event
    % guardID = find(cellfun(@satGuard,guardMap));
    
    % only one guard per mode, hence idx is hard coded to 1
    event = getEvent{eventID};
    if event.type == TRANSITION
        transition = getTransition(event.transitionID,transitionMap);
        %transition = transition{1};
        isSat = checkConstraintSat(transition.guard,Yr');
        %newMode = event.transitionID.toMode;
    elseif event.type == MODE_INVARIANT
        %modeInv = modeInvMap(event.mode);
        isSat = 1;
        %         % if modeinvariant is unsatisfied, then there is an event
        %         disp(modeInv)
        %         disp(Y')
        %         isSat = ~checkConstraintSat(modeInv,Y');
        %eventObj.mode =
    elseif event.type == PROP_SAT
        isSat = checkConstraintSat(event.cons,Yr');
    else
        error('checkNreturnEvent: event neither TRANSITION or MODE_INVARIANT!')
    end
    if isSat == 1
        eventFound = 1;
        %         % debug print
        %         if event.type == TRANSITION
        %             %fprintf('transition to new mode %d\n', newMode);
        %             %disp(transition.guard.A)
        %             %disp(transition.guard.b)
        %         elseif event.type == MODE_INVARIANT
        %             %disp('mode-inv')
        %         end
        break;
    end
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

%% Graph Processing
function [paths]=readKPaths(FILEPATH)
filename = FILEPATH;
%filename = './graphNpaths/grahel/paths';
%filename = './paths';
%filename = ['./graphNpaths/grahel/' FILEPATH '-0.4' '.paths'];

%filename = ['./graphNpaths/grahel/bball-0.4-hycu-finer-0.1.paths'];

%filename = './graphNpaths/grahel/nav-20-0.2-hycu-scatterSim-0.5.paths';
disp(['reading in file:' filename]);
paths = csvread(filename);
end

function [d,p] = getPathInfo(path)
path(find(path==0)) = [];
d = path(end);
p = path(2:end-2);
end

%% Pre-process

%% Optimization
function [costMat,inputSolCellMat]=prepMat4Opt(p,trajMat)
numPaths = rows(p);
fprintf('num of trajectories/paths to optimize: %d\n',numPaths);
inputSolCellMat = {};
costMat = [];
optCtr = 1;
for i = 1:numPaths
    [d,path] = getPathInfo(p(i,:));
    input4OptMat = trajMat(path,:);
    oldTotalCost = d;
    inputSolCellMat{optCtr} = input4OptMat;
    costMat(optCtr,:) = [i oldTotalCost];
    optCtr = optCtr+1;
end
end

function uniqueSolMat = pruneSimilarTraj2(inputSolMat,sys_def)

disp('pruning similar trajs...')

MODE_IDX = sys_def.MODE_IDX;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
uniqueModeSeqIdx = {};
uniqueModeSeqBin = {};
idxCellMat = {};
n = length(inputSolMat);

for k = 1:n
    modeSeq = inputSolMat{k}(:,MODE_IDX);
    %% getModeSeqIdx(modeSeq);
    idx = 0;
    for i = 1:length(uniqueModeSeqIdx)
        if isequal(uniqueModeSeqIdx{i},modeSeq)
            idx = i;
            break;
        end
    end
    if idx == 0
        idx = length(uniqueModeSeqIdx) + 1;
        uniqueModeSeqIdx{idx} = modeSeq;
        uniqueModeSeqBin{idx} = [];
        idxCellMat{idx} = [];
    end
    %% end
    uniqueModeSeqBin{idx} = [uniqueModeSeqBin{idx}; inputSolMat{k}(1,1:NUM_STATE_VARS)];
    idxCellMat{idx} = [idxCellMat{idx}; k];
end

uniqueSolMat = {};
for i = 1:length(uniqueModeSeqBin)
    [~,uniqueIdx,~] = unique(uniqueModeSeqBin{i},'rows');
    idx = idxCellMat{i}(uniqueIdx);
    for j = idx'
        uniqueSolMat = [uniqueSolMat;inputSolMat(j)];
    end
end
end

function writeAdjModeGraph(fid,NUM_MODES,transitionMap,selfAdj)
% diagBelowLeft = 0;
% diagBelowRight = 0;
% diagAboveLeft = 0;
% diagAboveRight = 0;

%TBD: fix it properlty for non-hybrid systems
if NUM_MODES == 1
    fprintf(fid,'\nmode1: 1 mode1');
else
    for mode1 = 1:NUM_MODES
        adjModes = {};
        ctr = 0;
        fprintf(fid,'\nmode%d:',mode1);
        for mode2 = 1:NUM_MODES
            if ~isempty(transitionMap{mode1, mode2})
                ctr = ctr + 1;
                adjModes{ctr} = ['mode' num2str(mode2)];
            end
        end
        if selfAdj == 1
            adjModes{ctr+1} = ['mode' num2str(mode1)];
        end
        fprintf(fid,' %d',length(adjModes));
        for k = 1:length(adjModes)
            fprintf(fid,' %s',adjModes{k});
        end
    end
end
fprintf(fid,'\n');
end

% TODO: replace this with faster version
function writeMat(fid,A,name,integer)
precision = '5';
delim = ' ';
if nargin == 4
    if strcmp(integer,'int') == 1
        formatStr = ['%d' delim];
    else
        formatStr = ['%.' precision 'f' delim];
    end
else
    formatStr = ['%.' precision 'f' delim];
end
[m,n] = size(A);
if ~isempty(name)
    fprintf(fid,'name: %s\n',name);
end
fprintf(fid,['size: %d' delim '%d\n'],m,n);
for i = 1:m
    for j = 1:n
        fprintf(fid,formatStr,A(i,j));
    end
    %fprintf(fid,'\n');
end
fprintf(fid,'\n');
end

function [hdr,matStr] = writeMatFast(A,name,integer)
hdr = '';
matStr = '';
precision = '5';
delim = ' ';
if nargin == 4
    if strcmp(integer,'int') == 1
        formatStr = ['%d' delim];
    else
        formatStr = ['%.' precision 'f' delim];
    end
else
    formatStr = ['%.' precision 'f' delim];
end
[m,n] = size(A);
if ~isempty(name)
    hdr = sprintf('name: %s\n',name);
end

hdr = [hdr sprintf(['size: %d' delim '%d\n'],m,n)];
matStr = num2str(A,formatStr);
matStr(:,end+1) = ' ';
end

function dumpSimData(trajsCellMatXi,trajsCellMatXf,timeCellMat,Xi,Xf,sys_def,sys_prop,sys_abs,sys_opt,selfAdj,scalingArr,FILEPATH)
NUM_MODES = sys_def.NUM_MODES;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
%FILEPATH = sys_def.FILEPATH;
transitionMap = sys_def.transitionMap;

len = length(timeCellMat);


fprintf('scaling...')
for i = 1:len
    r = length(timeCellMat{i});
    if r ~= 0
        trajsCellMatXi{i} = trajsCellMatXi{i}.*repmat(scalingArr,r,1);
        trajsCellMatXf{i} = trajsCellMatXf{i}.*repmat(scalingArr,r,1);
    end
end
fprintf('done\n')


opFileName = [FILEPATH '-info.dat'];
disp(['dumping info...' opFileName])
% 'W' is faster! no explicit flushing till fclose() is called
fid = fopen(opFileName,'W');
% % wbhWrite = waitbar(0,'writing file...');
fprintf(fid,'numModes: %d\n',len);
fprintf(fid,'numStateVars: %d\n',NUM_STATE_VARS);
fprintf(fid,'modeList: ');
for i = 1:NUM_MODES
    fprintf(fid,'mode%d ',i);
end
fprintf(fid,'\n');
fprintf(fid,'adjGraph:');
writeAdjModeGraph(fid,NUM_MODES,transitionMap,selfAdj);

fprintf(fid,'trajsCellMatXi\n');
for i = 1:len
    % %     waitbar(i/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(trajsCellMatXi{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end

fprintf(fid,'trajsCellMatXf\n');
for i = 1:len
    % %     waitbar((1*len+i)/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(trajsCellMatXf{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end


fprintf(fid,'timeCellMat\n');
for i = 1:len
    % %     waitbar((2*len+i)/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(timeCellMat{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end

fprintf(fid,'Xi\n');
writeMat(fid,Xi,'','int');

fprintf(fid,'Xf\n');
writeMat(fid,Xf,'','int');

fclose(fid);

% % close(wbhWrite)
end

function isSat = checkConstraintSat(cons,X)

if cons.isNonLinear == 0
    % WHY IS Y a ROW VECTOR? and not column?
    % all() is used to realize conjunction
    %     disp(AX_b(Ab,Y'))
    
    cons.A = [cons.A;cons.A];
    cons.b = [cons.b;cons.b];
    
    numVectors = size(X,2);
    bMat = repmat(cons.b,1,numVectors);
    AX_b = cons.A*X-bMat;
    %     bMat
    %     cons.A
    %     X
    %     AX_b
    
    isSat = all(AX_b<=0,1);
    
    
elseif cons.isNonLinear == 1
    isSat = [];
    f_arr = cons.f_arr;
    for i = 1:length(f_arr)
        isSat = [isSat; f_arr{i}(X) <= 0];
    end
    isSat = all(isSat,1);
else
    error('checkConstraintSat: cons.isNonLinear is neither 0 or 1!')
end
end

% function satIdx = getXSatCons(cons,X)
% satIdx = find(checkConstraintSat(cons,X));
% end

%% utility funcs....TODO: delete them

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

function [initCellMat,paramCellMat] = generateSamples(sys_def,sys_prop,sys_abs,sys_opt)
fprintf('generating samples...\n')
if sys_def.NUM_PARAMS > 0
    paramCons = sys_maps.paramConstraints;
else
    paramCons.cube = [];
end
initCons = sys_prop.initConstraints;
if initCons.isNonLinear == 1
    error('generateSamples: non-linear initial constraints! where is monte carlo sampling?')
else
    if strcmp(sys_opt.samplingType,'UniformAll')
        if initCons.isCube == 1
            initCellMat = cell(1,sys_def.NUM_MODES);
            paramCellMat = cell(1,sys_def.NUM_MODES);
            range = initCons.cube;
            initCellMat{initCons.mode} = uniformSample(range,sys_def);
            if sys_def.NUM_PARAMS > 0
                range = paramCons.cube;
                paramCellMat{initCons.mode} = uniformSample(range,sys_def);
            end
        else
            error('need an algo for constrainted sampling: UNHANDLED!!')
        end
    elseif strcmp(sys_opt.samplingType,'UniformRand')
        initCellMat = cell(1,sys_def.NUM_MODES);
        paramCellMat = cell(1,sys_def.NUM_MODES);
        %range = initCons.cube;
        range = initCons.cube;
        randPoints = rand(sys_def.numSamples,sys_def.NUM_STATE_VARS);
        %randPoints = rand(sys_info.numSamples,sys_def.NUM_STATE_VARS);
        initCellMat{initCons.mode} = genRandVectors(randPoints,range);
        range = paramCons.cube;
        if sys_def.NUM_PARAMS > 0
            randPoints = rand(sys_info.numSamples,sys_def.NUM_PARAMS);
            paramCellMat{initCons.mode} = genRandVectors(randPoints,range);
        end
    elseif strcmp(sys_opt.samplingType,'Halton')
        error('most probbaly unhandled, verify before proceeding');
        initCellMat = cell(1,sys_info.NUM_MODES);
        range = initCons.cube;
        randPoints = haltonset(sys_def.NUM_STATE_VARS,'Leap',1e13);
        initCellMat{initCons.mode} = genRandVectors(randPoints(:,:),range);
    else
        error('sys_opt.samplingType unknown')
    end
end
disp('done')
end

function stateSamples = uniformSample(range,sys_def)
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
delta = sys_opt.delta;

totalNumSamples = prod(floor((range(:,2)-range(:,1))/delta)+1);
stateSamples = zeros(totalNumSamples,NUM_STATE_VARS);

lastC = 1;
for k = NUM_STATE_VARS:-1:1
    x = range(k,1):delta:range(k,2);
    repSamp = [];
    for i = 1:length(x)
        repSamp = [repSamp; repmat(x(i),lastC,1)];
    end
    stateSamples(:,k) = repmat(repSamp,totalNumSamples/length(repSamp),1);
    lastC = length(repSamp);
end
%disp(stateSamples)
return
end
function poly = cubeToPoly(c)
NUM_STATE_VARS = rows(c);
Al = -eye(NUM_STATE_VARS);
bl = -c(:,1);
Ah = eye(NUM_STATE_VARS);
bh = c(:,2);

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

% converts cubic/hyper-rectangle constraints on initial and final set to polyhedral constraints
% these are then stored in the respective structures
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

% is called whebever program ends, either normally or in error
function cleanupFun()
%toc;
CLOCK = clock;
sprintf('Ending Time: %02d:%02d', CLOCK(4), CLOCK(5))
end

function randVectors = genRandVectors(randPoints,range)
numRPoints = rows(randPoints);
a = range(:,1)';
b = range(:,2)';
A = repmat(a,numRPoints,1);
B = repmat(b,numRPoints,1);
randVectors = randPoints.*(B-A)+A;

% % values whose ranges have 0 width
% zeroWidthIdx = range(:,1) - range(:,2) == 0;
% randVectors(zeroWidthIdx) = range();
end

function [X0,M0] = getX0AndM0(sys_def,sys_prop,sys_abs,sys_opt,inputMat)
X0 = inputMat(1,1:sys_def.NUM_STATE_VARS);
M0 = inputMat(1,sys_def.MODE_IDX);
end

function drawPropBox(sys_opt)

sys_opt.drawProp();

% used for diabeticFisher
% line([0 1],[-0.9 -0.9])
% cons_f = sys_prop.finalConstraints.cube;
% line([cons_f(1,1) cons_f(1,2) cons_f(1,2) cons_f(1,1) cons_f(1,1)],[cons_f(2,1) cons_f(2,1) cons_f(2,2) cons_f(2,2) cons_f(2,1)],'color','b');

% used for bball_sinusoid
% cons_f = sys_prop.finalConstraints.cube;
% line([cons_f(5,1) cons_f(5,2) cons_f(5,2) cons_f(5,1) cons_f(5,1)],[cons_f(2,1) cons_f(2,1) cons_f(2,2) cons_f(2,2) cons_f(2,1)],'color','b');

% cons_i = sys_prop.initConstraints.cube;
% line([cons_i(1,1) cons_i(1,2) cons_i(1,2) cons_i(1,1) cons_i(1,1)],[cons_i(2,1) cons_i(2,1) cons_i(2,2) cons_i(2,2) cons_i(2,1)],'color','g');
end

% The number of rows.
% ROWS is a more readable alternative to size(x,1).
function r = rows(x)
r = size(x,1);
end

% TODO: Do not use with sparse matrices
function y = scale_cols(x, s)
y = x.*repmat(s(:).', rows(x), 1);
end

function drawCells(C,grid_eps)
for i = 1:rows(C)
    c = C(i,:);
    r = getCellRange(c,grid_eps);
    line([r(1,1) r(1,2) r(1,2) r(1,1) r(1,1)],[r(2,1) r(2,1) r(2,2) r(2,2) r(2,1)],'color','b','linewidth',3);
end
end

function s = mySign(X)
s = abs(X)./X;
nanIdx = isnan(s);
% replace sign(0) with 1
s(nanIdx) = 1;
end

function myFigure(h)
% myFigure(h);
isFigureHandle = ishandle(h) && strcmp(get(h,'type'),'figure');
if isFigureHandle == 1
    set(0,'CurrentFigure',h)
else
    figure(h);
end
end

function [ret_t,ret_X] = simulateForSTaliroBB(sys_def,sys_prop,sys_abs,sys_opt,X0,M0,~)
X0 = X0'; % S-Taliro gives a column vector
ret_t = 0;
ret_X = X0;

totalTime = sys_prop.TH;

delta = sys_abs.Delta;

simTime = 0;

% modeDynStruct = sys_maps.modeDynMap(M0);
% SIM = modeDynStruct.F;
SIM = sys_def.SIM;

% myFigure(20)
% hold on

cons_i = sys_prop.initConstraints;
initMode = cons_i.mode;
currMode = initMode;

while simTime < totalTime
    
    [t,X,currMode] = SIM([0 delta],X0,[],currMode);
    %     [t,X] = SIM([0 delta],X0,[]);
    ret_t = [ret_t;t + simTime];
    ret_X = [ret_X;X];
    
    X0 = X(end,:);
    simTime = simTime + t(end);
    
end
%
% plot(ret_t,ret_X(:,3));
% plot(ret_X(:,1),ret_X(:,2));
% drawnow


% % plot(ret_t,ret_X(:,2));
% plot(ret_X(:,1),ret_X(:,2));

end

function [finalArr,trajsCellMatXi,trajsCellMatXf,timeCellMat,trajMat,simTime] = simulateSystemCellBasedBB(sys_def,sys_prop,sys_abs,sys_opt)
time_sim = tic;
numViolations = 0;
plotTrajs = 1;
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

function numViolations = simulateSystemFromInitArrBB(sys_def,sys_prop,sys_abs,sys_opt,initSet_cellMat)
numViolations = 0;
ONE = 1;
% There is only one mode
NUM_CINPUT_VARS = sys_def.NUM_CINPUT_VARS;
if NUM_CINPUT_VARS ~= 0
    Uh = sys_def.CINPUT_BOUNDS(:,2);
    Ul = sys_def.CINPUT_BOUNDS(:,1);
else
    Uh = 0;
    Ul = 0;
end

% modeDynStruct = sys_maps.modeDynMap(M0);
% SIM = modeDynStruct.F;
SIM = sys_def.SIM;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
delta = sys_abs.Delta;
initSet = initSet_cellMat{ONE};

%% simulate!
totalTime = sys_prop.TH;

cons_f = sys_prop.finalConstraints;
cons_i = sys_prop.initConstraints;
initMode = cons_i.mode;
currMode = initMode;

% myFigure(1);hold on
% myFigure(2);hold on
% myFigure(3);hold on
% myFigure(4);hold on

% myFigure(5);hold on
% myFigure(6);hold on
%
% myFigure(7);hold on
% myFigure(8);hold on
% myFigure(9);hold on
% myFigure(10);hold on
% myFigure(11);hold on
% myFigure(12);hold on
% myFigure(13);hold on


% parfor
for i = 1:rows(initSet)
    propHit = 0;
    simTime = 0;
    X0 = initSet(i,1:NUM_STATE_VARS);
    cip = rand(NUM_CINPUT_VARS,1).*(Uh-Ul)+Ul;
    
    savedX0 = X0;
    savedCIP = cip;
    
    while propHit ~= 1 && simTime < totalTime
        
        if NUM_CINPUT_VARS == 0
            [t,X] = SIM([0 totalTime],X0,[],initMode);
            % TBD: optimize below common parts, and remove check
            Xf = X(end,:);
            tf = t(end);
            %             if tf ~= delta
            %                 error('tf~=delta for BB, something wrong with SIM??')
            %             end
        else
            [t,X,currMode] = SIM([0 delta],X0,cip,currMode);
            % TBD: optimize below common parts, and remove check
            Xf = X(end,:);
            tf = t(end);
            if tf ~= delta
                error('tf~=delta for BB, something wrong with SIM??')
            end
        end
        
        %         propHit = checkProp(t,X,cons_f);
        propHit = any(checkConstraintSat(cons_f,X'));
        %         propHit = 0;
        
        if propHit ~= 1
            cip = rand(NUM_CINPUT_VARS,1).*(Uh-Ul)+Ul;
            savedCIP = [savedCIP cip];
        else
            warning('prop-sat')
            disp('X0,i/p')
            disp(savedX0)
            disp(savedCIP)
            numViolations = numViolations + 1;
        end
        
        %         %                 uncomment to plot
        sys_opt.plotFun(t+simTime,X,currMode);
        %                                 drawnow
        
        X0 = Xf;
        simTime = simTime + tf;
    end
    
end
end

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

options = odeset('Events',@events,'Refine',4,'RelTol',1e-6);%,'MaxStep',1e-4);

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
