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


% is called whebever program ends, either normally or in error
function cleanupFun()
%toc;
CLOCK = clock;
sprintf('Ending Time: %02d:%02d', CLOCK(4), CLOCK(5))
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

function drawCells(C,grid_eps)
for i = 1:rows(C)
    c = C(i,:);
    r = getCellRange(c,grid_eps);
    line([r(1,1) r(1,2) r(1,2) r(1,1) r(1,1)],[r(2,1) r(2,1) r(2,2) r(2,2) r(2,1)],'color','b','linewidth',3);
end
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

