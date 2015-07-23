
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
        randPoints = rand(sys_abs.numSamples,sys_def.NUM_STATE_VARS);
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
