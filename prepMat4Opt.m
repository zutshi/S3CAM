
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
