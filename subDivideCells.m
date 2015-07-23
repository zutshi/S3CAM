
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
