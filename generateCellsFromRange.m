
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
