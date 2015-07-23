
%% snaps any state to the center of the appropriate cell in the current grid
% For some reasons, there is some residue remaining after computations!
function cellRange = getCellRange(X,grid_eps)
cellRange = [(X - grid_eps/2)' (X + grid_eps/2)'];
end
