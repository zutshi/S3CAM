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
