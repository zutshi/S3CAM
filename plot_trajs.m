function plot_trajs()
% Because plots are projections,
% specify two dimensions two plot
% against each other
global PLOT_TITLE;
PLOT_TITLE = 'MPC';
% plot <x> vs <y>
global PLOT_x_y;
PLOT_x_y = [1 3];

%% save generated figures?
global SAVE_FIGS;
SAVE_FIGS = 1;

% plot_iterations()
plot_violating_traj()
end

function plot_iterations()

%% setup

ordered_file_list = {'MPC-hycu-scatterSim-save.mat';
    'MPC-hycu-scatterSim-finer-iter1-save.mat';
    'MPC-hycu-scatterSim-finer-iter2-save.mat';
    'MPC-hycu-scatterSim-finer-iter3-save.mat'};

grid_eps = [1.0, 1.0, 1.0];

NUM_DIM = length(grid_eps);


% How many trajectories to plot from a given cell pai A -> B
% currently choice is limited to 1 or all!
% TRAJ_PER_EDGE = 1;
TRAJ_PER_EDGE = inf;


%% plot each file
for j = 1:length(ordered_file_list)
    
    cell_list_i = inf*ones(1, NUM_DIM);
    cell_list_f = inf*ones(1, NUM_DIM);
    
    file_name = ordered_file_list{j};
    load(file_name)
    
    trajsXf = trajsCellMatXf{1};
    trajsXi = trajsCellMatXi{1};
    
    %% sanity check
    if all(size(trajsXi) == size(trajsXf)) == 1
    else
        error('Xi and Xf arrays not of same size!')
    end
    
    %% figure
    fig = figure();
    title([PLOT_TITLE ' - Iteration' num2str(j)]);
    xlabel(['x' num2str(PLOT_x_y(1))]);
    ylabel(['x' num2str(PLOT_x_y(2))]);
    hold on
    
    %% color cells in two passes, otherwise multi color cells result due to overwriting!
    reg_cells = [];
    init_cells = [];
    error_cells = [];
    
    %% collect cells and plot trajs
    for i = 1:size(trajsXi, 1)
        plot_traj = 0;
        
        %% collect unique 'source' cells [vertices at the begining of edges]
        C = snapToGrid(trajsXi(i, :), grid_eps);
        if ~ismember(C, cell_list_i, 'rows') || checkIntersection(getCellRange(C, grid_eps), init_cube) == 1
            plot_traj = plot_traj + 1;
            cell_list_i = [cell_list_i; C];
            if checkIntersection(getCellRange(C, grid_eps), init_cube) == 1
                %             if any(Xi == i)
                init_cells = [init_cells; C];
            else
                reg_cells = [reg_cells; C];
            end
        end
        
        %% collect unique 'sink' cells [vertices at the end of edges]
        C = snapToGrid(trajsXf(i, :), grid_eps);
        if ~ismember(C, cell_list_f, 'rows')
            plot_traj = plot_traj + 1;
            cell_list_f = [cell_list_f; C];
            if checkIntersection(getCellRange(C, grid_eps), final_cube) == 1
                %             if any(Xf == i)
                error_cells = [error_cells; C];
            else
                reg_cells = [reg_cells; C];
            end
        end
        
        %% plot trajectory
        if TRAJ_PER_EDGE == 1
            if plot_traj == 2
                plot([trajsXi(i, PLOT_x_y(1)), trajsXf(i, PLOT_x_y(1))], [trajsXi(i, PLOT_x_y(2)), trajsXf(i, PLOT_x_y(2))])
            end
        elseif TRAJ_PER_EDGE == inf
            plot([trajsXi(i, PLOT_x_y(1)), trajsXf(i, PLOT_x_y(1))], [trajsXi(i, PLOT_x_y(2)), trajsXf(i, PLOT_x_y(2))])
        end
    end
    
    %% draw regular cells
    for i = 1:size(reg_cells, 1)
        C = reg_cells(i, :);
        color = 'b';
        drawCells(C, grid_eps, color);
    end
    %% draw init cells
    for i = 1:size(init_cells, 1)
        C = init_cells(i, :);
        color = 'g';
        drawCells(C, grid_eps, color);
    end
    %% draw final cells
    for i = 1:size(error_cells, 1)
        C = error_cells(i, :);
        color = 'r';
        drawCells(C, grid_eps, color);
    end
    
    if SAVE_FIGS == 1
        fig_file_name = fullfile('saved_figs', ['iter_' num2str(j) '.fig']);
        savefig(fig, fig_file_name, 'compact')
    end
    
    grid_eps = grid_eps/2;
end
end

function cellRange = getCellRange(X,grid_eps)
cellRange = [(X - grid_eps/2)' (X + grid_eps/2)'];
end

function X_snapped = snapToGrid(X,grid_eps)
grid_eps_mat = repmat(grid_eps,size(X,1),1);
X_snapped = mySign(X).*(fix(abs(X)./grid_eps_mat).*grid_eps_mat + grid_eps_mat/2);
nanIdx = isnan(X_snapped);
X_snapped(nanIdx) = X(nanIdx);
X_snapped = round(X_snapped.*1e10)./1e10;
end

function drawCells(C,grid_eps, color)
global PLOT_x_y;

for i = 1:size(C,1)
    c = C(i,:);
    r = getCellRange(c,grid_eps);
    %     line([r(1,1) r(1,2) r(1,2) r(1,1) r(1,1)],[r(2,1) r(2,1) r(2,2) r(2,2) r(2,1)],'color',color,'linewidth',3);
    line([r(PLOT_x_y(1),1) r(PLOT_x_y(1),2) r(PLOT_x_y(1),2) r(PLOT_x_y(1),1) r(PLOT_x_y(1),1)],...
        [r(PLOT_x_y(2),1) r(PLOT_x_y(2),1) r(PLOT_x_y(2),2) r(PLOT_x_y(2),2) r(PLOT_x_y(2),1)],...
        'color',color,'linewidth',3);
    
end
end
function s = mySign(X)
s = abs(X)./X;
nanIdx = isnan(s);
% replace sign(0) with 1
s(nanIdx) = 1;
end

function result = checkIntersection(cube1,cube2)

I = [max(cube1(:,1),cube2(:,1)) min(cube1(:,2),cube2(:,2))];
diff = I(:,2) - I(:,1);
% diff = fix(diff*1e8)/1e8;
result = all(diff >= 0);
% result = all((I(:,2) - I(:,1))>=0);
end

%% TODO: This function is not generic! tailored for MPC!
function plot_violating_traj()
load('final_error_traj.mat')
global PLOT_TITLE;
global SAVE_FIGS;

fig1 = figure();
title([PLOT_TITLE ' - Error Trajectory']);

subplot(3,1,1)
plot(TT,XX(:,1))
xlabel('t');
ylabel('x1');

subplot(3,1,2)
plot(TT,XX(:,2))
xlabel('t');
ylabel('x2');

subplot(3,1,3)
hold on
plot(TT,XX(:,3))
% highlight error set: shade maybe
plot([0, 30],[-1.15, -1.15], 'r')
xlabel('t');
ylabel('x3');

if SAVE_FIGS == 1
    fig_file_name = fullfile('saved_figs', 'error_trace_projections.fig');
    savefig(fig1, fig_file_name, 'compact')
end

fig2 = figure();
title([PLOT_TITLE ' - Error Trajectory']);
plot3(XX(:, 1), XX(:, 2), XX(:, 3))
hold on
% highlight error set: shade maybe
% plot3([0 10], [-10 10], [-1.15, -1.15])

xlabel('x1');
ylabel('x2');
zlabel('x3');

if SAVE_FIGS == 1
    fig_file_name = fullfile('saved_figs', 'error_trace_3D.fig');
    savefig(fig2, fig_file_name, 'compact')
end

end

function y = init_cube()
y = [0 10
    0 10
    0 10];
end

% % 1. MPC example from Mike Huang in Michigan.
% %     - Region of interest: ball of radius 20
% %     - Safety : do not leave ball of radius 20
% %     - Init : Ball of radius 1
% % With these specs there should be no trajectory found, as we have a proof
% % that the system has a barrier.   TBD: figure out an init region so that
% % we can escape ball of radius 20, or figure out a smaller ball which can
% % be escaped from ball of radius 1.


function y = final_cube()
y = [-inf inf
    -inf inf
    -Inf  -1.15];
end