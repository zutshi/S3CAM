
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
