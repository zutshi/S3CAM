
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
