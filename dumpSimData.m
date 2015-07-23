
function dumpSimData(trajsCellMatXi,trajsCellMatXf,timeCellMat,Xi,Xf,sys_def,sys_prop,sys_abs,sys_opt,selfAdj,scalingArr,FILEPATH)
NUM_MODES = sys_def.NUM_MODES;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
%FILEPATH = sys_def.FILEPATH;
transitionMap = sys_def.transitionMap;

len = length(timeCellMat);


fprintf('scaling...')
for i = 1:len
    r = length(timeCellMat{i});
    if r ~= 0
        trajsCellMatXi{i} = trajsCellMatXi{i}.*repmat(scalingArr,r,1);
        trajsCellMatXf{i} = trajsCellMatXf{i}.*repmat(scalingArr,r,1);
    end
end
fprintf('done\n')


opFileName = [FILEPATH '-info.dat'];
disp(['dumping info...' opFileName])
% 'W' is faster! no explicit flushing till fclose() is called
fid = fopen(opFileName,'W');
% % wbhWrite = waitbar(0,'writing file...');
fprintf(fid,'numModes: %d\n',len);
fprintf(fid,'numStateVars: %d\n',NUM_STATE_VARS);
fprintf(fid,'modeList: ');
for i = 1:NUM_MODES
    fprintf(fid,'mode%d ',i);
end
fprintf(fid,'\n');
fprintf(fid,'adjGraph:');
writeAdjModeGraph(fid,NUM_MODES,transitionMap,selfAdj);

fprintf(fid,'trajsCellMatXi\n');
for i = 1:len
    % %     waitbar(i/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(trajsCellMatXi{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end

fprintf(fid,'trajsCellMatXf\n');
for i = 1:len
    % %     waitbar((1*len+i)/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(trajsCellMatXf{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end


fprintf(fid,'timeCellMat\n');
for i = 1:len
    % %     waitbar((2*len+i)/(3*len),wbhWrite);
    [hdr,matStr] = writeMatFast(timeCellMat{i},sprintf('mode%d', i));
    fprintf(fid,'%s',hdr);
    fprintf(fid,'%s\n',matStr');
end

fprintf(fid,'Xi\n');
writeMat(fid,Xi,'','int');

fprintf(fid,'Xf\n');
writeMat(fid,Xf,'','int');

fclose(fid);

% % close(wbhWrite)
end
