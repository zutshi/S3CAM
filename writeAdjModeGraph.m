
function writeAdjModeGraph(fid,NUM_MODES,transitionMap,selfAdj)
% diagBelowLeft = 0;
% diagBelowRight = 0;
% diagAboveLeft = 0;
% diagAboveRight = 0;

%TBD: fix it properlty for non-hybrid systems
if NUM_MODES == 1
    fprintf(fid,'\nmode1: 1 mode1');
else
    for mode1 = 1:NUM_MODES
        adjModes = {};
        ctr = 0;
        fprintf(fid,'\nmode%d:',mode1);
        for mode2 = 1:NUM_MODES
            if ~isempty(transitionMap{mode1, mode2})
                ctr = ctr + 1;
                adjModes{ctr} = ['mode' num2str(mode2)];
            end
        end
        if selfAdj == 1
            adjModes{ctr+1} = ['mode' num2str(mode1)];
        end
        fprintf(fid,' %d',length(adjModes));
        for k = 1:length(adjModes)
            fprintf(fid,' %s',adjModes{k});
        end
    end
end
fprintf(fid,'\n');
end
