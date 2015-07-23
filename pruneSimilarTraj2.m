
function uniqueSolMat = pruneSimilarTraj2(inputSolMat,sys_def)

disp('pruning similar trajs...')

MODE_IDX = sys_def.MODE_IDX;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
uniqueModeSeqIdx = {};
uniqueModeSeqBin = {};
idxCellMat = {};
n = length(inputSolMat);

for k = 1:n
    modeSeq = inputSolMat{k}(:,MODE_IDX);
    %% getModeSeqIdx(modeSeq);
    idx = 0;
    for i = 1:length(uniqueModeSeqIdx)
        if isequal(uniqueModeSeqIdx{i},modeSeq)
            idx = i;
            break;
        end
    end
    if idx == 0
        idx = length(uniqueModeSeqIdx) + 1;
        uniqueModeSeqIdx{idx} = modeSeq;
        uniqueModeSeqBin{idx} = [];
        idxCellMat{idx} = [];
    end
    %% end
    uniqueModeSeqBin{idx} = [uniqueModeSeqBin{idx}; inputSolMat{k}(1,1:NUM_STATE_VARS)];
    idxCellMat{idx} = [idxCellMat{idx}; k];
end

uniqueSolMat = {};
for i = 1:length(uniqueModeSeqBin)
    [~,uniqueIdx,~] = unique(uniqueModeSeqBin{i},'rows');
    idx = idxCellMat{i}(uniqueIdx);
    for j = idx'
        uniqueSolMat = [uniqueSolMat;inputSolMat(j)];
    end
end
end
