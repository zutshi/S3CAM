
function [Xi_,Xf_] = getXiXf(trajMat_,sys_def,sys_prop,sys_abs,sys_opt)
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
MODE_IDX = sys_def.MODE_IDX;
initConstraints = sys_prop.initConstraints;
finalConstraints = sys_prop.finalConstraints;

disp('computing Xi Xf...')
cons_i = initConstraints;
cons_f = finalConstraints;
%         disp(cons_i)
Xi_ = find(checkConstraintSat(cons_i,trajMat_(:,1:NUM_STATE_VARS)'));
%         disp(Xi_)
%         disp(trajMat_(Xi_,MODE_IDX))
%disp('================================================')
%length(Xi_)
% don't check mode if its not applicable
% usefull when the hybrid state is defined over all modes
if cons_i.mode >= 0
    Xi_ = Xi_(find(trajMat_(Xi_,MODE_IDX) == cons_i.mode));
end
%length(Xi_)
Xf_ = find(checkConstraintSat(cons_f,trajMat_(:,NUM_STATE_VARS+1:2*NUM_STATE_VARS)'));
% don't check mode if its not applicable
% usefull when the hybrid state is defined over all modes
if cons_f.mode >= 0
    Xf_ = Xf_(find(trajMat_(Xf_,MODE_IDX) == cons_f.mode));
end
%disp('================================================')
Xi_ = Xi_';
Xf_ = Xf_';

end
