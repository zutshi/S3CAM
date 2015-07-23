
function numViolations = simulateSystemFromInitArrBB(sys_def,sys_prop,sys_abs,sys_opt,initSet_cellMat)
numViolations = 0;
ONE = 1;
% There is only one mode
NUM_CINPUT_VARS = sys_def.NUM_CINPUT_VARS;
if NUM_CINPUT_VARS ~= 0
    Uh = sys_def.CINPUT_BOUNDS(:,2);
    Ul = sys_def.CINPUT_BOUNDS(:,1);
else
    Uh = 0;
    Ul = 0;
end

% modeDynStruct = sys_maps.modeDynMap(M0);
% SIM = modeDynStruct.F;
SIM = sys_def.SIM;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
delta = sys_abs.Delta;
initSet = initSet_cellMat{ONE};

%% simulate!
totalTime = sys_prop.TH;

cons_f = sys_prop.finalConstraints;
cons_i = sys_prop.initConstraints;
initMode = cons_i.mode;

% myFigure(1);hold on
% myFigure(2);hold on
% myFigure(3);hold on
% myFigure(4);hold on

% myFigure(5);hold on
% myFigure(6);hold on
%
% myFigure(7);hold on
% myFigure(8);hold on
% myFigure(9);hold on
% myFigure(10);hold on
% myFigure(11);hold on
% myFigure(12);hold on
% myFigure(13);hold on


% parfor
parfor i = 1:rows(initSet)
    propHit = 0;
    simTime = 0;
    X0 = initSet(i,1:NUM_STATE_VARS);
    currMode = initMode;
    cip = rand(NUM_CINPUT_VARS,1).*(Uh-Ul)+Ul;
    
    savedX0 = X0;
    savedCIP = cip;
    
    while propHit ~= 1 && simTime < totalTime
        
        if NUM_CINPUT_VARS == 0
            [t,X,~] = SIM([0 totalTime],X0,[],initMode);
%             display([num2str(i) ' - done'])
            % TBD: optimize below common parts, and remove check
%             t
            Xf = X(end,:);
            tf = t(end);
%             if tf ~= totalTime
%                 error('tf~=totalTime for BB, something wrong with SIM??')
%             end
        else
            [t,X,currMode] = SIM([0 delta],X0,cip,currMode);
            % TBD: optimize below common parts, and remove check
            Xf = X(end,:);
            tf = t(end);
            if tf ~= delta
                error('tf~=delta for BB, something wrong with SIM??')
            end
        end
        
        %         propHit = checkProp(t,X,cons_f);
        propHit = any(checkConstraintSat(cons_f,X'));
        %         propHit = 0;
        
        if propHit ~= 1
            cip = rand(NUM_CINPUT_VARS,1).*(Uh-Ul)+Ul;
            savedCIP = [savedCIP cip];
        else
            warning('prop-sat')
            disp('X0,i/p')
            disp(savedX0)
            disp(savedCIP)
            numViolations = numViolations + 1;
        end
        
        %         %                 uncomment to plot
%         sys_opt.plotFun(t+simTime,X,currMode);
        %                                 drawnow
        
        X0 = Xf;
        simTime = simTime + tf;
    end
    
end
end
