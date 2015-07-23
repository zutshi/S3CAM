
function [ret_t,ret_X] = simulateForSTaliroBB(sys_def,sys_prop,sys_abs,sys_opt,X0,M0,~)
X0 = X0'; % S-Taliro gives a column vector
ret_t = 0;
ret_X = X0;

totalTime = sys_prop.TH;

delta = sys_abs.Delta;

simTime = 0;

% modeDynStruct = sys_maps.modeDynMap(M0);
% SIM = modeDynStruct.F;
SIM = sys_def.SIM;

% myFigure(20)
% hold on

cons_i = sys_prop.initConstraints;
initMode = cons_i.mode;
currMode = initMode;

while simTime < totalTime
    
    [t,X,currMode] = SIM([0 delta],X0,[],currMode);
    %     [t,X] = SIM([0 delta],X0,[]);
    ret_t = [ret_t;t + simTime];
    ret_X = [ret_X;X];
    
    X0 = X(end,:);
    simTime = simTime + t(end);
    
end
%
% plot(ret_t,ret_X(:,3));
% plot(ret_X(:,1),ret_X(:,2));
% drawnow


% % plot(ret_t,ret_X(:,2));
% plot(ret_X(:,1),ret_X(:,2));

end

