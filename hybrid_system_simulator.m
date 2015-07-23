
function sim_fn = hybrid_system_simulator(sys_def,sys_prop,sys_opt)

if sys_def.NUM_PARAMS ~= 0
    error('parameter handlind disabled: code them in as regular state variables')
end

if sys_prop.finalConstraints.isCube == 1
    poly = cubeToPoly(sys_prop.finalConstraints.cube);
    sys_prop.finalConstraints.A = poly.A;
    sys_prop.finalConstraints.b = poly.b;
end

[eventToTransitionInvMap,eventSet,terminalSet,eventInfo] = prepEventDetection(sys_def,sys_prop);

modeDynMap = sys_def.modeDynMap;
transitionMap = sys_def.transitionMap;
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
options = odeset('Events',@event_fun,'Refine',4,'RelTol',1e-6);%,'MaxStep',1e-4);
NUM_CINPUT_VARS = sys_def.NUM_CINPUT_VARS;
MAX_SWITCHINGS = sys_opt.ScatterSim.MAX_SWITCHINGS;

% expects constant inputs!
    function [simtimeArr, XArr, MArr] = SIM(Tspan, XArr, InpArr, MArr)
        % Consts
        DONE = -1;
        NOT_DONE = 1;
        
        tArr = Tspan(:, 1);
        TArr = Tspan(:, 2);
        %% allocate
        numInitStates = rows(XArr);
        statusArr = NOT_DONE*ones(numInitStates,1);
        simtimeArr = tArr;
        numSwitchArr = zeros(numInitStates,1);
        
        while any(statusArr == NOT_DONE)
%             fprintf('simulating %d samples\n',numInitStates);
            % parfor here
            for i = 1:numInitStates
                if statusArr(i) == NOT_DONE
                    X0 = XArr(i,1:NUM_STATE_VARS);
                    currMode = MArr(i);
                    eventToTransitionInv = eventToTransitionInvMap{currMode};
                    modeDynStruct = modeDynMap(currMode);

                    
                    if NUM_CINPUT_VARS ~= 0
                        if modeDynStruct.isNonLinear == 0
                            modeDyn = @(~,X,~,U,~,~,~) modeDynStruct.A*X + modeDynStruct.b + modeDynStruct.B*U;
                        else
                            modeDyn = @(t,X,~,U,currMode,~,~) modeDynStruct.F(t,X,U,currMode);
                        end
                    else
                        if modeDynStruct.isNonLinear == 0
                            modeDyn = @(~,X,~,~,~,~,~) modeDynStruct.A*X + modeDynStruct.b;
                        else
                            modeDyn = @(t,X,~,~,currMode,~,~) modeDynStruct.F(t,X,[],currMode);
                        end
                    end

                    if NUM_CINPUT_VARS == 0
                        [t_,X_,te,ye,ie] = ode45(modeDyn,[simtimeArr(i) TArr(i)],X0,options,[],[],currMode,eventSet,terminalSet);
%                         plot(t_,X_(:,2))
                    else
                        cip = InpArr(i,1);
                        [t_,X_,te,ye,ie] = ode45(modeDyn,[simtimeArr(i) TArr(i)],X0,options,[],cip,currMode,eventSet,terminalSet);
                    end
                    [teout, Y, eventFound, event] = checkNreturnEvent(te,ye,ie,eventToTransitionInv,eventInfo,transitionMap,sys_opt);
                    if isempty(te) || ~eventFound
                        MArr(i) = currMode;
                        XArr(i,:) = X_(end,:);
                        simtimeArr(i) = t_(end);
                        statusArr(i) = DONE;
                        if simtimeArr(i) ~= TArr(i)
                            error('internal: not possible! floating point errors?')
                        end
                        
                    elseif event.type == TRANSITION
                        %             warning('TRANSITION')
                        MArr(i) = event.transitionID.toMode;
                        transition = getTransition(event.transitionID,transitionMap);
                        if transition.reset.isNonLinear == 1
                            XArr(i,:) = transition.reset.F(Y')';
                        elseif transition.reset.isNonLinear == 0
                            XArr(i,:) = (transition.reset.A*Y' + transition.reset.b)';
                        else
                            error('simulate: reset.isNonLinear is neither 0 or 1!')
                        end
                        simtimeArr(i) = teout;
                        numSwitchArr(i) = numSwitchArr(i) + 1;
                        statusArr(i) = NOT_DONE;
                    elseif event.type == MODE_INVARIANT
                        MArr(i) = currMode;
                        XArr(i,:) = Y;
                        simtimeArr(i) = teout;
                        
                        % add one more fake data point using the given time
                        % horizon
                        % This removes inconsistancies
                        simtimeArr(i) = TArr(i);
                        
                        statusArr(i) = DONE;
                        %             warning('invariant hit!')
                    elseif event.type == PROP_SAT
                        %                     statusArr(i) = -1;
                        warning('prop-sat!')
                        MArr(i) = currMode;
                        XArr(i,:) = Y;
                        simtimeArr(i) = teout;
%                         statusArr(i) = NOT_DONE;
                        
                        % another hack like in the invariant section
                        % to make it work!
                        simtimeArr(i) = TArr(i);
                        statusArr(i) = DONE;
                    else
                        error('simulateSystem: this code is unreachable!')
                    end
                    if numSwitchArr(i) >= MAX_SWITCHINGS
                        statusArr(i) = DONE;
                        simtimeArr(i) = TArr(i);
                    end
                end
            end
        end
    end
sim_fn = @SIM;
end
