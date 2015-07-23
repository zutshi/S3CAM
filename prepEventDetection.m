% TODO: check common events...separate them and prioritize them in some
% order...lots of overlap between guards and invariants
function [eventToTransitionInvMap,eventSet,terminalSet,eventInfo] = prepEventDetection(sys_def,sys_prop)
eventToTransitionInvMap = {};
eventSet = {};
terminalSet = {};
eventID = 0;
eventInfo = {};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
transitionMap = sys_def.transitionMap;
MODE_LIST = sys_def.MODE_LIST;
modeInvMap = sys_def.modeInvMap;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for currMode = MODE_LIST
    %    fprintf('currmode = %d', currMode);
    P = [];
    q = [];
    F_arr = [];
    %transMap = modeToOutGoingTransitionLinGuardMap(currMode);
    % TODO: too inefficient, use mode2transition map
    eventToTransitionInvMap{currMode} = [];
    terminalArr = [];
    addPropDetect = 1;
    %% ADD Property detection
    if addPropDetect == 1
        errCons = sys_prop.finalConstraints;
        % error lies in the current mode, and hence add event detection for the error
        if currMode == errCons.mode || errCons.mode == -1
            if errCons.isNonLinear == 1
                F_arr = [F_arr;errCons.f_arr];
                numEvents = length(errCons.f_arr);
                error('prepEventDetection: NonLin error constraints unhandled:')
            elseif errCons.isNonLinear == 0
                P = [P;errCons.A];
                q = [q;errCons.b];
                numEvents = length(errCons.b);
            else
                error('prepEventDetection: error constraints neither lin nor nonLin!!')
            end
            % num of conjuncts, each conjunct equivalent to an event
            %numEvents = length(errCons.b) + length(errCons.f_arr);
            %TODO: change this to 0 or 1 later
            % new structure
            event = [];
            event.type = PROP_SAT;
            event.cons = errCons;
            eventID = eventID + 1;
            eventIDArr = eventID*ones(numEvents,1);
            eventInfo{eventID} = event;
            eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
            
            terminalArr = [terminalArr;zeros(numEvents,1)];
%             if numEvents > 1
%                 % do not terminate on event detection
%                 % instead do a post evaluation and decide if the transition was enabled or not
%                 terminalArr = [terminalArr;zeros(numEvents,1)];
%             else
%                 % because there is only one transition, terminate immediately
%                 terminalArr = [terminalArr;1];
%             end
            
        end
    end
    
    if sys_def.hybrid == 1
        %% ADD Transitions
        for toMode = MODE_LIST
            % inf implies that get all transitions
            %TODO: handle all transitions and not just 1!
            allTransitions = createTransitionID(currMode,toMode,inf);
            transitionCell = getTransition(allTransitions,transitionMap);
            for k = 1:length(transitionCell)
                transition = transitionCell{k};
                %transition = transition{1};
                guard = transition.guard;
                % else there is only one transition at idx 1
                if guard.isNonLinear == 0
                    P = [P;guard.A];
                    q = [q;guard.b];
                    numEvents = length(guard.b);
                elseif guard.isNonLinear == 1
                    F_arr = [F_arr;guard.f_arr];
                    numEvents = length(guard.f_arr);
                else
                    % TODO: print the mode name
                    error('simulate: guard.isNonLinear is neither 0 or 1!')
                end
                % num of conjuncts, each conjunct equivalent to an event
                %numEvents = length(guard.b) + length(guard.f_arr);
                %TODO: change this to 0 or 1 later
                % new structure
                event = [];
                event.type = TRANSITION;
                event.transitionID = createTransitionID(currMode,toMode,k);
                eventID = eventID + 1;
                eventIDArr = eventID*ones(numEvents,1);
                eventInfo{eventID} = event;
                eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
                
                terminalArr = [terminalArr;zeros(numEvents,1)];
%                 if numEvents > 1
%                     % do not terminate on event detection
%                     % instead do a post evaluation and decide if the transition was enabled or not
%                     terminalArr = [terminalArr;zeros(numEvents,1)];
%                 else
%                     terminalArr = [terminalArr;zeros(numEvents,1)];
%                     % disable special case handling because non-det still needs to be considered...use it when absolutely
%                     % sure that non-det is not there and the transition is forcing....
%                     
%                     %                 disp(numEvents)
%                     %                 error('transition:numEvents <= 1')
%                     %                 % because there is only one transition, terminate immediately
%                     %                 % TODO: this means non determinism is yet not handled!
%                     %                 terminalArr = [terminalArr;1];
%                 end
            end
        end
        
        %% ADD Mode Invariants
        % TODO: enable with checks, if modeinv == guard
        modeInv = modeInvMap(currMode);
        if modeInv.isNonLinear == 0
            % negate the invariants because we want to detect when they are not
            % satisfied
            P = [P;-modeInv.A];
            q = [q;-modeInv.b];
            numEvents = length(modeInv.b);
        elseif modeInv.isNonLinear == 1
            F_arr = [F_arr;modeInv.f_arr];
            numEvents = length(modeInv.f_arr);
        else
            % TODO: print the mode name
            error('simulate: guard.isNonLinear is neither 0 or 1!')
        end
        %numEvents = length(modeInv.b) + length(modeInv.f_arr);
        % new structure
        event = [];
        event.type = MODE_INVARIANT;
        % should it be uniform?
        event.mode = currMode;
        eventID = eventID + 1;
        eventIDArr = eventID*ones(numEvents,1);
        eventInfo{eventID} = event;
        eventToTransitionInvMap{currMode} = [eventToTransitionInvMap{currMode};eventIDArr];
        
        terminalArr = [terminalArr;zeros(numEvents,1)];
%         if numEvents > 1
%             % do not terminate on event detection
%             % instead do a post evaluation and decide if the transition was enabled or not
%             terminalArr = [terminalArr;zeros(numEvents,1)];
%         else
%             %error('modeInv:numEvents <= 1')
%             % because there is only one transition, terminate immediately
%             % TODO: this means non determinism is yet not handled!
%             terminalArr = [terminalArr;zeros(numEvents,1)];
%         end
    end
    
    %% DONE
    eventSet{currMode}.Ab.A = P;
    eventSet{currMode}.Ab.b = q;
    eventSet{currMode}.f_arr = F_arr;
    terminalSet{currMode} = terminalArr;
    
end
end

function transitionID = createTransitionID(fromMode,toMode,num)
transitionID.fromMode = fromMode;
transitionID.toMode = toMode;
transitionID.num = num;
end