
function [teout, Y, eventFound, event] = checkNreturnEvent(te,ye,ie,eventToTransitionInvMap,getEvent,transitionMap,sys_opt)
teout = [];Y = [];event = [];
eventFound = 0;
%disp(ie')
for i = 1:length(te)
    % TODO: round these off to prevent numerical issues from cropping up...specially when the guards are equality
    %  round to two digits
    %  Only for detection purposes...but note down the true values..
    %  Bring this option as a parameter outside
    Y = ye(i,:);
    Yr = round(Y*(10^sys_opt.GuardDetectRoundDigits))/(10^sys_opt.GuardDetectRoundDigits);
    teout = te(i);
    ieout = ie(i);
    % get the guard whose conjunct might have triggererd the event
    %     disp('--------')
    %     disp(eventToGuardMap{currMode}')
    eventID = eventToTransitionInvMap(ieout);
    % % ignore event id and check all guards going out from the current mode
    % % to find the guard that triggered the event
    % guardID = find(cellfun(@satGuard,guardMap));
    
    % only one guard per mode, hence idx is hard coded to 1
    event = getEvent{eventID};
    if event.type == TRANSITION
        transition = getTransition(event.transitionID,transitionMap);
        %transition = transition{1};
        isSat = checkConstraintSat(transition.guard,Yr');
        %newMode = event.transitionID.toMode;
    elseif event.type == MODE_INVARIANT
        %modeInv = modeInvMap(event.mode);
        isSat = 1;
        %         % if modeinvariant is unsatisfied, then there is an event
        %         disp(modeInv)
        %         disp(Y')
        %         isSat = ~checkConstraintSat(modeInv,Y');
        %eventObj.mode =
    elseif event.type == PROP_SAT
        isSat = checkConstraintSat(event.cons,Yr');
    else
        error('checkNreturnEvent: event neither TRANSITION or MODE_INVARIANT!')
    end
    if isSat == 1
        eventFound = 1;
        %         % debug print
        %         if event.type == TRANSITION
        %             %fprintf('transition to new mode %d\n', newMode);
        %             %disp(transition.guard.A)
        %             %disp(transition.guard.b)
        %         elseif event.type == MODE_INVARIANT
        %             %disp('mode-inv')
        %         end
        break;
    end
end
end
