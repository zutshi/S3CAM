
function transitionCell = getTransition(transitionID,transitionMap)
transitionCell = transitionMap{transitionID.fromMode,transitionID.toMode};
% if not return {}
if isempty(transitionCell)
    transitionCell = {};
    return
end
if length(transitionCell) > 1
    warning('more than one transition to the same mode. EXPERIMENTAL');
end

if transitionID.num == inf
    % give back all transitions as a cell matrix
else
    % not really a cell!
    transitionCell = transitionCell{transitionID.num};
end

end
