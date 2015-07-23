
function [value, isterminal, direction] = event_fun(~,X,~,~,currMode,eventSet,terminalSet)
%TODO: assumes all the events are described by a linear or all by non linear function
% Its a bit too strong when watching for both mode invariant and guards
% if non linear guards

% if ~isempty(eventSet{currMode}.Ab.b) && ~isempty(eventSet{currMode}.f_arr)
%     error('mix of non-lin and lin guards is UNHANDLED! sorry!')
% end
%
% if isempty(eventSet{currMode}.Ab.b)
%     f_arr = eventSet{currMode}.f_arr;
%     X_rep_arr = repmat({X},size(f_arr));
%     value = cellfun(@feval,f_arr,X_rep_arr);
% else
%     value = eventSet{currMode}.Ab.A*X-eventSet{currMode}.Ab.b;
% end
% numEvents = length(value);
% isterminal = terminalSet{currMode};
% direction = -1*ones(numEvents,1);

value_nlin = [];
value_lin = [];

if ~isempty(eventSet{currMode}.f_arr)
    %if isfield(eventSet{currMode},'f_arr')
    f_arr = eventSet{currMode}.f_arr;
    X_rep_arr = repmat({X},size(f_arr));
    value_nlin = cellfun(@feval,f_arr,X_rep_arr);
end
if ~isempty(eventSet{currMode}.Ab.b)
    %if isfield(eventSet{currMode},'Ab')
    value_lin = eventSet{currMode}.Ab.A*X-eventSet{currMode}.Ab.b;
end

value = [value_lin; value_nlin];
numEvents = length(value);
isterminal = terminalSet{currMode};
direction = -1*ones(numEvents,1);
% disp('a')

end

