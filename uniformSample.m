
function stateSamples = uniformSample(range,sys_def)
NUM_STATE_VARS = sys_def.NUM_STATE_VARS;
delta = sys_opt.delta;

totalNumSamples = prod(floor((range(:,2)-range(:,1))/delta)+1);
stateSamples = zeros(totalNumSamples,NUM_STATE_VARS);

lastC = 1;
for k = NUM_STATE_VARS:-1:1
    x = range(k,1):delta:range(k,2);
    repSamp = [];
    for i = 1:length(x)
        repSamp = [repSamp; repmat(x(i),lastC,1)];
    end
    stateSamples(:,k) = repmat(repSamp,totalNumSamples/length(repSamp),1);
    lastC = length(repSamp);
end
%disp(stateSamples)
return
end
