
function P = scatterX(X,sys_def,numPoints,scatterRadius, constraints)
% disp('=====================')
% numPoints
% disp('=====================')
%numPoints = 5;
numRPoints = numPoints*100;
% hyper Cube Side
s = scatterRadius/2;
a = (X - s);
%b = (X + s);
range = 2*s;
% scatter by picking random points
%P = rand(numRPoints,sys_def.NUM_STATE_VARS).*(repmat(b,numRPoints,1)-repmat(a,numRPoints,1))+repmat(a,numRPoints,1);
R = rand(numRPoints,sys_def.NUM_STATE_VARS);
P = scale_cols(R,range);
P = bsxfun(@plus, P, a);

zeroWidthIdx = range == 0;
P(:,zeroWidthIdx) = repmat(a(zeroWidthIdx),numRPoints,1);

% P
% if system is not hybrid, ignore invariants (this might stil be usefull if there is a Region Of Interest)
% if sys_info.hybrid == 1

%send constraints directly to scatterX
%modeInv = sys_def.modeInvMap(newMode);
%isSat = checkConstraintSat(modeInv,P');


isSat = checkConstraintSat(constraints,P');

%      error('lala')
% return only the satisfactory points
% P = P(isSat,:);

%seems redundant
%     actualNumPoints = min(numPoints,length(find(isSat)));
%     if actualNumPoints < numPoints/2

actualNumPoints = length(find(isSat));

% % if actualNumPoints < numPoints/2
% %     %     warning('<actualNumPoints < numPoints>')
% %     %     fprintf(2,'actualNumPoints:%d < numPoints:%d\n',actualNumPoints,numPoints);
% %     %     warning('\<actualNumPoints < numPoints>')
% %     %     P
% %     %disp(newMode)
% %     %disp(X)
% %     %     disp(maxJmp)
% %     %     disp(a)
% %     %     disp(b)
% % end


% % if actualNumPoints == 0
% %     warning('actualNumPoints = 0!')
% % end

if actualNumPoints < numPoints
    P = P(isSat,:);
else
    P = P(isSat,:);
    P = P(1:numPoints,:);
end
% else
%     P = P(1:numPoints,:);
% end

%P = [P; X];


%fprintf('scattered %d points within a Linf of %f\n',size(P,1),maxJmp);
end
