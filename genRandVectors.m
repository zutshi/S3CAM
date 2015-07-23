function randVectors = genRandVectors(randPoints,range)
numRPoints = rows(randPoints);
a = range(:,1)';
b = range(:,2)';
A = repmat(a,numRPoints,1);
B = repmat(b,numRPoints,1);
randVectors = randPoints.*(B-A)+A;

% % values whose ranges have 0 width
% zeroWidthIdx = range(:,1) - range(:,2) == 0;
% randVectors(zeroWidthIdx) = range();
end