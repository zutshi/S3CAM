function s = mySign(X)
s = abs(X)./X;
nanIdx = isnan(s);
% replace sign(0) with 1
s(nanIdx) = 1;
end