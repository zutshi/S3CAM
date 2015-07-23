
% converts cubic/hyper-rectangle constraints on initial and final set to polyhedral constraints
% these are then stored in the respective structures
function [initConstraints,finalConstraints] = computePoly(initConstraints,finalConstraints)
if initConstraints.isNonLinear == 0
    if initConstraints.isCube == 1
        poly = cubeToPoly(initConstraints.cube);
        initConstraints.A = poly.A;
        initConstraints.b = poly.b;
    end
else
    error('computePoly: init constraints neither lin nor nonLin!!')
end
% repeat with final constraints
if finalConstraints.isNonLinear == 0
    if finalConstraints.isCube == 1
        poly = cubeToPoly(finalConstraints.cube);
        finalConstraints.A = poly.A;
        finalConstraints.b = poly.b;
    end
else
    error('computePoly: final constraints neither lin nor nonLin!!')
end
end
