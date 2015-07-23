function isSat = checkConstraintSat(cons,X)

if cons.isNonLinear == 0
    % WHY IS Y a ROW VECTOR? and not column?
    % all() is used to realize conjunction
    %     disp(AX_b(Ab,Y'))
    
    cons.A = [cons.A;cons.A];
    cons.b = [cons.b;cons.b];
    
    numVectors = size(X,2);
    bMat = repmat(cons.b,1,numVectors);
    AX_b = cons.A*X-bMat;
    %     bMat
    %     cons.A
    %     X
    %     AX_b
    
    isSat = all(AX_b<=0,1);
    
    
elseif cons.isNonLinear == 1
    isSat = [];
    f_arr = cons.f_arr;
    for i = 1:length(f_arr)
        isSat = [isSat; f_arr{i}(X) <= 0];
    end
    isSat = all(isSat,1);
else
    error('checkConstraintSat: cons.isNonLinear is neither 0 or 1!')
end
end
