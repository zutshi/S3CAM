%% Helper Functions
function y = helper(NUM_STATE_VARS)
y.getConstraint = @getConstraint;
y.getConsMat = @getConsMat;
y.getResetMap = @getResetMap;
y.getTrans = @getTrans;
y.getLinExpr = @getLinExpr;
y.getDyn = @getDyn;
y.allStateSpace = @allStateSpace;

    function cons = getConstraint(P,q)
        cons.A = zeros(1,NUM_STATE_VARS);
        cons.A(P(:,1)) = P(:,2);
        cons.b = q;
    end

    function inv = getConsMat(consArr)
        inv.isNonLinear = 0;
        inv.A = reshape([consArr.A],NUM_STATE_VARS,length(consArr))';
        inv.b = [consArr.b]';
    end


% exprG = getLinExpr(G,[G 1; I 2],0);
% exprI = getLinExpr(I,[G 1; I 2],0);
    function assignExpr = getLinExpr(var,P,q)
        assignExpr.A = zeros(1,NUM_STATE_VARS);
        assignExpr.var = var;
        if ~isempty(P)
            assignExpr.A(P(:,1)) = P(:,2);
        end
        assignExpr.b = q;
    end

    function resetMap = getResetMap(exprArr)
        resetMap.isNonLinear = 0;
        resetMap.A = eye(NUM_STATE_VARS,NUM_STATE_VARS);
        resetMap.b = zeros(NUM_STATE_VARS,1);
        
        if isempty(exprArr)
            % do nothing
        else
            vars = [exprArr.var]';
            resetMap.A(vars,:) = reshape([exprArr.A],NUM_STATE_VARS,length(exprArr))';
            resetMap.b(vars) = [exprArr.b]';
        end
        % if nargin == 0
        %     re = eye(NUM_STATE_VARS,NUM_STATE_VARS);
        % elseif nargin == 1
        % else
        %     error('getResetExpr: unexpected number of args recieved')
        % end
    end

    function dyn = getDyn(exprArr)
        dyn.isNonLinear = 0;
        dyn.A = zeros(NUM_STATE_VARS,NUM_STATE_VARS);
        dyn.b = zeros(NUM_STATE_VARS,1);
        
        if isempty(exprArr)
            % do nothing
        else
            vars = [exprArr.var]';            
            dyn.A(vars,:) = reshape([exprArr.A],NUM_STATE_VARS,length(exprArr))';
            dyn.b(vars) = [exprArr.b]';
        end
    end

    function trans = getTrans(g,r)
        trans.guard = g;
        trans.reset = r;
    end

    function allStateSpaceCons = allStateSpace()
        consArr = [];
        for varIdx = 1:NUM_STATE_VARS
            % -inf <= var <= inf
            consArr = [consArr getConstraint([varIdx 1],inf) getConstraint([varIdx -1],inf)];            
        end
        allStateSpaceCons = getConsMat(consArr);
    end

end
