function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];  %waypoints为 path(:, 1)即第一个点，而先对x轴求多项式系数，则waypoints(1)只提取第一个点的x轴坐标
    end_cond   = [waypoints(end), 0, 0, 0];
    
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq); %poly_coef 为n段分轨迹多项式式子的系数的组合，长度为n_seg*8，每段8个系数
end