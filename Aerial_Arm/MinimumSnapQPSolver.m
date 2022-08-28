function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];  %waypointsΪ path(:, 1)����һ���㣬���ȶ�x�������ʽϵ������waypoints(1)ֻ��ȡ��һ�����x������
    end_cond   = [waypoints(end), 0, 0, 0];
    
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq); %poly_coef Ϊn�ηֹ켣����ʽʽ�ӵ�ϵ������ϣ�����Ϊn_seg*8��ÿ��8��ϵ��
end