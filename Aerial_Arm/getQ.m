function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for j = 1:n_seg  %һ����n_seg��
        Q_k = zeros(8,8); %minsnap������7��ϵ��Ϊ8

        % STEP 1.1: calculate Q_k of the k-th segment 
         for i=4:n_order
            for l=4:n_order
                L = factorial(l)/factorial(l-4);
                I = factorial(i)/factorial(i-4);             
                Q_k(i+1,l+1) = L*I/(i+l-7);
            end
         end
        Q = blkdiag(Q, Q_k);
    end
end
