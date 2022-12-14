function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);%未知量总数
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    for k= 0:3
        Aeq_start(k+1,k+1) = factorial(k);
        %求4个阶次的情况，起始点时间t为0，那么位置就是p0，速度就是p1,加速度就是2p2，多项式系数是我二次规划要求的未知数，
                                          %所以，此处起始点每个阶次，要求的p前的常数项系数为n的阶乘
    end
    beq_start = start_cond'; 
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    start_idx_2 = (n_order + 1)*(n_seg - 1);
    for k=0 : 3
        for i=k : 7   %总共0-3 4个阶次，k到7意思是八个系数求导之后，前面几个系数都消失了，
             Aeq_end(k+1,start_idx_2 + 1 + i ) = factorial(i)/factorial(i-k)*ts(n_seg)^(i-k);
        end
    end
    beq_end = end_cond';
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for i=0:n_seg-2  %一共n_seg-1个中间点
        start_idx_2 = (n_order + 1)*(i+1);  %前面几段有多少个系数
        Aeq_wp(i+1,start_idx_2+1) = 1;%
        beq_wp(i+1,1) = waypoints(i+2);%右端为 本段分轨迹的末点的坐标（已知量）
    end
    % position continuity constrain between each 2
    % segments，连接点处前后两段在此点的位置、速度、加速度、加加速度相等
    Aeq_con = zeros((n_seg-1)*4, n_all_poly);
    beq_con = zeros((n_seg-1)*4, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
     for k=0:3
        for j=0:n_seg-2   %n_seg-1个中间点
            for i = k:7   %循环顺序不重要，只是遍历 n-1个中间点，的4个阶次pvaj，以及使i>=7的情况下遍历8次
                start_idx_1 = (n_seg-1)*k;
                start_idx_2 = (n_order+1)*j;
                Aeq_con(start_idx_1 + j + 1,start_idx_2 + i+1)=...
                    factorial(i)/factorial(i-k)*ts(j+1)^(i-k);   %代表同一个连接点在前一段的多项式方程的末尾点pvaj（求k阶导），此时时间在前一段中为ts
                if(i == k)   %因为后一段的起始点时间为0，所以求导后，除了i==k时，其他部分都为0，所以不用算
                    Aeq_con(start_idx_1+j+1,start_idx_2+(n_order+1)+i+1) = ...
                                                            -factorial(i);%代表同一个连接点在后一段的多项式方程中的计算,此时时间在后一段中为0
                                                            
                end  %两者相减等于0                                        % %因为在后一个连接点的多项式中，t为0，所以pi前的系数只有阶乘
            end
        end
    end
    
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end
