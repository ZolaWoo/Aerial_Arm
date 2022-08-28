function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);%δ֪������
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    for k= 0:3
        Aeq_start(k+1,k+1) = factorial(k);
        %��4���״ε��������ʼ��ʱ��tΪ0����ôλ�þ���p0���ٶȾ���p1,���ٶȾ���2p2������ʽϵ�����Ҷ��ι滮Ҫ���δ֪����
                                          %���ԣ��˴���ʼ��ÿ���״Σ�Ҫ���pǰ�ĳ�����ϵ��Ϊn�Ľ׳�
    end
    beq_start = start_cond'; 
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    start_idx_2 = (n_order + 1)*(n_seg - 1);
    for k=0 : 3
        for i=k : 7   %�ܹ�0-3 4���״Σ�k��7��˼�ǰ˸�ϵ����֮��ǰ�漸��ϵ������ʧ�ˣ�
             Aeq_end(k+1,start_idx_2 + 1 + i ) = factorial(i)/factorial(i-k)*ts(n_seg)^(i-k);
        end
    end
    beq_end = end_cond';
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for i=0:n_seg-2  %һ��n_seg-1���м��
        start_idx_2 = (n_order + 1)*(i+1);  %ǰ�漸���ж��ٸ�ϵ��
        Aeq_wp(i+1,start_idx_2+1) = 1;%
        beq_wp(i+1,1) = waypoints(i+2);%�Ҷ�Ϊ ���ηֹ켣��ĩ������꣨��֪����
    end
    % position continuity constrain between each 2
    % segments�����ӵ㴦ǰ�������ڴ˵��λ�á��ٶȡ����ٶȡ��Ӽ��ٶ����
    Aeq_con = zeros((n_seg-1)*4, n_all_poly);
    beq_con = zeros((n_seg-1)*4, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
     for k=0:3
        for j=0:n_seg-2   %n_seg-1���м��
            for i = k:7   %ѭ��˳����Ҫ��ֻ�Ǳ��� n-1���м�㣬��4���״�pvaj���Լ�ʹi>=7������±���8��
                start_idx_1 = (n_seg-1)*k;
                start_idx_2 = (n_order+1)*j;
                Aeq_con(start_idx_1 + j + 1,start_idx_2 + i+1)=...
                    factorial(i)/factorial(i-k)*ts(j+1)^(i-k);   %����ͬһ�����ӵ���ǰһ�εĶ���ʽ���̵�ĩβ��pvaj����k�׵�������ʱʱ����ǰһ����Ϊts
                if(i == k)   %��Ϊ��һ�ε���ʼ��ʱ��Ϊ0�������󵼺󣬳���i==kʱ���������ֶ�Ϊ0�����Բ�����
                    Aeq_con(start_idx_1+j+1,start_idx_2+(n_order+1)+i+1) = ...
                                                            -factorial(i);%����ͬһ�����ӵ��ں�һ�εĶ���ʽ�����еļ���,��ʱʱ���ں�һ����Ϊ0
                                                            
                end  %�����������0                                        % %��Ϊ�ں�һ�����ӵ�Ķ���ʽ�У�tΪ0������piǰ��ϵ��ֻ�н׳�
            end
        end
    end
    
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end
