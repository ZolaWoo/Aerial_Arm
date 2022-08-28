% clc;clear;close all;
% path = [1,1; 1,4; 2,3; 4,5]; % ·���ؼ���
% T = 25; % ��ʱ��,��λ��
% tstep = 0.1; % �켣ÿһ���ʱ��������λ��

% ���磬4���㣬3�Σ�3��7�׶���ʽ��ÿ������ʽ8��δ֪�������ϵ���ܹ�24��
% ��mini-snap������������ʱ��Ĺ켣�滮�����x,y����Ҫ�ֱ�滮

function  [traj, vel] = mini_snap(path,T,tstep)
n_order       = 7;% ����ʽ�Ľ״� mini_snapΪ7 ��mini_jerkΪ5
n_seg         = size(path,1)-1;  % n_seg ����·���Ķ���
n_poly_perseg = (n_order+1); % ����ʽ���̵�δ֪������
variable_num = size(path,2);

if variable_num == 2
  %% ʱ�����
    % ʱ�����һ�������֣�һ���ǰ�·�����ȷ���ʱ�䣬һ���ǰ������˶���ʽ����ʱ��
    % �˴���������֮��ľ������ʱ�����
    dist     = zeros(n_seg, 1);
    dist_sum = 0;
    t_sum    = 0;
    
    for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2); % ��i�ε�ֱ�߾���
    dist_sum = dist_sum+dist(i); % ֱ�߾����ܳ���
    end
    
    for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;  %ǰ��n-1��·�̶��ǰ�����/�ܾ���������ʱ��
    t_sum = t_sum+ts(i);
    end
    ts(n_seg) = T - t_sum;  %�����һ�����ܵ�ʱ���ȥǰ������ʱ��֮��

    % �˴�Ϊͼ����ֱ�ӽ�ÿ��ʱ�丳ֵ1
    % ts=ones(5,1);
    
  %% ������ʽϵ��
    %���x���y��Ķ���ʽϵ��
    poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
    poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);
    
  %% ����minimun snap·��
    % display the trajectory
    X_n = []; Vx_n = [];
    Y_n = []; Vy_n = [];
    k_seg = 1; % �ڹ滮�õ�ƽ���켣������ȡ�㣬k_seg*tstep�������ζ�����Ҫ����ʱ��
    
    for i=0:n_seg-1
    
    % STEP 3: get the coefficients of i-th segment of both x-axis and y-axis
    % ��i�ε�x,y��ϵ��
    start_idx = n_poly_perseg * i;
    Pxi = poly_coef_x(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pxi = flipud(Pxi); % ��ת�õ�����ʽϵ��
    Vxi = polyder(Pxi); % �ٶȶ���ʽϵ��
    Pyi = poly_coef_y(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pyi = flipud(Pyi); % ����ʽ�ӽ����ߵĿ�ͷ
    Vyi = polyder(Pyi); % �ٶȶ���ʽϵ��
    
        for t = 0:tstep:ts(i+1)  %��tstep�Ĳ�����������ʱ��   ��ts(i+1)����ʱ���β��
            X_n(k_seg)  = polyval(Pxi, t); %�������ʽPxi��t�㴦��ÿһ��ֵ
            Vx_n(k_seg)  = polyval(Vxi, t);
            Y_n(k_seg)  = polyval(Pyi, t);
            Vy_n(k_seg)  = polyval(Vyi, t);
            k_seg = k_seg + 1;
        end
    end
 
    figure();
    plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);  %�������й켣
    grid on
    hold on
    scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2)); 

    title('ץȡ�켣�滮','FontSize',12)
    xlabel('$q1/rad$','interpreter','latex','FontName','Times New Roman','FontSize',12);
    ylabel('$q2/rad$','interpreter','latex','FontName','Times New Roman','FontSize',12);
    
    traj = [X_n',Y_n']; %2��
    vel = [Vx_n',Vy_n'];

elseif variable_num == 3
   %% ʱ�����
    % ʱ�����һ�������֣�һ���ǰ�·�����ȷ���ʱ�䣬һ���ǰ������˶���ʽ����ʱ��
    % �˴���������֮��ľ������ʱ�����
    dist     = zeros(n_seg, 1);
    dist_sum = 0;
    t_sum    = 0;
    
    for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2 + (path(i+1, 3) - path(i, 3))^2); % ��i�ε�ֱ�߾���
    dist_sum = dist_sum+dist(i); % ֱ�߾����ܳ���
    end
    
    for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;  %ǰ��n-1��·�̶��ǰ�����/�ܾ���������ʱ��
    t_sum = t_sum+ts(i);
    end
    ts(n_seg) = T - t_sum;  %�����һ�����ܵ�ʱ���ȥǰ������ʱ��֮��

    % �˴�Ϊͼ����ֱ�ӽ�ÿ��ʱ�丳ֵ1
    % ts=ones(5,1);
    
  %% ������ʽϵ��
    %���x���y��Ķ���ʽϵ��
    poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
    poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);
    poly_coef_z = MinimumSnapQPSolver(path(:, 3), ts, n_seg, n_order);
    
  %% ����minimun snap·��
    % display the trajectory
    X_n = []; Vx_n = [];
    Y_n = []; Vy_n = [];
    Z_n = []; Vz_n = [];
    k_seg = 1; % �ڹ滮�õ�ƽ���켣������ȡ�㣬k_seg*tstep�������ζ�����Ҫ����ʱ��
    
    for i=0:n_seg-1
    
    % STEP 3: get the coefficients of i-th segment of both x-axis and y-axis
    % ��i�ε�x,y,z��ϵ��
    start_idx = n_poly_perseg * i;
    Pxi = poly_coef_x(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pxi = flipud(Pxi); % ��ת�õ�����ʽϵ��
    Vxi = polyder(Pxi); % �ٶȶ���ʽϵ��
    Pyi = poly_coef_y(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pyi = flipud(Pyi); % ����ʽ�ӽ����ߵĿ�ͷ
    Vyi = polyder(Pyi); % �ٶȶ���ʽϵ��
    Pzi = poly_coef_z(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pzi = flipud(Pzi); % ����ʽ�ӽ����ߵĿ�ͷ
    Vzi = polyder(Pzi); % �ٶȶ���ʽϵ��
    
        for t = 0:tstep:ts(i+1)  %��tstep�Ĳ�����������ʱ��   ��ts(i+1)����ʱ���β��
            X_n(k_seg)  = polyval(Pxi, t); %�������ʽPxi��t�㴦��ÿһ��ֵ
            Vx_n(k_seg)  = polyval(Vxi, t);
            Y_n(k_seg)  = polyval(Pyi, t);
            Vy_n(k_seg)  = polyval(Vyi, t);
            Z_n(k_seg)  = polyval(Pzi, t);
            Vz_n(k_seg)  = polyval(Vzi, t);
            k_seg = k_seg + 1;
        end
    end
 
    figure();
    
    plot3(X_n, Y_n ,Z_n, 'Color', [0 1.0 0], 'LineWidth', 2);  %�������й켣
    hold on
    grid on
    scatter3(path(1:size(path, 1), 1), path(1:size(path, 1), 2), path(1:size(path, 1), 3)); 

    
    traj = [X_n',Y_n',Z_n']; %3��
    vel = [Vx_n',Vy_n',Vz_n'];
    
end

end
    





 




