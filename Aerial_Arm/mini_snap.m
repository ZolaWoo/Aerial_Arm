% clc;clear;close all;
% path = [1,1; 1,4; 2,3; 4,5]; % 路径关键点
% T = 25; % 总时间,单位秒
% tstep = 0.1; % 轨迹每一点的时间间隔，单位秒

% 比如，4个点，3段，3个7阶多项式，每个多项式8个未知数，因此系数总共24个
% 而mini-snap求的是坐标关于时间的轨迹规划，因此x,y坐标要分别规划

function  [traj, vel] = mini_snap(path,T,tstep)
n_order       = 7;% 多项式的阶次 mini_snap为7 ，mini_jerk为5
n_seg         = size(path,1)-1;  % n_seg 代表路径的段数
n_poly_perseg = (n_order+1); % 多项式方程的未知量个数
variable_num = size(path,2);

if variable_num == 2
  %% 时间分配
    % 时间分配一般有两种，一种是按路径长度分配时间，一种是按梯形运动方式分配时间
    % 此处根据两点之间的距离计算时间分配
    dist     = zeros(n_seg, 1);
    dist_sum = 0;
    t_sum    = 0;
    
    for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2); % 第i段的直线距离
    dist_sum = dist_sum+dist(i); % 直线距离总长度
    end
    
    for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;  %前面n-1段路程都是按距离/总距离来计算时间
    t_sum = t_sum+ts(i);
    end
    ts(n_seg) = T - t_sum;  %而最后一段是总的时间减去前面所有时间之和

    % 此处为图简便可直接将每段时间赋值1
    % ts=ones(5,1);
    
  %% 求解多项式系数
    %求解x轴和y轴的多项式系数
    poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
    poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);
    
  %% 绘制minimun snap路径
    % display the trajectory
    X_n = []; Vx_n = [];
    Y_n = []; Vy_n = [];
    k_seg = 1; % 在规划好的平滑轨迹上重新取点，k_seg*tstep就是整段动作需要的总时间
    
    for i=0:n_seg-1
    
    % STEP 3: get the coefficients of i-th segment of both x-axis and y-axis
    % 第i段的x,y轴系数
    start_idx = n_poly_perseg * i;
    Pxi = poly_coef_x(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pxi = flipud(Pxi); % 翻转得到多项式系数
    Vxi = polyder(Pxi); % 速度多项式系数
    Pyi = poly_coef_y(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pyi = flipud(Pyi); % 多项式从阶数高的开头
    Vyi = polyder(Pyi); % 速度多项式系数
    
        for t = 0:tstep:ts(i+1)  %以tstep的步长遍历所有时间   （ts(i+1)就是时间结尾）
            X_n(k_seg)  = polyval(Pxi, t); %计算多项式Pxi在t点处的每一个值
            Vx_n(k_seg)  = polyval(Vxi, t);
            Y_n(k_seg)  = polyval(Pyi, t);
            Vy_n(k_seg)  = polyval(Vyi, t);
            k_seg = k_seg + 1;
        end
    end
 
    figure();
    plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);  %画出所有轨迹
    grid on
    hold on
    scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2)); 

    title('抓取轨迹规划','FontSize',12)
    xlabel('$q1/rad$','interpreter','latex','FontName','Times New Roman','FontSize',12);
    ylabel('$q2/rad$','interpreter','latex','FontName','Times New Roman','FontSize',12);
    
    traj = [X_n',Y_n']; %2列
    vel = [Vx_n',Vy_n'];

elseif variable_num == 3
   %% 时间分配
    % 时间分配一般有两种，一种是按路径长度分配时间，一种是按梯形运动方式分配时间
    % 此处根据两点之间的距离计算时间分配
    dist     = zeros(n_seg, 1);
    dist_sum = 0;
    t_sum    = 0;
    
    for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2 + (path(i+1, 3) - path(i, 3))^2); % 第i段的直线距离
    dist_sum = dist_sum+dist(i); % 直线距离总长度
    end
    
    for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;  %前面n-1段路程都是按距离/总距离来计算时间
    t_sum = t_sum+ts(i);
    end
    ts(n_seg) = T - t_sum;  %而最后一段是总的时间减去前面所有时间之和

    % 此处为图简便可直接将每段时间赋值1
    % ts=ones(5,1);
    
  %% 求解多项式系数
    %求解x轴和y轴的多项式系数
    poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
    poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);
    poly_coef_z = MinimumSnapQPSolver(path(:, 3), ts, n_seg, n_order);
    
  %% 绘制minimun snap路径
    % display the trajectory
    X_n = []; Vx_n = [];
    Y_n = []; Vy_n = [];
    Z_n = []; Vz_n = [];
    k_seg = 1; % 在规划好的平滑轨迹上重新取点，k_seg*tstep就是整段动作需要的总时间
    
    for i=0:n_seg-1
    
    % STEP 3: get the coefficients of i-th segment of both x-axis and y-axis
    % 第i段的x,y,z轴系数
    start_idx = n_poly_perseg * i;
    Pxi = poly_coef_x(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pxi = flipud(Pxi); % 翻转得到多项式系数
    Vxi = polyder(Pxi); % 速度多项式系数
    Pyi = poly_coef_y(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pyi = flipud(Pyi); % 多项式从阶数高的开头
    Vyi = polyder(Pyi); % 速度多项式系数
    Pzi = poly_coef_z(start_idx+1 : start_idx+n_poly_perseg , 1);
    Pzi = flipud(Pzi); % 多项式从阶数高的开头
    Vzi = polyder(Pzi); % 速度多项式系数
    
        for t = 0:tstep:ts(i+1)  %以tstep的步长遍历所有时间   （ts(i+1)就是时间结尾）
            X_n(k_seg)  = polyval(Pxi, t); %计算多项式Pxi在t点处的每一个值
            Vx_n(k_seg)  = polyval(Vxi, t);
            Y_n(k_seg)  = polyval(Pyi, t);
            Vy_n(k_seg)  = polyval(Vyi, t);
            Z_n(k_seg)  = polyval(Pzi, t);
            Vz_n(k_seg)  = polyval(Vzi, t);
            k_seg = k_seg + 1;
        end
    end
 
    figure();
    
    plot3(X_n, Y_n ,Z_n, 'Color', [0 1.0 0], 'LineWidth', 2);  %画出所有轨迹
    hold on
    grid on
    scatter3(path(1:size(path, 1), 1), path(1:size(path, 1), 2), path(1:size(path, 1), 3)); 

    
    traj = [X_n',Y_n',Z_n']; %3列
    vel = [Vx_n',Vy_n',Vz_n'];
    
end

end
    





 




