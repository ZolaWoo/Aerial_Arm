% MPC控制，直接针对系统的动力学方程，能控制位置和速度达到目标
% 轨迹规划将完整动作分解成匀速的小段，每一个小段用一次MPC使位置和速度达到当时的目标
% function 输入初值（上一阶段的控制结果）和目标值（读取轨迹规划值）
% function 输出控制量u，和控制后的实际值（不需要输出预测值）

% % 状态量初值,取飞行状态终值
% x_0 = x_track(end);
% z_0 = z_track(end);
% beta_0 = beta_track(end)*pi/180;
% betad_0 = 0;

% 初始状态量-如果不能在下一步回到约束范围内，则会造成无解
% x = [xb, zb, beta, xb_d, zb_d, beta_d]
% x0=[x_0; z_0; beta_0; 0; 0; 0];

% 参考输入
% xd=10*ones(1, 2*Np);
% xd=[x_track(end); z_track(end); pi/4; 0; 0; 0];
% xd=[5; 6; pi/4; 0; 0; 0];

% 控制量ut的初始值
% u0=zeros(Np,5); 

% Np预测步长,（控制步长为1）
% Np=8;

% xd是目标位置，xd和xd1是连续的两个目标点，用于求解目标速度
% 初值状态是x0点, 目标速度是目标位置时的斜率

% x = [xb, zb, theta, q1, q2, dxb, dzb, dtheta, dq1, dq2]

function [u,x_total,v_total] = my_mpc(u0,x0,xd,Np,opt_num)
% u是5列，x_total是5列,v_total是5列

%% 参数
e3 = [0;0;1]; 
g = 9.8;
L = 0.3; % 旋翼到中心的距离
L1 = 0.12; L2 = 0.12; % 连杆长度，m
mb = 1.58; mL1 = 0.1; mL2 = 0.1; % 单位kg，暂时忽略执行器和目标物的质量
ms = mb+mL1+mL2;
Ixx= 3*0.114;Iyy=3*0.114;Izz=3*0.158;
hL1 = 0.001; hL2 = 0.001;
Hb = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % m^2.kg
HL1 = [0 0 0;0 0 0;0 0 hL1]; HL2 = [0 0 0;0 0 0;0 0 hL2]; %机械臂杆的惯量


% 声明u来保存每一步采用的控制量
u=[];

% 控制量ut的上下限
lb = [];
ub = []; % 这里设总升力的上限为150N，舵机扭力上限为18kg（5V正常工作状态）
for i = 1:Np
    lb = [lb;0, -ms*g,-75*L, -1/2*mL1*g*L1-mL2*g*L1-1/2*mL2*g*L2, -1/2*mL2*g*L2];
    ub = [ub;150, 150-ms*g, 75*L, 20*9.8*0.01-1/2*mL1*g*L1-mL2*g*L1-1/2*mL2*g*L2, 18*9.8*0.01-1/2*mL2*g*L2];
end


% 误差反馈
xe=x0-xd;
x=xe; %用来计算和保存误差状态量
xxe=xe; % 用来保存误差值
xx=x0; % 用来显示最后的真实状态量

% 状态量的上下限
X_u = [];
X_l = [];
for i = 1:Np
% X_u = [X_u;1000;1000;pi/2;pi;pi;100;100;100;pi/0.48;pi/0.48];
% X_l = [X_l;-1000;-1000;-pi/2;0;-pi;-100;-100;-100;-pi/0.48;-pi/0.48];
X_u = [X_u;1000;1000;pi/2;pi;pi;100;100;100;pi/0.48;pi/0.48];
X_l = [X_l;-1000;-1000;-pi/2;-pi;-pi;-100;-100;-100;-pi/0.48;-pi/0.48];
end

for k=1:opt_num
    
    xb = xd(1) + xe(1); dxb = xd(6) + xe(6);
    zb = xd(2) + xe(2); dzb = xd(7) + xe(7);
    theta = xd(3) + xe(3); dtheta = xd(8) + xe(8);
    q1 = xd(4) + xe(4); dq1 = xd(9) + xe(9);
    q2 = xd(5) + xe(5); dq2 = xd(10) + xe(10);
    
    % 状态方程参数矩阵（跟随状态量更新）
B0 = zeros(5,5); C = zeros(5,5); G = zeros(5,1);
    % 计算B0
B0(1,1) = mL1 + mL2 + mb; B0(1,2) = 0;
B0(1,3) = (mL1*L1/2 + mL2*L1)*sin(q1-theta) + (mL2*L2/2)*sin(q1+q2-theta);
B0(1,4) = -(mL1*L1/2 + mL2*L1)*sin(q1-theta) - (mL2*L2/2)*sin(q1+q2-theta);
B0(1,5) = -(mL2*L2/2)*sin(q1+q2-theta);

B0(2,1) = 0; B0(2,2) = mL1 + mL2 + mb;
B0(2,3) = -(mL1*L1/2 + mL2*L1)*cos(q1-theta) - (mL2*L2/2)*cos(q1+q2-theta);
B0(2,4) = (mL1*L1/2 + mL2*L1)*cos(q1-theta) + (mL2*L2/2)*cos(q1+q2-theta);
B0(2,5) = (mL2*L2/2)*cos(q1+q2-theta);

B0(3,1) = (mL1*L1/2 + mL2*L1)*sin(q1-theta) + (mL2*L2/2)*sin(q1+q2-theta);
B0(3,2) = -(mL1*L1/2 + mL2*L1)*cos(q1-theta) - (mL2*L2/2)*cos(q1+q2-theta);
B0(3,3) = hL1 + hL2 + Iyy + mL1*(L1/2)^2 + mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)); 
B0(3,4) = -mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)) - mL1*(L1/2)^2;
B0(3,5) = -mL2*L2/2*(L2/2+L1*cos(q2));

B0(4,1) = -(mL1*L1/2 + mL2*L1)*sin(q1-theta) - (mL2*L2/2)*sin(q1+q2-theta);
B0(4,2) = (mL1*L1/2 + mL2*L1)*cos(q1-theta) + (mL2*L2/2)*cos(q1+q2-theta);
B0(4,3) = -mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)) - mL1*(L1/2)^2;
B0(4,4) = mL1*(L1/2)^2 + mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2));
B0(4,5) = mL2*L2/2*(L1*cos(q2) + L2/2);

B0(5,1) = -(mL2*L2/2)*sin(q1+q2-theta);
B0(5,2) = (mL2*L2/2)*cos(q1+q2-theta);
B0(5,3) = -mL2*L2/2*(L2/2+L1*cos(q2));
B0(5,4) = mL2*L2/2*(L1*cos(q2) + L2/2);
B0(5,5) = mL2*L2*L2/4;

B1 = inv(B0);

    % 计算C 
C(1,1) = 0; C(1,2) = 0;
C(1,3) =  dq1*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dtheta*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) + (L2*dq2*mL2*cos(q1 + q2 - theta))/2;
C(1,4) = dtheta*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dq1*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - (L2*dq2*mL2*cos(q1 + q2 - theta))/2;
C(1,5) = (L2*dtheta*mL2*cos(q1 + q2 - theta))/2 - (L2*dq2*mL2*cos(q1 + q2 - theta))/2 - (L2*dq1*mL2*cos(q1 + q2 - theta))/2;

C(2,1) = 0; C(2,2) = 0;
C(2,3) = dq1*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - dtheta*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) + (L2*dq2*mL2*sin(q1 + q2 - theta))/2;
C(2,4) = dtheta*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - dq1*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - (L2*dq2*mL2*sin(q1 + q2 - theta))/2;
C(2,5) = (L2*dtheta*mL2*sin(q1 + q2 - theta))/2 - (L2*dq2*mL2*sin(q1 + q2 - theta))/2 - (L2*dq1*mL2*sin(q1 + q2 - theta))/2;
 
C(3,1) = dq1*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dtheta*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) + (L2*dq2*mL2*cos(q1 + q2 - theta))/2;
C(3,2) = dq1*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - dtheta*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) + (L2*dq2*mL2*sin(q1 + q2 - theta))/2;
C(3,3) = - dxb*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dzb*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - (L1*L2*dq2*mL2*sin(q2))/2;
C(3,4) = dxb*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) + dzb*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) + (L1*L2*dq2*mL2*sin(q2))/2;
C(3,5) = (L2*dzb*mL2*sin(q1 + q2 - theta))/2 + (L2*dxb*mL2*cos(q1 + q2 - theta))/2 + (L1*L2*dq1*mL2*sin(q2))/2 + (L1*L2*dq2*mL2*sin(q2))/2 - (L1*L2*dtheta*mL2*sin(q2))/2;

C(4,1) = dtheta*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dq1*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - (L2*dq2*mL2*cos(q1 + q2 - theta))/2;
C(4,2) = dtheta*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - dq1*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - (L2*dq2*mL2*sin(q1 + q2 - theta))/2;
C(4,3) = dxb*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) + dzb*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) + (L1*L2*dq2*mL2*sin(q2))/2;
C(4,4) = - dxb*(cos(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*cos(q1 + q2 - theta))/2) - dzb*(sin(q1 - theta)*((L1*mL1)/2 + L1*mL2) + (L2*mL2*sin(q1 + q2 - theta))/2) - (L1*L2*dq2*mL2*sin(q2))/2;
C(4,5) = (L1*L2*dtheta*mL2*sin(q2))/2 - (L2*dxb*mL2*cos(q1 + q2 - theta))/2 - (L1*L2*dq1*mL2*sin(q2))/2 - (L1*L2*dq2*mL2*sin(q2))/2 - (L2*dzb*mL2*sin(q1 + q2 - theta))/2;

C(5,1) = (L2*dtheta*mL2*cos(q1 + q2 - theta))/2 - (L2*dq2*mL2*cos(q1 + q2 - theta))/2 - (L2*dq1*mL2*cos(q1 + q2 - theta))/2;
C(5,2) = (L2*dtheta*mL2*sin(q1 + q2 - theta))/2 - (L2*dq2*mL2*sin(q1 + q2 - theta))/2 - (L2*dq1*mL2*sin(q1 + q2 - theta))/2;
C(5,3) = (L2*dzb*mL2*sin(q1 + q2 - theta))/2 + (L2*dxb*mL2*cos(q1 + q2 - theta))/2 + (L1*L2*dq1*mL2*sin(q2))/2 + (L1*L2*dq2*mL2*sin(q2))/2 - (L1*L2*dtheta*mL2*sin(q2))/2;
C(5,4) = (L1*L2*dtheta*mL2*sin(q2))/2 - (L2*dxb*mL2*cos(q1 + q2 - theta))/2 - (L1*L2*dq1*mL2*sin(q2))/2 - (L1*L2*dq2*mL2*sin(q2))/2 - (L2*dzb*mL2*sin(q1 + q2 - theta))/2;
C(5,5) = (L1*L2*dtheta*mL2*sin(q2))/2 - (L2*dxb*mL2*cos(q1 + q2 - theta))/2 - (L1*L2*dq1*mL2*sin(q2))/2 - (L2*dzb*mL2*sin(q1 + q2 - theta))/2;
    
    % 计算G
G(1,1) = 0;
G(2,1) = (mL1 + mL2 + mb)*g;
G(3,1) = - g*mL1*(L1/2*cos(q1-theta)) - g*mL2*(L1*cos(q1-theta)+L2/2*cos(q1+q2-theta));
G(4,1) = g*mL1*(L1/2*cos(q1-theta)) + g*mL2*(L1*cos(q1-theta)+L2/2*cos(q1+q2-theta));
G(5,1) = g*mL2*L2/2*cos(q1 + q2-theta);

    % 线性系统系数矩阵，转换成MPC标准形式的状态方程
    A=[zeros(5,5), eye(5);zeros(5,5),-B1*C]; B=[zeros(5,5);B1];
    
    % 优化目标参数，加权矩阵
    Q=eye(10); R=eye(5);
    % 转化为用控制量ut表示的，关于状态量的推导方程的矩阵
    At=[]; Bt=[]; temp=[];
    % 转换后的加权矩阵
    Qt=[]; Rt=[];
    
    % 加权矩阵的计算过程，以及推导方程矩阵的叠加过程（通用的）
    for i=1:Np
        At=[At; A^i];
        Bt=[Bt zeros(size(Bt,1), size(B,2));
            A^(i-1)*B temp];
        temp=[A^(i-1)*B temp];
        
        Qt=[Qt zeros(size(Qt,1),size(Q,1));
            zeros(size(Q,1),size(Qt,1)) Q];
        Rt=[Rt zeros(size(Rt,1),size(R,1));
            zeros(size(R,1),size(Rt,1)) R];
    end

    % 转换后的优化目标函数矩阵，循环优化函数中H后的表达式为优化目标的另一项
    H=2*(Bt'*Qt*Bt + Rt);
    
    % 转换后的优化中的不等式约束左边系数矩阵，后面循环中的bi为不等式右边 （要约束舵机MG996R转角 转速8.0554 转矩）
        Ai=[Bt; -Bt];
    % 关于ut的不等式约束，实际上约束的是状态量，常数4就是状态量约束的上下边界
         bi=[X_u-At*(xe+xd); -X_l+At*(xe+xd)]; 
    % 一切准备就绪，进行二次优化,xe取代当前状态量xk
    % [ut, fval, exitflag]=quadprog(H,(2*xe'*At'*Qt*Bt)',[],[],[],[],lb,ub,u0);
     [ut, fval, exitflag]=quadprog(H,(2*xe'*At'*Qt*Bt)',Ai,bi,[],[],lb,ub,u0);
    % 显示求解结果是否正常
    fprintf('%d\n', exitflag)
%     xt = At*xk + Bt*ut;
%     
%     plot(xt');
%     hold on;
    xp=[]; %xp用来记录每个time step上对未来的预测状态量
    xp_0=xe; %第一个值为当前状态量
    xp=xp_0;
    
    % 整理一下ut的形式
    ut_tmp = [];
    for mu = 1:Np
        % disp('mu=');
        % disp(mu);
       ut_tmp = [ut_tmp;ut(5*mu-4), ut(5*mu-3), ut(5*mu-2), ut(5*mu-1), ut(5*mu)];
       % disp('tmp:');
       % disp(mu);
       % disp(k);
    end
    ut = ut_tmp;
    
    % 预测状态量
    for j=1:Np
        
        xp(:, j+1) = A*xp(:, j) + B*[ut(j,:)]';
        
    end
% 对每一次预测的状态量作图
    tp=k:(k+Np);
    tpu=k:(k+Np-1);

%     figure();
%     subplot(2, 1, 1);
%     plot(tp,xp','linewidth',1.5);
%     hold on;
%     subplot(2, 1, 2);
%     plot(tpu, ut, 'linewidth', 1.5);
%     hold on;
     
    % 采用优化得到的控制量的第一个元素作为实际作用的控制量，代入到原系统中得到下一个时刻的状态量
    u(k,:) = ut(1,:);
    x(:, k+1) = A*x(:, k) + B*(u(k,:))';
    xe = x(:, k+1);
    xxe=[xxe, xe];
    xx=[xx, xe+xd];  % 真实状态值
    % 对优化初始值进行修改，采用预测值的后段作为下一步的初始值
    u0 = [ut(2:Np,:); ut(Np,:)];
    
 end
% 计算x状态值的绝对值之和，作为衡量控制效果的指标
count=sum(abs(xxe'));

x_total = xx(1:5,:);
x_total = x_total';

v_total = xx(6:10,:);
v_total = v_total';

% figure(5);
% subplot(2,1,1);
% plot(x_total,'linewidth',1.5); grid on;
% legend('x','z','theta','q1','q2'); title('states');
% 
% subplot(2,1,2);
% plot(u,'linewidth',1.5); grid on;
% legend('u1','u2','u3','u4','u5'); title('inputs');
end
