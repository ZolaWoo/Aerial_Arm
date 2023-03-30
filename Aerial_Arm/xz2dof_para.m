e3 = [0;0;1]; 
g = 9.8;
L = 0.3; % 旋翼到中心的距离
L1 = 0.12; L2 = 0.12; % 连杆长度，m
mb = 1.8; mL1 = 0.08; mL2 = 0.08; % 单位kg，暂时忽略执行器和目标物的质量
ms = mb+mL1+mL2;
Ixx= 3*0.114;Iyy=3*0.114;Izz=3*0.158;
hL1 = 0.001; hL2 = 0.001;Hb = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % m^2.kg
HL1 = [0 0 0;0 0 0;0 0 hL1]; HL2 = [0 0 0;0 0 0;0 0 hL2]; %机械臂杆的惯量

% x阶跃（空载）
% kdx = 30; kpx = 40; kix = 0;
% kdz = 80; kpz = 80; kiz = 0;
% kdb = 40; kpb = 80; kib = 0;
% ktp = 80; ktd = 1000;


% x阶跃（空载）
%  kdx = 80; kpx = 100; kix = 0;
%  kdz = 80; kpz = 80; kiz = 0;
%  kdb = 40; kpb = 80; kib = 0;
%  ktp = 80; ktd = 1000;

% beta阶跃（空载）
% kdx = 5; kpx = 60; kix = 0.1;
% kdz = 50; kpz = 40; kiz = 0;
% kdb = 200; kpb = 200; kib = 0;
% ktd = 1000;ktp = 80; 

% % beta阶跃（空载）
% kdx = 10; kpx = 20; kix = 0;
% kdz = 50; kpz = 10; kiz = 0;
% kdq1 = 50; kpq1 = 20; kiq1 = 0;
% kdq2 = 50; kpq2 = 20; kiq2 = 0;
% ktd = 10; ktp = 1;

% %  q1阶跃
% kdx = 17; kpx = 180; kix = 10;
% kdz = 50; kpz = 50; kiz = 0;
% kdq1 = 300; kpq1 = 5000; kiq1 = 3000;
% kdq2 = 1000; kpq2 = 5000; kiq2 = 3000;
% ktd = 10; ktp = 1;

% q2阶跃（适用于q1,q2）
kdx = 17; kpx = 180; kix = 10;
kdz = 50; kpz = 50; kiz = 0;
kdq1 = 800; kpq1 = 1200; kiq1 = 500;
kdq2 = 300; kpq2 = 2000; kiq2 = 1000;
ktd = 10; ktp = 1;

% 初值
dxb = 0; dzb = 0; dtheta = 0; dq1 = 0; dq2 = 0;
q1 = pi/2; q2 = 0; theta = 0;


%% 计算B初值
B(1,1) = mL1 + mL2 + mb; B(1,2) = 0;
B(1,3) = (mL1*L1/2 + mL2*L1)*sin(q1-theta) + (mL2*L2/2)*sin(q1+q2-theta);
B(1,4) = -(mL1*L1/2 + mL2*L1)*sin(q1-theta) - (mL2*L2/2)*sin(q1+q2-theta);
B(1,5) = -(mL2*L2/2)*sin(q1+q2-theta);

B(2,1) = 0; B(2,2) = mL1 + mL2 + mb;
B(2,3) = -(mL1*L1/2 + mL2*L1)*cos(q1-theta) - (mL2*L2/2)*cos(q1+q2-theta);
B(2,4) = (mL1*L1/2 + mL2*L1)*cos(q1-theta) + (mL2*L2/2)*cos(q1+q2-theta);
B(2,5) = (mL2*L2/2)*cos(q1+q2-theta);

B(3,1) = (mL1*L1/2 + mL2*L1)*sin(q1-theta) + (mL2*L2/2)*sin(q1+q2-theta);
B(3,2) = -(mL1*L1/2 + mL2*L1)*cos(q1-theta) - (mL2*L2/2)*cos(q1+q2-theta);
B(3,3) = hL1 + hL2 + Iyy + mL1*(L1/2)^2 + mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)); 
B(3,4) = -mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)) - mL1*(L1/2)^2;
B(3,5) = -mL2*L2/2*(L2/2+L1*cos(q2));

B(4,1) = -(mL1*L1/2 + mL2*L1)*sin(q1-theta) - (mL2*L2/2)*sin(q1+q2-theta);
B(4,2) = (mL1*L1/2 + mL2*L1)*cos(q1-theta) + (mL2*L2/2)*cos(q1+q2-theta);
B(4,3) = -mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2)) - mL1*(L1/2)^2;
B(4,4) = mL1*(L1/2)^2 + mL2*(L1^2 + (L2/2)^2 + L1*L2*cos(q2));
B(4,5) = mL2*L2/2*(L1*cos(q2) + L2/2);

B(5,1) = -(mL2*L2/2)*sin(q1+q2-theta);
B(5,2) = (mL2*L2/2)*cos(q1+q2-theta);
B(5,3) = -mL2*L2/2*(L2/2+L1*cos(q2));
B(5,4) = mL2*L2/2*(L1*cos(q2) + L2/2);
B(5,5) = mL2*L2*L2/4;

B1 = inv(B);

%% 计算C初值
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

C1 = C;