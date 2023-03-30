% grasp
e3 = [0;0;1]; 
g = 9.8;
L1=0.12;
L2=0.12;
hL1=0.001;
hL2=0.001;
Hb = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % m^2.kg
HL1 = [0 0 0;0 0 0;0 0 hL1]; HL2 = [0 0 0;0 0 0;0 0 hL2]; %机械臂杆的惯量

T = 10; % 总时间/秒
tstep = 0.1; % 轨迹规划步长
step_num = T/tstep; % 轨迹规划步数
arm_path=[pi/2 0; pi/4 pi/2; pi/3  2*pi/3];
[arm_traj,arm_vel] = mini_snap(arm_path,T,tstep);

kpx_g=180;
kix_g=10;
kdx_g=17;
kpz_g=50;
kiz_g=0;
kdz_g=50;
            
kpq1=1200;
kiq1=500;
kdq1=800;
            
kpq2=2000;
kiq2=1000;
kdq2=300;
            
ktd=10;
ktp=1;
            
Np=8;
opt_num=5;

            % 初值
            dxb = 0; dzb = 0; dtheta = 0; dq1 = 0; dq2 = 0;
            q1 = pi/2; q2 = 0; theta = 0;

            B = zeros(5,5); C = zeros(5,5);
            
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
