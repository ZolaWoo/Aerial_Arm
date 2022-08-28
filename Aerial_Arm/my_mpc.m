% MPC���ƣ�ֱ�����ϵͳ�Ķ���ѧ���̣��ܿ���λ�ú��ٶȴﵽĿ��
% �켣�滮�����������ֽ�����ٵ�С�Σ�ÿһ��С����һ��MPCʹλ�ú��ٶȴﵽ��ʱ��Ŀ��
% function �����ֵ����һ�׶εĿ��ƽ������Ŀ��ֵ����ȡ�켣�滮ֵ��
% function ���������u���Ϳ��ƺ��ʵ��ֵ������Ҫ���Ԥ��ֵ��

% % ״̬����ֵ,ȡ����״̬��ֵ
% x_0 = x_track(end);
% z_0 = z_track(end);
% beta_0 = beta_track(end)*pi/180;
% betad_0 = 0;

% ��ʼ״̬��-�����������һ���ص�Լ����Χ�ڣ��������޽�
% x = [xb, zb, beta, xb_d, zb_d, beta_d]
% x0=[x_0; z_0; beta_0; 0; 0; 0];

% �ο�����
% xd=10*ones(1, 2*Np);
% xd=[x_track(end); z_track(end); pi/4; 0; 0; 0];
% xd=[5; 6; pi/4; 0; 0; 0];

% ������ut�ĳ�ʼֵ
% u0=zeros(Np,5); 

% NpԤ�ⲽ��,�����Ʋ���Ϊ1��
% Np=8;

% xd��Ŀ��λ�ã�xd��xd1������������Ŀ��㣬�������Ŀ���ٶ�
% ��ֵ״̬��x0��, Ŀ���ٶ���Ŀ��λ��ʱ��б��

% x = [xb, zb, theta, q1, q2, dxb, dzb, dtheta, dq1, dq2]

function [u,x_total,v_total] = my_mpc(u0,x0,xd,Np,opt_num)
% u��5�У�x_total��5��,v_total��5��

%% ����
e3 = [0;0;1]; 
g = 9.8;
L = 0.3; % �������ĵľ���
L1 = 0.12; L2 = 0.12; % ���˳��ȣ�m
mb = 1.58; mL1 = 0.1; mL2 = 0.1; % ��λkg����ʱ����ִ������Ŀ���������
ms = mb+mL1+mL2;
Ixx= 3*0.114;Iyy=3*0.114;Izz=3*0.158;
hL1 = 0.001; hL2 = 0.001;
Hb = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % m^2.kg
HL1 = [0 0 0;0 0 0;0 0 hL1]; HL2 = [0 0 0;0 0 0;0 0 hL2]; %��е�۸˵Ĺ���


% ����u������ÿһ�����õĿ�����
u=[];

% ������ut��������
lb = [];
ub = []; % ������������������Ϊ150N�����Ť������Ϊ18kg��5V��������״̬��
for i = 1:Np
    lb = [lb;0, -ms*g,-75*L, -1/2*mL1*g*L1-mL2*g*L1-1/2*mL2*g*L2, -1/2*mL2*g*L2];
    ub = [ub;150, 150-ms*g, 75*L, 20*9.8*0.01-1/2*mL1*g*L1-mL2*g*L1-1/2*mL2*g*L2, 18*9.8*0.01-1/2*mL2*g*L2];
end


% ����
xe=x0-xd;
x=xe; %��������ͱ������״̬��
xxe=xe; % �����������ֵ
xx=x0; % ������ʾ������ʵ״̬��

% ״̬����������
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
    
    % ״̬���̲������󣨸���״̬�����£�
B0 = zeros(5,5); C = zeros(5,5); G = zeros(5,1);
    % ����B0
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

    % ����C 
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
    
    % ����G
G(1,1) = 0;
G(2,1) = (mL1 + mL2 + mb)*g;
G(3,1) = - g*mL1*(L1/2*cos(q1-theta)) - g*mL2*(L1*cos(q1-theta)+L2/2*cos(q1+q2-theta));
G(4,1) = g*mL1*(L1/2*cos(q1-theta)) + g*mL2*(L1*cos(q1-theta)+L2/2*cos(q1+q2-theta));
G(5,1) = g*mL2*L2/2*cos(q1 + q2-theta);

    % ����ϵͳϵ������ת����MPC��׼��ʽ��״̬����
    A=[zeros(5,5), eye(5);zeros(5,5),-B1*C]; B=[zeros(5,5);B1];
    
    % �Ż�Ŀ���������Ȩ����
    Q=eye(10); R=eye(5);
    % ת��Ϊ�ÿ�����ut��ʾ�ģ�����״̬�����Ƶ����̵ľ���
    At=[]; Bt=[]; temp=[];
    % ת����ļ�Ȩ����
    Qt=[]; Rt=[];
    
    % ��Ȩ����ļ�����̣��Լ��Ƶ����̾���ĵ��ӹ��̣�ͨ�õģ�
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

    % ת������Ż�Ŀ�꺯������ѭ���Ż�������H��ı��ʽΪ�Ż�Ŀ�����һ��
    H=2*(Bt'*Qt*Bt + Rt);
    
    % ת������Ż��еĲ���ʽԼ�����ϵ�����󣬺���ѭ���е�biΪ����ʽ�ұ� ��ҪԼ�����MG996Rת�� ת��8.0554 ת�أ�
        Ai=[Bt; -Bt];
    % ����ut�Ĳ���ʽԼ����ʵ����Լ������״̬��������4����״̬��Լ�������±߽�
         bi=[X_u-At*(xe+xd); -X_l+At*(xe+xd)]; 
    % һ��׼�����������ж����Ż�,xeȡ����ǰ״̬��xk
    % [ut, fval, exitflag]=quadprog(H,(2*xe'*At'*Qt*Bt)',[],[],[],[],lb,ub,u0);
     [ut, fval, exitflag]=quadprog(H,(2*xe'*At'*Qt*Bt)',Ai,bi,[],[],lb,ub,u0);
    % ��ʾ������Ƿ�����
    fprintf('%d\n', exitflag)
%     xt = At*xk + Bt*ut;
%     
%     plot(xt');
%     hold on;
    xp=[]; %xp������¼ÿ��time step�϶�δ����Ԥ��״̬��
    xp_0=xe; %��һ��ֵΪ��ǰ״̬��
    xp=xp_0;
    
    % ����һ��ut����ʽ
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
    
    % Ԥ��״̬��
    for j=1:Np
        
        xp(:, j+1) = A*xp(:, j) + B*[ut(j,:)]';
        
    end
% ��ÿһ��Ԥ���״̬����ͼ
    tp=k:(k+Np);
    tpu=k:(k+Np-1);

%     figure();
%     subplot(2, 1, 1);
%     plot(tp,xp','linewidth',1.5);
%     hold on;
%     subplot(2, 1, 2);
%     plot(tpu, ut, 'linewidth', 1.5);
%     hold on;
     
    % �����Ż��õ��Ŀ������ĵ�һ��Ԫ����Ϊʵ�����õĿ����������뵽ԭϵͳ�еõ���һ��ʱ�̵�״̬��
    u(k,:) = ut(1,:);
    x(:, k+1) = A*x(:, k) + B*(u(k,:))';
    xe = x(:, k+1);
    xxe=[xxe, xe];
    xx=[xx, xe+xd];  % ��ʵ״ֵ̬
    % ���Ż���ʼֵ�����޸ģ�����Ԥ��ֵ�ĺ����Ϊ��һ���ĳ�ʼֵ
    u0 = [ut(2:Np,:); ut(Np,:)];
    
 end
% ����x״ֵ̬�ľ���ֵ֮�ͣ���Ϊ��������Ч����ָ��
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
