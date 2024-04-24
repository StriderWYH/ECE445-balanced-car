clear;
clc;

syms x xd xdd     T Tp    thetadd thetad theta    phidd phid phi    P N PM NM    L LM;
%% 参数设定
% 均为标准单位制
g = 9.81;  

% 驱动轮
R = 0.06225;      %轮子半径
mw = 0.5;          %轮子质量
Iw = mw*R^2/2;   %轮子转动惯量

% 大腿
l_active_leg = 0.15;
m_active_leg = 0.20;
% 小腿
l_slave_leg = 0.27;
m_slave_leg = 0.305;
% 关节间距
joint_distance = 0.15;
% 摆杆
mp = (m_active_leg + m_slave_leg)*2;
Ip = mp*L^2/3;  %摆杆转动惯量

% 机体
M = 5.215;      %机体重量,应为整车的一半
l = -0.0145;    %机体质心到关节电机转轴的距离
IM = M*(0.2^2 + 0.1^2)/12;

%% 经典力学方程
fu1=N-NM==mp*(xdd+L*(thetadd*cos(theta)-thetad*thetad*sin(theta)));
fu2=P-PM-mp*g==mp*L*(-thetadd*sin(theta)-thetad*thetad*cos(theta));
fu3=NM==M*(xdd+(L+LM)*(thetadd*cos(theta)-thetad*thetad*sin(theta))-l*(phidd*cos(phi)-phid*phid*sin(phi)));
fu4=PM-M*g==M*((L+LM)*(-thetadd*sin(theta)-thetad*thetad*cos(theta))+l*(-phidd*sin(phi)-phid*phid*cos(phi)));

%% 不同部件之间的力求解
[N,NM,P,PM]=solve(fu1,fu2,fu3,fu4,N,NM,P,PM);
f1=xdd==(T-N*R)/(Iw/R+mw*R);
f2=Ip*thetadd==(P*L+PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
f3=IM*phidd==Tp+NM*l*cos(phi)+PM*l*sin(phi);
[xdd,thetadd,phidd]=solve(f1,f2,f3,xdd,thetadd,phidd);

%% 计算雅可比矩阵A and B
func=[thetad,thetadd,xd,xdd,phid,phidd];
A_lin_model=jacobian(func,[theta,thetad,x,xd,phi,phid]);
temp_A=subs(A_lin_model,[theta,thetad,xd,phi,phid],zeros(1,5));
final_A=simplify(temp_A);

B_lin_model=jacobian(func,[T Tp]);
temp_B=subs(B_lin_model,[theta,thetad,xd,phi,phid],zeros(1,5));
final_B=simplify(temp_B);

%% 计算不同腿长下LQR增益K
L_var = 0.112;   % 摆杆质心到驱动轮轴的距离，为腿长的一半

Q_mat = diag([1,1,5500,1000,6000,1]);
R_mat = diag([1,0.25]);
K = zeros(20,12);
leg_len = zeros(20,1);

for i=1:20
    L_var = L_var + 0.005;
    leg_len(i) = L_var*2;
    A = double(subs(final_A, [L LM], [L_var L_var]));
    B = double(subs(final_B, [L LM], [L_var L_var]));
    KK = lqrd(A, B, Q_mat, R_mat, 0.001);
    KK_t=KK.';
    K(i,:)=KK_t(:);
end

%% 不同腿长下二项式拟合K
K_cons=zeros(12,3);  
eul = [0 pi/2 0];
qZYX = eul2quat(eul);
for i=1:12
    res=fit(leg_len,K(:,i),"poly2");
    K_cons(i,:)=[res.p1, res.p2, res.p3];
end

for j=1:12
    for i=1:3
        fprintf("%f,",K_cons(j,i));
    end
    fprintf("\n");
end