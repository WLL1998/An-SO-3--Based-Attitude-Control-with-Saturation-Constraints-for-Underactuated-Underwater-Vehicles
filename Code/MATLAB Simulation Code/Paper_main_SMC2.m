clc;
clear;
state = [0;0;0;0;0;0;0;0;-25;1;0;0;0];
a = 1;
Y = zeros(a,13); %状态变量
tau = [1;0;0;0;0;0];
dis = [0;0;0;0;0;0];
omega = [0;0;0];
omega_d = [0;0;0];
Num = 8000;
h = 0.1;
time = zeros(1,Num);
%Rd = Rotationmatrix(0,35,0);
R = eye(3);
vector = [1,0,0]';
x = ones(1,Num);
y = zeros(1,Num);
z = zeros(1,Num);
xd = ones(1,Num);
yd = zeros(1,Num);
zd = zeros(1,Num);
ada_para = zeros(3,Num);

e2 = [0;1;0]; e3 = [0;0;1];
delta = [0;0;0];
epsi = [0.1;0.1;0.1];
Ua = [0;0;0];
K1 = [0.001 0 0;0 0.001 0;0 0 0.001];
K2 = [0.45 0 0;0 0.35 0;0 0 0.45];


lUa1 = 0;
lUa2 = 0;
ldUa1 = 0;
ldUa2 = 0;
le1 = 0;
le2 = 0;
sum11 = 0;
sum12 = 0;
sum21 = 0;
sum22 = 0;

beta11 = 2.1; beta12 = 3.2;
beta21 = 2.1; beta22 = 3.2;

M = 3^3; 
Thetafq = 0.1*ones(1,M); %Surge_model_parameter 纵向速度模型估计参数
basfq = 0.1*ones(1,M);
Thetafr = 0.1*ones(1,M); %Surge_model_parameter 纵向速度模型估计参数
basfr = 0.1*ones(1,M);

desired_pitch = 65;
%desired_yaw = 45;
qd = zeros(4,Num);%EulToQue(0,desired_pitch,desired_yaw);
M = 0;
N = 0;
%Rd = Rotationmatrix(0,0,45);

lambda1 = 0;
lambda2 = 0;

K11 = 14; K12 = 15-K11;
K21 = 14; K22 = 15-K21;

Ad = zeros(3,Num);
delta_kine = zeros(3,Num);

Ideal_p = [0;0;-25];

for t=1:1:Num
    time(t) = t*h;
    
    %----Kinematic contrller-------
    omega_d = [0;0;1]*10^-2;
    domega_d = [0;0;0]*10^-2;
%     if time(t)<=200
%         des_pitch = 89.9; des_yaw = 0;
%         Rd = Rotationmatrix(0,89.9,0);
%         qd(:,t) = EulToQue(0,90,0);
%         Ad(1,t)=0; Ad(2,t) = 90; Ad(3,t) = 0;
%     elseif time(t)>200 && time(t)<=300
%         des_pitch = 0; des_yaw = 0;
%         Rd = Rotationmatrix(0,0,0);
%         qd(:,t) = EulToQue(0,0,0);
%         Ad(1,t)=0; Ad(2,t) = 0; Ad(3,t) = 0;
%     else
%         des_pitch = -89.9; des_yaw = 0;
%         Rd = Rotationmatrix(0,-89,0);
%         qd(:,t) = EulToQue(0,-90,0);
%         Ad(1,t)=0; Ad(2,t) = -90; Ad(3,t) = 0;
%     end
    desired_yaw = controlHeading(0.01*time(t))*180/pi;
    Ad(1,t)=0; Ad(2,t) = 65; Ad(3,t) = desired_yaw;
    qd(:,t) = EulToQue(0,65,desired_yaw);
    Rd = Rotationmatrix(0,desired_pitch,desired_yaw);
    
    Re = Rd'*R;
    theta_e = norm(acos(0.5*(trace(Re)-1)));
    Szeta = (theta_e/(2*sin(theta_e)))*(Re-Re');
    zeta_e = [Szeta(3,2);Szeta(1,3);Szeta(2,1)];
    mu = (state(4)*[1;0;0]-Re'*omega_d);
    zv = zeta_e - delta;
    
    if (abs(norm(zeta_e))-pi)>0.01 && zeta_e'*omega >0
        omega = omega +0;
        zeta_e = zeta_e - (2*pi)*(zeta_e/norm(zeta_e));
        Ua = [0;0;0];
    else
        hz = Smatrix(zeta_e);
        tempS = 2*sin(norm(zeta_e)/2)/norm(zeta_e);
        temp1 = ((1-(cos(norm(zeta_e)/2)/tempS))/(norm(zeta_e)^2))*(hz'*hz);
        J = eye(3) + 0.5*hz + temp1;
        Ua = -K2*inv(J)*zv-K1*inv(J)*delta;
    end

    dd1 = adaptivelaw(delta,K1,zv,epsi,J,mu,Ua(1));
    dd2 = adaptivelaw(delta+0.5*h.*dd1,K1,zv,epsi,J,mu,Ua(1));
    dd3 = adaptivelaw(delta+0.5*h.*dd2,K1,zv,epsi,J,mu,Ua(1));
    dd4 = adaptivelaw(delta+h.*dd3,K1,zv,epsi,J,mu,Ua(1));
    delta = delta + (h/6).*(dd1+2.*dd2+2.*dd3+dd4);
    delta_kine(:,t) = delta;
    
    %----Dynamic controller-------
    q = state(5);
    e1 = (q-Ua(2));
    de1 = (e1-le1)/h; %误差的一阶导数
    le1 = e1;
    sum11 = sum11 + (1+e1*e1)*atan(e1); % int((1+e^2)*atan(e))
    sum12 = sum12 + de1;% int(de)
    s1 = beta11 * sum11 + beta12 * sum12 +de1;%滑模面
    
    dUa1 = (Ua(2)-lUa1)/h; %期望运动控制器的一阶导数
    lUa1 = Ua(2);
    ddUa1 = (dUa1 - ldUa1)/h; %期望运动控制器的二阶导数
    ldUa1 = dUa1;
    
    D1 = s1*(8.33*beta11*(1+e1^2)*atan(e1)+8.33*beta12*de1-8.33*ddUa1);
    D2 = s1*((Thetafq*basfq'));
    dlambda1 = -0.01*lambda1+(K12*s1-D2-D1)/(sqrt(1+lambda1*lambda1));
    lambda1 = lambda1 + dlambda1*h;
     
    lambda1_dyn(:,t) = lambda1;
    
    tau1 = -(2*K11/pi)*atan(pi*s1/(2*0.1))-K12*lambda1/(sqrt(1+lambda1*lambda1));
    tau1 = (tau1);

    dt1 = Actuator(M,tau1);
    dt2 = Actuator(M+0.5*h.*dt1,tau1);
    dt3 = Actuator(M+0.5*h.*dt2,tau1);
    dt4 = Actuator(M+h.*dt3,tau1);
    
    M = M+(h/6).*(dt1+2.*dt2+2.*dt3+dt4);
    
    [d1,] = FLS(Thetafq,state,s1,1,1);
    [d2,] = FLS(Thetafq+0.5*h.*d1,state,s1,1,1);
    [d3,] = FLS(Thetafq+0.5*h.*d2,state,s1,1,1);
    [d4,basfq] = FLS(Thetafq+h.*d3,state,s1,1,1);
    Thetafq = Thetafq + (h/6).*(d1+2.*d2+2.*d3+d4);
    Thetafq1(t,:) = Thetafq;
    
    %---Yaw control---
    r = state(6);
    e2 = (r-Ua(3));
    de2 = (e2-le2)/h; %误差的一阶导数
    le2 = e2;
    sum21 = sum21 + (1+e2*e2)*atan(e2); % int((1+e^2)*atan(e))
    sum22 = sum22 + de2;% int(de)
    s2 = beta21 * sum21 + beta22 * sum22 +de2;%滑模面
    
    dUa2 = (Ua(3)-lUa2)/h; %期望运动控制器的一阶导数
    lUa2 = Ua(3);
    ddUa2 = (dUa2 - ldUa2)/h; %期望运动控制器的二阶导数
    ldUa2 = dUa2;
    
    D1 = s2*(8.33*beta21*(1+e2^2)*atan(e2)+8.33*beta22*de2-8.33*ddUa2);
    D2 = s2*((Thetafr*basfr'));
    dlambda2 = -0.01*lambda2+(K22*s2-D2-D1)/(sqrt(1+lambda2*lambda2));
    lambda2 = lambda2 + dlambda2*h;
    
    lambda2_dyn(:,t) = lambda2;

    tau2 = -(2*K21/pi)*atan(pi*s2/(2*0.1))-K22*lambda2/(sqrt(1+lambda2*lambda2));
    tau2 = (tau2);

    dt1 = Actuator(N,tau2);
    dt2 = Actuator(N+0.5*h.*dt1,tau2);
    dt3 = Actuator(N+0.5*h.*dt2,tau2);
    dt4 = Actuator(N+h.*dt3,tau2);
    
    N = N+(h/6).*(dt1+2.*dt2+2.*dt3+dt4);
    
    [d1,] = FLS(Thetafr,state,s2,2,1);
    [d2,] = FLS(Thetafr+0.5*h.*d1,state,s2,2,1);
    [d3,] = FLS(Thetafr+0.5*h.*d2,state,s2,2,1);
    [d4,basfr] = FLS(Thetafr+h.*d3,state,s2,2,1);
    Thetafr = Thetafr + (h/6).*(d1+2.*d2+2.*d3+d4);
    Thetafr1(t,:) = Thetafr;
    
    tau = [2;0;0;0;M;N]; 
    dis = [0;0.2*sin(0.2*2*pi*time(t));0;0;0.1*sin(0.01*2*pi*time(t));-0.2*cos(0.01*2*pi*time(t))];
    
    pTorque1(t) = M;
    pTorque2(t) = N;
    k1 = AUV_model(state,tau,dis);
    k2 = AUV_model(state+0.5*h.*k1,tau,dis);
    k3 = AUV_model(state+0.5*h.*k2,tau,dis);
    k4 = AUV_model(state+h.*k3,tau,dis);
    state = state+(h/6).*(k1+2.*k2+2.*k3+k4);
    des_yaw = controlHeading(0.01*time(t))*180/pi;
    dp = Ideal_trajectory(state,0,65,des_yaw);
    Ideal_p = Ideal_p + dp*h;
    Ideal_position(:,t) = Ideal_p;
    
    q0 = state(10); q1 = state(11); q2 = state(12); q3 = state(13);
    R = [q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);
      2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
      2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) q0^2-q1^2-q2^2+q3^2];
    ax(t) = Ua(1);
    angle = QueToEul(q0,q1,q2,q3);
    pA1(t) = angle(1)*180/pi;
    pA2(t) = angle(2)*180/pi;
    pA3(t) = angle(3)*180/pi;
    
    Y(t,1) = state(1); %u
    Y(t,2) = state(2); %v
    Y(t,3) = state(3);
    Y(t,4) = state(4);
    Y(t,5) = state(5); %q
    Y(t,6) = state(6); %r
    Y(t,7) = state(7); %x
    Y(t,8) = state(8); %y
    Y(t,9) = state(9); %z
    Y(t,10) = state(10); %fai
    Y(t,11) = state(11);%theta
    Y(t,12) = state(12); %psai
    Y(t,13) = state(13);
    ada_para(:,t) = delta;
end

U  = Y(:,1);
V  = Y(:,2);
W  = Y(:,3);
P  = Y(:,4);
Q  = Y(:,5);
R  = Y(:,6);
pXp2 = Y(:,7);
pYp2 = Y(:,8);
pZp2 = Y(:,9);
pQ0 = Y(:,10);
pQ1 = Y(:,11);
pQ2 = Y(:,12);
pQ3 = Y(:,13);

figure(1);
subplot(3,1,1);plot(time,U,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('纵向速度u[m/s]');grid on;hold on;
subplot(3,1,2);plot(time,V,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('纵向速度u[m/s]');grid on;hold on;
subplot(3,1,3);plot(time,W,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('垂向速度w[m/s]');grid on;hold on;


figure(2);
plot3(pXp2,pYp2,pZp2,'b','LineWidth',2);grid on;xlabel('X');ylabel('Y');zlabel('Z');axis equal;
hold on;
plot3(Ideal_position(1,:),Ideal_position(2,:),Ideal_position(3,:),'r:','LineWidth',2);

figure(3);
subplot(4,1,1);plot(time,pQ0,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('\eta0');grid on;hold on;
plot(time,qd(1,:),'r:','LineWidth',2);
subplot(4,1,2);plot(time,pQ1,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('q_1');grid on;hold on;
plot(time,qd(2,:),'r:','LineWidth',2);
subplot(4,1,3);plot(time,pQ2,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('q_2');grid on;hold on;
plot(time,qd(3,:),'r:','LineWidth',2);
subplot(4,1,4);plot(time,pQ3,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('q_3');grid on;hold on;
plot(time,qd(4,:),'r:','LineWidth',2);


figure(4);
subplot(3,1,1);plot(time,P,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('横滚角速度[rad/s]');grid on;hold on;
subplot(3,1,2);plot(time,Q,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('俯仰角速度[rad/s]');grid on;hold on;
subplot(3,1,3);plot(time,R,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('艏向角速度[rad/s]');grid on;hold on;

figure(5);
subplot(2,1,1);plot(time,pTorque1,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('俯仰力矩[N/m]');grid on;hold on;
subplot(2,1,2);plot(time,pTorque2,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('偏航力矩[N/m]');grid on;hold on;

figure(6);
subplot(3,1,1);plot(time,pA1,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('横滚角[DEG]');grid on;hold on;
subplot(3,1,2);plot(time,pA2,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('俯仰角[DEG]');grid on;hold on;
subplot(3,1,3);plot(time,pA3,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('艏向角[DEG]');grid on;hold on;

save('A1p.mat','pA1');
save('A2p.mat','pA2');
save('A3p.mat','pA3');

save('Xp2p.mat','pXp2');
save('Yp2p.mat','pYp2');
save('Zp2p.mat','pZp2');

save('Time.mat','time');
save('Torque1p.mat','pTorque1');
save('Torque2p.mat','pTorque2');

save('Ad.mat','Ad');
save('lambda1_dyn.mat','lambda1_dyn');
save('lambda2_dyn.mat','lambda2_dyn');
save('Thetafq1.mat','Thetafq1');
save('Thetafr1.mat','Thetafr1');
save('delta_kine.mat','delta_kine');

save('Ideal_position.mat','Ideal_position');