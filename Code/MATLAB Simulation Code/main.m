clc;
clear;
Q = [1;0;0;0];
omega = [0;0;0];
omega_d = [0;0;0];
Num = 2000;
h = 0.1;
time = zeros(1,Num);
%Rd = Rotationmatrix(0,35,0);
R = Rotationmatrix(-2,35,0);
vector = [1,0,0]';
x = ones(1,Num);
y = zeros(1,Num);
z = zeros(1,Num);
xd = ones(1,Num);
yd = zeros(1,Num);
zd = zeros(1,Num);
ada_para = zeros(3,Num);

e2 = [0;1;0]; e3 = [0;0;1];
delta = [1;1;1];
epsi = [0.5;0.5;0.5];
Ua = [0;0;0];
K1 = [1.1 0 0;0 1.2 0;0 0 1.1];
K2 = [0.01 0 0;0 0.02 0;0 0 0.01];

for t=1:1:Num
    time(t) = t*h;
    Rd = Rotationmatrix(0,10,0);
    omega_d = [0;0;0]*10^-2;
    domega_d = [0;0;0]*10^-2;
    omega(1) = 0.01*cos(0.1*time(t)*pi)*exp(-0.01*time(t));
    omega(3) = 0.01*cos(0.1*time(t)*pi)*exp(-0.01*time(t));
    Re = Rd'*R;
    theta_e = norm(acos(0.5*(trace(Re)-1)));
    Szeta = (theta_e/(2*sin(theta_e)))*(Re-Re');
    zeta_e = [Szeta(3,2);Szeta(1,3);Szeta(2,1)];
    e2 = [0;1;0]; e3 = [0;0;1];e1 = [1;0;0];
    mu = (omega(1)*e1+omega(3)*e3-Re'*omega_d);
    zv = zeta_e - delta;
    
    if (abs(norm(zeta_e))-pi)>0.001 && zeta_e'*omega >0
        omega = omega +0;
        zeta_e = zeta_e - (2*pi)*(zeta_e/norm(zeta_e));
        Ua = [0;0;0];
        k1 = zeros(3,3);
    else
        hz = Smatrix(zeta_e);
        tempS = 2*sin(norm(zeta_e)/2)/norm(zeta_e);
        temp1 = ((1-(cos(norm(zeta_e)/2)/tempS))/(norm(zeta_e)^2))*(hz'*hz);
        J = eye(3) + 0.5*hz + temp1;
 
        Ua = -K1*inv(J)*zv-K2*inv(J)*delta;
        omega(2) = Ua(2);
    end
    dd1 = adaptivelaw2(delta,K2,zv,epsi,J,mu,Ua(1),Ua(3));
    dd2 = adaptivelaw2(delta+0.5*h.*dd1,K2,zv,epsi,J,mu,Ua(1),Ua(3));
    dd3 = adaptivelaw2(delta+0.5*h.*dd2,K2,zv,epsi,J,mu,Ua(1),Ua(3));
    dd4 = adaptivelaw2(delta+h.*dd3,K2,zv,epsi,J,mu,Ua(1),Ua(3));
    delta = delta + (h/6).*(dd1+2.*dd2+2.*dd3+dd4);
    
%     if abs(delta(1))>5
%         delta(1) = sign(delta(1))*5;
%     end
%     if abs(delta(2))>5
%         delta(2) = sign(delta(2))*5;
%     end
%     if abs(delta(3))>5
%         delta(3) = sign(delta(3))*5;
%     end
    
    dQ1 = Kinematic(Q,omega);
    dQ2 = Kinematic(Q+0.5*h.*dQ1,omega);
    dQ3 = Kinematic(Q+0.5*h.*dQ2,omega);
    dQ4 = Kinematic(Q+h.*dQ3,omega);
    Q = Q+(h/6).*(dQ1+2.*dQ2+2.*dQ3+dQ4);
    q0 = Q(1); q1 = Q(2); q2 = Q(3); q3 = Q(4);
    R = [1-2*(q2*q2+q3*q3) 2*(q1*q2-q3*q0) 2*(q1*q3+q2*q0);
         2*(q1*q2+q3*q0) 1-2*(q1*q1+q3*q3) 2*(q2*q3-q1*q0);
         2*(q1*q3-q2*q0) 2*(q2*q3+q1*q0) 1-2*(q1*q1+q2*q2)];
    theta(t) = Ua(1);
    Q0(t) = q0;
    Q1(t) = q1;
    Q2(t) = q2;
    Q3(t) = q3;
    angle = QueToEul(q0,q1,q2,q3);
    A1(t) = angle(1);
    A2(t) = angle(2);
    A3(t) = angle(3);
    w1(t) = Ua(1);
    w2(t) = omega(2);
    w3(t) = Ua(3);
    SUM(t) = q0*q0+q1*q1+q2*q2+q3*q3;
    X = R*[2;0;0];
    Xd = Rd*[2;0;0];
    if t==1
        x(t) = X(1)*h+x(1);
        xd(t) = Xd(1)*h+xd(1);
        y(t) = X(2)*h+y(1);
        yd(t) = Xd(2)*h+yd(1);
        z(t) = X(3)*h+z(1);
        zd(t) = Xd(3)*h+zd(1);
    else
        x(t) = X(1)*h+x(t-1);
        xd(t) = Xd(1)*h+xd(t-1);
        y(t) = X(2)*h+y(t-1);
        yd(t) = Xd(2)*h+yd(t-1);
        z(t) = X(3)*h+z(t-1);
        zd(t) = Xd(3)*h+zd(t-1);
    end
    ada_para(:,t) = delta;
end
figure(1);
subplot(3,1,1);plot(time,A1*180/pi);grid on;
subplot(3,1,2);plot(time,A2*180/pi);grid on;
subplot(3,1,3);plot(time,A3*180/pi);grid on;
figure(2);
vector1 = R*vector;
vector2 = Rd*vector;
quiver3(0,0,0,1,0,0,'b');hold on;grid on;
quiver3(0,0,0,vector1(1),vector1(2),vector1(3),'g','Linewidth',2);
quiver3(0,0,0,vector2(1),vector2(2),vector2(3),'r');
axis equal;
figure(3);
plot(time,SUM);
figure(4);
plot3(x,y,z,'b-');grid on;hold on;
plot3(xd,yd,zd,'r:');axis equal;
figure(5);
plot(time,w1,'r');hold on;
plot(time,w2,'g');hold on;
plot(time,w3,'b');
figure(6);
plot(time,ada_para(1,:),'r');hold on;grid on;
plot(time,ada_para(2,:),'g');hold on;grid on;
plot(time,ada_para(3,:),'b');