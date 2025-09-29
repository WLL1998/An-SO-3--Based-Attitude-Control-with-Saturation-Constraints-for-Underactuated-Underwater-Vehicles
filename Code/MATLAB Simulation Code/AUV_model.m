function [ds,T1] = AUV_model(state,T,d)
% Hull

u = state(1);
v = state(2);
w = state(3);
p = state(4); % roll
q = state(5);
r = state(6);
q0 = state(10);
q1 = state(11);
q2 = state(12);
q3 = state(13);

D = 1.91e-1; % m
L = 1.33; % m
g = 9.81;
rho = 1030;% kg/m^3
m = 300/9.81; % kg
W = 300+5; % N
B = 1*W; % N  %(1 + 1e-7, 1 + 1e-8)
Af = 2.85e-2;
Sw = 7.09e-1;
Ap = 2.26e-1;
V = 3.15e-2;

Ixx = 1.77e-1; Ixy = 0; Ixz = 0;
Iyy = 3.45; Iyx = Ixy; Iyz = 0;
Izz = 3.45; Izx = Ixz; Izy = Iyz;

xB = 0; yB = 0; zB = 0;
xG = 0; yG = 0; zG = 0.0002; %1.96e-2

MRB = [m, 0, 0, 0, m*zG, -m*yG;
        0, m, 0, -m*zG, 0 ,m*xG;
        0 ,0, m, m*yG, -m*xG, 0;
        0, -m*zG, m*yG, Ixx, 0, 0;
        m*zG, 0, -m*xG, 0, Iyy, 0;
        -m*yG, m*xG, 0, 0, 0, Izz];
var1  =1;
% X hydrodynamic coefficients
Xuu = -1.62;
Xud = (-9.30e-1);
Xvd = 0;
Xwd = 0;
Xpd = 0;
Xqd = 0;
Xrd = 0;
Xnn = 0.0001497;
Xvr = (+3.55e1); % Xvr = -Yvd
Xrr = (-1.93e0); % Xrr = -Yrd
Xqq = (-1.93); % Xqq = Zqd
Xwq = (-3.55e1); % Xwq = Zwd

Xud = var1*Xud; Xvd = var1*Xvd; Xwd = var1*Xwd; 
Xpd = var1*Xpd; Xqd = var1*Xqd; Xrd = var1*Xrd;

Xvr = var1*Xvr; Xwq = var1*Xwq; 
Xqq = var1*Xqq; Xrr = var1*Xrr;  

Yud = 0; % Added mass
Yvd = (-3.55e1); % added mass
Ywd = 0; % added mass
Ypd = 0; % added `mass
Yqd = 0; % added mass
Yrd = (1.93); % added mass
Yuv = (-2.86e1); %  body cross-lift coefficient
Yvv = -1.31e2; % body cross drag coefficient
Yrr = 6.32e-1; % body cross drag coefficient
Yur = (5.22);
Ywp = (3.55e1); % Ywp = -Zwd
Ypq = (1.93e0); % Ypq = -Zqd
Ydr = 9.64;

Zud = 0; 
Zvd = 0;
Zwd = (-3.55e1);
Zqd = (-1.93e0);
Zpd = 0;
Zrd = 0;
Zuq = (-5.22);
Zvp = (-3.55e1);
Zrp = (1.93e0);
Zww = -1.31e2;
Zqq = -6.32e-1;
Zuw = (-2.86e1);
Zds = -9.64;

Kpd = (-7.04e-2); 
Kud = 0;
Kvd = 0;
Kwd = 0;
Kqd = 0;
Krd = 0;
Kpp = -1.3e-1;
Knn = 2.2e-5; 

Mud = 0;
Mvd = 0;
Mwd = (-1.93);
Mpd = 0;
Mqd = (-4.88);
Mrd = 0;
Mww = 3.18;
Mqq = -1.88e2;
Muw = (2.40e1); % Muw = -(Zwd - Xud)
Muq = (-2); % Muq = -Zqd
Mvp = (-1.93); % Mvp = -Yrd
Mrp = (4.86); % Mrp = (Kpd - Nrd)
Mds = -6.15;

Nud = 0;
Nvd = (1.93);
Nwd = 0;
Npd = 0;
Nqd = 0;
Nrd = (-4.88);
Nvv = -3.18;
Nrr = -9.40e1;
Nuv = (-2.40e1); % Nuv = -(Xud - Yvd)
Nur = (-2); % Nur = Yrd
Nwp = (-1.93); % Nwp = Zqd
Npq = (-4.86); % Npq = -(Kpd - Mqd)
Ndr = -6.15; 

MA = -[Xud,Xvd,Xwd,Xpd,Xqd,Xrd;
       Yud,Yvd,Ywd,Ypd,Yqd,Yrd;
       Zud,Zvd,Zwd,Zpd,Zqd,Zrd;
       Kud,Kvd,Kwd,Kpd,Kqd,Krd;
       Mud,Mvd,Mwd,Mpd,Mqd,Mrd;
       Nud,Nvd,Nwd,Npd,Nqd,Nrd];
   
Mmat = MRB + MA;

GX = -(W - B)*2*(q0*q2-q1*q3);
GY = 2*(q0*q1+q2*q3)*(W - B);
GZ = -(-q0^2+q1^2+q2^2-q3^2)*(W-B);
GK = -(yG*W - yB*B)*(-q0^2+q1^2+q2^2-q3^2) - (zG*W - zB*B)*2*(q0*q1+q2*q3);
GM =  -(zG*W - zB*B)*2*(q0*q2-q1*q3) + (xG*W - xB*B)*(-q0^2+q1^2+q2^2-q3^2);
GN = (xG*W - xB*B)*2*(q0*q1+q2*q3) + (yG*W - yB*B)*2*(q0*q2-q1*q3);

Xext = GX + Xuu*u*abs(u) + Xwq*w*q + Xqq*q*q + Xvr*v*r + Xrr*r*r ;
Yext = GY + Yvv*v*abs(v) + Yrr*r*abs(r) + Yur*u*r + Ywp*w*p + Ypq*p*q + Yuv*u*v  ;
Zext = GZ + Zww*w*abs(w) + Zqq*q*abs(q) + Zuq*u*q + Zvp*v*p + Zrp*r*p + Zuw*u*w ;
Kext = GK + Kpp*p*abs(p) ;
Mext = GM + Mww*w*abs(w) + Mqq*q*abs(q) + Muq*u*q + Mvp*v*p + Mrp*r*p + Muw*u*w ;
Next = GN + Nvv*v*abs(v) + Nrr*r*abs(r) + Nur*u*r + Nwp*w*p + Npq*p*q + Nuv*u*v ;
Fext = [Xext;Yext;Zext;Kext;Mext;Next];

XB = m*(-v*r + w*q - xG*(q^2 + r^2) + yG*p*q + zG*(p*r));
YB = m*(-w*p + u*r - yG*(r^2 + p^2) + zG*(q*r) + xG*q*p);
ZB = m*(-u*q + v*p - zG*(p^2 + q^2) + xG*(r*p) + yG*r*q);
KB = (Izz - Iyy)*q*r + m*yG*(-u*q + v*p) - m*zG*(-w*p + u*r);
MB = (Ixx - Izz)*r*p + m*zG*(-v*r + w*q) - m*xG*(-u*q + v*p);
NB = (Iyy - Ixx)*p*q + m*xG*(-w*p + u*r) - m*yG*(-v*r + w*q);
FB = [XB;YB;ZB;KB;MB;NB];


ds1 = inv(Mmat)*(Fext-FB+T+d);
T1 = [q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);
      2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
      2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) q0^2-q1^2-q2^2+q3^2];
T2 = 0.5*[-q1 -q2 -q3;
      q0 -q3 q2;
      q3 q0 -q1;
      -q2 q1 q0];
J = [T1 zeros(3,3);zeros(4,3) T2];
ds2 = J*[u;v;w;p;q;r];
ds = [ds1;ds2];

end

