function dd = adaptivelaw2(delta,para,z,epsi,J,mu,alpha1,alpha2,alpha3)

judge = abs(z-delta);
j1 = judge(1); j2 = judge(2); j3 = judge(3);
temp1 = z'*J*[1;0;0]*alpha1+z'*J*[0;1;0]*alpha2+z'*J*[0;0;1]*alpha3;
temp2 = z'*J*mu;
temp4 = z-delta;


d1 = delta(1); d2 = delta(2); d3 = delta(3);
K1 = para(1,1); K2 = para(2,2); K3 = para(3,3);
% 
% if j1>epsi(1)
%     dd1 = -K1*abs(d1)+(-temp1+temp2)/sqrt((temp4(1)^2)+1);
% else
%     dd1 =  0;%-K1*abs(d1)+(sign(temp4(1))*(-temp1+temp3(1)))/(epsi(1)^2+j1^2);
% end
% 
% 
% if j2>epsi(2)
%     dd2 = -K2*abs(d2)+(-temp1+temp2)/sqrt((temp4(2)^2)+1);
% else
%     dd2 =  0;%-K2*abs(d2)+(sign(temp4(2))*(-temp1+temp3(2)))/(epsi(2)^2+j2^2);
% end
% 
% if j3>epsi(3)
%     dd3 = -K3*abs(d3)+(-temp1+temp2)/sqrt((temp4(3)^2)+1);
% else
%     dd3 = 0;% -K3*abs(d3)+(sign(temp4(3))*(-temp1+temp3(3)))/(epsi(3)^2+j3^2);
% end
dd1 = -K1*abs(d1)+(-temp1+temp2)/sqrt((temp4(1)^2)+1);
dd2 = -K2*abs(d2)+(-temp1+temp2)/sqrt((temp4(2)^2)+1);
dd3 = -K3*abs(d3)+(-temp1+temp2)/sqrt((temp4(3)^2)+1);

dd = [dd1;dd2;dd3];
end