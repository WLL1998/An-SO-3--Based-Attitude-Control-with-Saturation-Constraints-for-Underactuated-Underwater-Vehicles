function dd = adaptivelaw(delta,para,z,epsi,J,mu,alpha)

judge = abs(z-delta);
j1 = judge(1); j2 = judge(2); j3 = judge(3);
temp1 = z'*J*[1;0;0]*alpha;
temp2 = z'*J*mu;
temp3 = 2*epsi*temp2;
temp4 = z-delta;


d1 = delta(1); d2 = delta(2); d3 = delta(3);
K1 = para(1,1); K2 = para(2,2); K3 = para(3,3);

if j1>epsi(1)
    dd1 = -K1*(d1)+(-temp1+temp2)/(temp4(1));
else
    dd1 =  0;
end


if j2>epsi(2)
    dd2 = -K2*(d2)+(-temp1+temp2)/(temp4(2));
else
    dd2 = 0;
end

if j3>epsi(3)
    dd3 = -K3*(d3)+(-temp1+temp2)/(temp4(3));
else
    dd3 =  0;
end

dd = [dd1;dd2;dd3];
end