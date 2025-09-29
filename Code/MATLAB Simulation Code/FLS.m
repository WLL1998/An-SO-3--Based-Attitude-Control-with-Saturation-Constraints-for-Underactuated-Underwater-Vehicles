function [dt,bas] = FLS(W,state,s,beta,xi)

x0 = state(4);%p
x1 = state(5);%q
x2 = state(6);%r

x0UMF = zeros(1,3);%-2~2
x0LMF = zeros(1,3);%-2~2
x1UMF = zeros(1,3);%-2~2
x1LMF = zeros(1,3);%-2~2
x2UMF = zeros(1,3);%-2~2
x2LMF = zeros(1,3);%-2~2


for i=1:1:3
     gls1 = -((x0-15+(i-1)*15)/(15/2)).^2;
     gls2 = -((x0-15+(i-1)*15)/((15/4)-0.25)).^2;
     x0UMF(i) = 1*exp(gls1);
     x0LMF(i) = exp(gls2);
end

for i=1:1:3
     gls1 = -((x1-15+(i-1)*15)/(15/2)).^2;
     gls2 = -((x1-15+(i-1)*15)/((15/4)-0.25)).^2;
     x1UMF(i) = 1*exp(gls1);
     x1LMF(i) = exp(gls2);
end

for i=1:1:3
     gls1 = -((x2-15+(i-1)*15)/(15/2)).^2;
     gls2 = -((x2-15+(i-1)*15)/((15/4)-0.25)).^2;
     x2UMF(i) = 1*exp(gls1);
     x2LMF(i) = 1*exp(gls2);
end

M = 3^3;

fly = zeros(1,M);
fuy = zeros(1,M);
k=1;
for a=1:1:3
    for b=1:1:3
        for c = 1:1:3
            fly(k) = x0LMF(a)*x1LMF(b)*x2LMF(c);
            fuy(k) = x0UMF(a)*x1UMF(b)*x2UMF(c);
            k=k+1;
        end
    end
end
flb = fly/(sum(fly));
fub = fuy/(sum(fuy));
bas = 0.5*(flb+fub);
dt = -2*beta*(W)+s*bas*xi;
end

