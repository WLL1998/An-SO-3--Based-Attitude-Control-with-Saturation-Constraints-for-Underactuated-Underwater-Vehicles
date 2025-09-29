function dp = Ideal_trajectory(state,phi,theta,psi)

phi = phi*pi/180;
theta = theta*pi/180;
psi = psi*pi/180;

u = state(1);
v = state(2);
w = state(3);

R = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+sin(theta)*sin(phi)*cos(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta);
 sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
 -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

dp = R*[u;v;w];

end