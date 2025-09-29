function R = Rotationmatrix(x,y,z)
phi = x*pi/180; theta = y*pi/180; psi=z*pi/180;
R = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+sin(theta)*sin(phi)*cos(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta);
       sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
       -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
end