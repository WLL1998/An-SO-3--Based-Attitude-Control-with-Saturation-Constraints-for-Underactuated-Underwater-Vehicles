function q = EulToQue(roll,pitch,yaw)

roll = roll*pi/180;
pitch = pitch*pi/180;
yaw = yaw*pi/180;

q0 = cos(pitch/2)*cos(roll/2)*cos(yaw/2)-sin(pitch/2)*sin(roll/2)*sin(yaw/2);
q2 = sin(pitch/2)*cos(roll/2)*cos(yaw/2)-cos(pitch/2)*sin(roll/2)*sin(yaw/2);
q1 = cos(pitch/2)*sin(roll/2)*cos(yaw/2)+sin(pitch/2)*cos(roll/2)*sin(yaw/2);
q3 = cos(pitch/2)*cos(roll/2)*sin(yaw/2)+sin(pitch/2)*sin(roll/2)*cos(yaw/2);

q = [q0,q1,q2,q3];

end