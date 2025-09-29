function dQ = Kinematic(Q,omega)
w1 = omega(1);
w2 = omega(2);
w3 = omega(3);
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);
T = 0.5*[-q1 -q2 -q3;
      q0 -q3 q2;
      q3 q0 -q1;
      -q2 q1 q0];
dQ = T*[w1;w2;w3];