function S = Smatrix(vec)

w1 = vec(1);
w2 = vec(2);
w3 = vec(3);

S = [0 -w3 w2;
    w3 0 -w1;
    -w2 w1 0];
end