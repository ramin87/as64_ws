clc;
close all;
clear;

syms z er ep Iz ...
     dz der dep dIz ...
     Iz a k G ...
     s ...
     p11 p12 p13 p14 ...
     p21 p22 p23 p24 ...
     p31 p32 p33 p34 ...
     p41 p42 p43 p44 ...
     real

[A, b] = equationsToMatrix([  a*(-z - k*ep) == dz, ...
                              z - k*ep + k*Iz == der, ...
                              z + G*er - (k+G)*ep + k*Iz == dep, ...
                              z == dIz], ...
                             [z, er, ep, Iz]);

X = [z; er; ep; Iz];

A
b


% b == A*X

r = collect(det(s*eye(4)-A), s)

%P = eye(4);
%P = diag([p11, p22, p33, p44]);
P = [p11  p12  p13    0
       0  p22    0    0
       0    0  p33    0
       0    0    0  p44];

R = A*P + P*A';

dR1 = simplify(det(R(1,1)))

dR2 = simplify(det(R(1:2,1:2)))

dR3 = simplify(det(R(1:3,1:3)))

dR4 = simplify(det(R(1:4,1:4)))







