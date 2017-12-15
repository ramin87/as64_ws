clc;
close all;
clear;


Q0 = [1 0 0 0]';

Q1 = [3 4 1 6];
Q1 = Q1/norm(Q1);


Q2 = [1 5 2 3];
Q2 = Q2/norm(Q2);


v12 = quatLog(quatProd(Q1,quatInv(Q2)));

v1 = quatLog(quatProd(Q0,quatInv(Q1)));
v2 = quatLog(quatProd(Q0,quatInv(Q2)));
v12_temp = v1-v2;

v12
v12_temp

