clc;
clear;
close all;

dt = 0.002;
t = 0:dt:20;

c = 10;
a = 1.5;

x = 1 ./ ( 1 + exp(-a*(c-t)) );


figure;
plot(t,x);

