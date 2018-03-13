clc;
close all;
clear;


a = 950;
c = 0.03;

t = 0:0.0005:0.08;

x = 1 - 1./( 1 + exp(a*(c-t)) );

figure;
plot(t,x);