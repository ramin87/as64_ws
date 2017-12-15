clc;
close all
clear;


x0 = 2;

x_end = (x0-1)+0.99;

t_end = 1;
tau = t_end;
a = 150;
c = tau - (tau/a)*log((x0-x_end)/x_end);


t = 0:0.01:1.2*tau;
x = x0*sigmf(t,[-a/tau c]);
plot(t,x);
ylabel('phase variable');
xlabel('time [s]');
ylim([-0.05+min(y) max(x)+0.05]);