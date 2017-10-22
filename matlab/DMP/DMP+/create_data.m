clc;
close all;
clear;

Tend = 2;

Ts = 0.005;
t = 0:Ts:Tend;

D = 3;
y = zeros(D,length(t));
%y(1,:) = t.^2.5 - 0.2*t.^5;
y(1,:) = sin(8*t) + t - 2*exp(t);
y(2,:) = cos(12*t) + 2*t.^2 - 3*exp(t);
y(3,:) = 2*t.^2 - 2*t.^3 - 3*exp(-2*t);


data = y;
save('data.mat','data','Ts');

P = y';
save('demonstration.mat','P');

figure;
for i=1:D
    subplot(D,1,i);
    plot(t,y(i,:));
end


