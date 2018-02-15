clc;
close all;
clear;

Fs = 1000;
Ts = 1/Fs;
Time = 0:Ts:2;
f1 = 2;
f2 = 40;
f3 = 100;
X = 10*sin(Time*2*pi*f1) + 3*sin(Time*2*pi*f2) + 1*sin(Time*2*pi*f3);

Freq_cut = 20;


[f, Pfx, F_X] = getSingleSidedFourier(X, Fs);


k = find(f<Freq_cut);
k = k(end);
F_Y = F_X;
F_Y(k:end) = [];
Time2(k:end) = [];
F_Y = 2*F_Y; % * sum(abs(F_X))/sum(abs(F_Y));
Y = ifft(F_Y);
Y = real(Y);
[f2, Pfy, F_Y] = getSingleSidedFourier(Y, Fs);


figure
plot(Time, X, Time, Y);
legend('x','y');
xlabel('time [s]');


figure
hold on
plot(f,Pfx);
plot(f2,Pfy);
legend('Pfx','Pfy');
xlabel('frequency [Hz]');