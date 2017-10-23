clc;
close all
clear;

global Md Dd Kd Fd ...
       a k ...
       pT ...
       MAX_ITERS ...
       TOL_STOP ...
       dt


Md = 0;
Dd = 1;
Kd = 20;
Fd = 10;
a = 10;
k = a/4;

pT = 2;

MAX_ITERS = 5000;
TOL_STOP = 0.5e-4;
dt = 0.002;

p_data = [];
pr_data = [];
z_data = [];
Fd_data = [];

p = 0.4;
pr = 0.4;
z = 0;

iters = 0;
t = 0;

while (iters < MAX_ITERS)
    
    iters = iters + 1;
    
    p_data = [p_data p];
    pr_data = [pr_data pr];
    z_data = [z_data z];
    Fd_data = [Fd_data Fd];
    
    ep = norm(p-pT);
    if (ep < TOL_STOP), break; end
    ep
    
    Fd = fd_fun(t);
    
    [dz, dpr, dp] = DMP_sys(z,p,pr,pT,Fd);
      
    p = p + dp*dt;
    pr = pr + dpr*dt;
    z = z + dz*dt;
    t = t + dt;   
    
end

Time = 0:dt:(iters-1)*dt;

fontsize = 14;

figure;
plot(Time,p_data,Time,pr_data,Time,z_data,Time(end),pT,'r*','Markersize',10);
legend({'$p$','$p_r$','$z$','$p_T$'},'Interpreter','latex','fontsize',fontsize);

figure;
subplot(2,1,1);
plot(Time,pr_data-pT);
legend({'$e_r=p_r-p_T$'},'Interpreter','latex','fontsize',fontsize);
subplot(2,1,2);
plot(Time,p_data-pT);
legend({'$e_p=p-p_T$'},'Interpreter','latex','fontsize',fontsize);

figure;
plot(Time,Fd_data);
title('Disturbance force','Interpreter','latex','fontsize',fontsize);

% function dp = robot_sys(p,pr,dpr)
% 
%     global Md Dd Kd Fd
% 
%     dp = dpr - (Kd/Dd)*(p-pr) + Fd/Dd;
% 
% end


function [dz, dpr, dp] = DMP_sys(z,p,pr,pT,Fd)

    global Md Dd Kd a k

    er = pr - pT;
    ep = p - pT;
    
    dz = a*(-z-k*er);
    der = z + k*er - k*ep;
    dep = der - (Kd/Dd)*(ep-er) + Fd/Dd;
    
    dpr = der;
    dp = dep;
    
end


function Fd = fd_fun(t)

Fd = 0;

if (t>0.4 && t<1.4)
    Fd = 60;
end
    
end


