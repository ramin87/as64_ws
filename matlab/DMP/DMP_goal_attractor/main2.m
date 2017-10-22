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
Kd = 30;
Fd = 20;
a = 20;
k = a/4;

pT = 2;

MAX_ITERS = 1000;
TOL_STOP = 1e-3;
dt = 0.002;

p_data = [];
pr_data = [];
z_data = [];
Iz_data = [];

% Initial values
p = 0.4;
pr = 0.4;
z = 0;
Iz = -pT;

iters = 0;

while (iters < MAX_ITERS)
    
    iters = iters + 1;
    
    p_data = [p_data p];
    pr_data = [pr_data pr];
    z_data = [z_data z];
    Iz_data = [Iz_data Iz];
    
    ep = norm(p-pT);
    if (ep < TOL_STOP), break; end
    ep
    
    [dz, dpr, dp, dIz] = DMP_sys(z,p,pr,Iz,pT);
      
    p = p + dp*dt;
    pr = pr + dpr*dt;
    z = z + dz*dt;
    Iz = Iz +dIz*dt;
end

Time = 0:dt:(iters-1)*dt;

fontsize = 14;

figure;
plot(Time,p_data,Time,pr_data,Time,z_data,Time,Iz_data,Time(end),pT,'r*','Markersize',10);
legend({'$p$','$p_r$','$z$','$I_z$','$p_T$'},'Interpreter','latex','fontsize',fontsize);

figure;
subplot(2,1,1);
plot(Time,pr_data-pT);
legend({'$e_r=p_r-p_T$'},'Interpreter','latex','fontsize',fontsize);
subplot(2,1,2);
plot(Time,p_data-pT);
legend({'$e_p=p-p_T$'},'Interpreter','latex','fontsize',fontsize);

% function dp = robot_sys(p,pr,dpr)
% 
%     global Md Dd Kd Fd
% 
%     dp = dpr - (Kd/Dd)*(p-pr) + Fd/Dd;
% 
% end


function [dz, dpr, dp, dIz] = DMP_sys(z,p,pr,Iz,pT)

    global Md Dd Kd Fd a k

    er = pr - pT;
    ep = p - pT;
    
    dz = a*(-z-k*ep);
    der = z - k*ep + k*Iz;
    dIz = z;
    dep = der - (Kd/Dd)*(ep-er) + Fd/Dd;
    
    dpr = der;
    dp = dep;
    
end



