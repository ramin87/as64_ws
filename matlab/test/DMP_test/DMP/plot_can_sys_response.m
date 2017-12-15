clc;
close all;
clear;

set_matlab_utils_path();

CAN_CLOCK_TYPE = 'lin';
CAN_FUN_TYPE = 'sigmoid';
tau = 2.5;
dt = 0.002;
t = 0:dt:tau;
u_end = 0.99;
u0 = 1;

fontsize = 14;
lineWidth = 1.5;

can_sys_ptr = CanonicalSystem();
can_sys_ptr.init(CAN_CLOCK_TYPE, CAN_FUN_TYPE, tau, u_end, u0);


%% ============================================

[x, u] = can_sys_ptr.get_output(t);

T0 = t;
X0 = x;
U0 = u;
% figure;
% plot(t,x,t,u,'LineWidth',lineWidth);
% legend({'$x$','u'},'Interpreter','latex','fontsize',fontsize);
% xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
% axis tight

%% ============================================

tau = can_sys_ptr.get_tau();

can_sys_ptr.set_tau(tau*1.0);

x = 0;
u = can_sys_ptr.get_shapeVar(x);
t = 0;

x_data = x;
u_data = u;
t_data = t;

while (abs(u-u_end)>1e-4 && x<=1)

    [dx, u]=  can_sys_ptr.get_output_dot(x);

%     if (t>0.8 && t<1.5)
%         dx = 0;
%     end

    t = t + dt;
    x = x + dx*dt;

    u_err = abs(u-u_end)
    
    t_data = [t_data t];
    x_data = [x_data x];
    u_data = [u_data u];

end

figure;
subplot(1,2,1);
plot(T0,X0,T0,U0,'LineWidth',lineWidth);
legend({'$x$','u'},'Interpreter','latex','fontsize',fontsize);
xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
axis tight;
% axis equal;
subplot(1,2,2);
plot(t_data,x_data,t_data,u_data,'LineWidth',lineWidth);
legend({'$x$','u'},'Interpreter','latex','fontsize',fontsize);
xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
axis tight;
% axis equal;







    
