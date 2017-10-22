clc;
close all;
clear;

CAN_SYS_TYPE = 'spring-damper';
tau = 1;
time_step = 0.002;
t = 0:time_step:tau;
x_end = 0.005;
x0 = 1;

fontsize = 14;
lineWidth = 1.5;


if (strcmpi(CAN_SYS_TYPE,'lin'))
    can_sys_ptr = DMP_lin_canonical_system(x_end, tau);
elseif (strcmpi(CAN_SYS_TYPE,'exp'))
    can_sys_ptr = DMP_exp_canonical_system(x_end, tau);
elseif (strcmpi(CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = DMP_spring_damper_canonical_system(x_end, tau);
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',CAN_SYS_TYPE);
end


X = can_sys_ptr.get_continuous_output(t, x0);

n_out = size(X,1);

x = X(1,:);
u = x;
if (n_out == 2)
    u = X(2,:);
end
    
figure;
plot(t,x,t,u,'LineWidth',lineWidth);
legend({'$x$','u'},'Interpreter','latex','fontsize',fontsize);
xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);









    
