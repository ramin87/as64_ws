clc;
close all;
clear;

global tau a_u b_u x_end

tau = 1.5; 
x_end = 0.01;

can_sys = DMP_spring_damper_canonical_system(x_end, tau);

a_u = can_sys.a_u;
b_u = can_sys.b_u;


a_u
b_u

% 
% a_u = 20;
% b_u = a_u/4;
% can_sys.a_u = a_u;
% can_sys.b_u = b_u;
% 
% a_u
% b_u

dt = 0.001;

Time = 0;
x_data = [];
dx_data = [];

x0 = 1;
x_g = 0;

x = x0;
dx = 0;

tol_stop = x_end;

while (true)

    x_data = [x_data x];
    dx_data = [dx_data dx];
    
    ddx = a_u*(b_u*(x_g-x) - dx*tau)/tau^2;
    
    x = x + dx*dt;
    dx = dx + ddx*dt;

%     X_in = [x; dx];
%     
%     X_out = can_sys.get_single_step_output(dt, X_in);
%     x = X_out(1);
%     dx = X_out(2);

    err = norm(x-x_g);
    if (err < tol_stop), break; end
    
    %err

    Time = [Time Time(end)+dt];
    
end


X2 = can_sys.get_continuous_output(Time, 1);
x2_data = X2(1,:);
dx2_data = X2(2,:);

% omega_x = a_u/(2*tau);
% x2_data = x0*( exp(-omega_x*Time) + omega_x*Time.*exp(-omega_x*Time) );

figure;
subplot(2,1,1);
plot(Time,x_data, Time,x2_data);
legend('x','x2');
subplot(2,1,2);
plot(Time,dx_data, Time,dx2_data);
legend('dx','dx2');









