clc;
close all;
clear;

%% Set random number generator
s = rng;
rng(s);


%% general params
g = 9.81; % gravity acceleration

%% Object params
m_o = 10; % mass
l = 0.5; % length
d = 0.3; % distance of CoM from left end of the object
d_hat = 0.0; % initial estimate of CoM
w_o = m_o*g;

dt = 0.01; % simulation timestep
sigma_f_n = 0.15*w_o; % noise variance in measurements of forces
sigma_t_n = 0.05*w_o*d; % noise variance in measurements of torque

%% RLS params
d_rls = d_hat; % initial estimate
P_rls = 100; % covariance of initial estimation
lambda = 0.995; % forgeting factor

%% KF params
d_kf = d_hat; % initial estimate
P_kf = 100; % covariance of initial estimation
S_torq_n = 0.1; % covariance of measurement noise (in torque measurement)


%% initial pose and force/torque of the object
theta0 = -pi/4; % initial position
theta_g = pi/4; % goal position
dtheta = pi/6; % assume object rotates with constant speed

theta = theta0; % current position
force = [sin(theta); cos(theta)] * (-m_o*g);
f_x = force(1) + sigma_f_n*randn();
f_y = force(2) + sigma_f_n*randn();
torque = d*force(2) + sigma_t_n*randn();

theta_data = [];
f_x_data = [];
f_y_data = [];
torque_data = [];
Time_data = [];
d_rls_data = [];
d_kf_data = [];
t = 0.0; % current timestamp

while (true)
    
    d_rls_data = [d_rls_data d_rls];
    d_kf_data = [d_kf_data d_kf];
    Time_data = [Time_data t];
    theta_data = [theta_data theta];
    f_x_data = [f_x_data f_x];
    f_y_data = [f_y_data f_y];
    torque_data = [torque_data torque];
    
    %% calculate forces and torques
    force = [sin(theta); cos(theta)] * (-m_o*g);
    f_x = force(1) + sigma_f_n*randn();
    f_y = force(2) + sigma_f_n*randn();
    torque = d*force(2) + sigma_t_n*randn();
    
    %% KF estimation
    err_kf = torque-f_y*d_kf;
    K_kf = f_y*P_kf*inv(f_y^2*P_kf + S_torq_n);
    d_kf = d_kf + K_kf*err_kf;
    S_torq_n = S_torq_n - K_kf*f_y*P_kf;
    
    %% RLS estimation
    err_rls = torque-f_y*d_rls;
    K_rls = P_rls*f_y * inv(lambda + f_y'*P_rls*f_y);
    d_rls = d_rls + K_rls*err_rls;
    P_rls = (1/lambda) * (P_rls - K_rls*f_y'*P_rls);
    
    %% stopping conditions
    if (abs(theta-theta_g) < 1e-2)
        break;
    end
    
    %% numerical integration
    t = t + dt;
    theta = theta + sign(theta_g-theta)*dtheta*dt;
    
end

fontsize = 14;

figure;
subplot(2,2,[1 2]);
hold on;
plot(Time_data, theta_data*180/pi, 'LineWidth',1.2);
plot(Time_data(1), theta0*180/pi, 'ro', 'LineWidth',2, 'MarkerSize',8);
plot(Time_data(end), theta_g*180/pi, 'r*', 'LineWidth',2, 'MarkerSize',8);
legend({'$\theta$','$\theta_0$','$\theta_g$'}, 'Interpreter','latex', 'fontsize',fontsize);
ylabel('$degrees$', 'Interpreter','latex', 'fontsize',fontsize);
xlabel('time[$s$]', 'Interpreter','latex', 'fontsize',fontsize);
title('Object angle evolution', 'Interpreter','latex', 'fontsize',fontsize);
hold off;
subplot(2,2,3);
plot(Time_data, f_x_data, Time_data, f_y_data, 'LineWidth',1.2);
legend({'$f_x$','$f_y$'}, 'Interpreter','latex', 'fontsize',fontsize);
ylabel('[$N$]', 'Interpreter','latex', 'fontsize',fontsize);
xlabel('time[$s$]', 'Interpreter','latex', 'fontsize',fontsize);
title('Forces', 'Interpreter','latex', 'fontsize',fontsize);
subplot(2,2,4);
plot(Time_data, torque_data, 'LineWidth',1.2);
legend({'$\tau$'}, 'Interpreter','latex', 'fontsize',fontsize);
ylabel('[$Nm$]', 'Interpreter','latex', 'fontsize',fontsize);
xlabel('time[$s$]', 'Interpreter','latex', 'fontsize',fontsize);
title('Torque', 'Interpreter','latex', 'fontsize',fontsize);


figure;
hold on;
plot(Time_data, d_kf_data, 'LineWidth',1.2);
plot(Time_data, d_rls_data, 'LineWidth',1.2);
legend({'KF','RLS'}, 'Interpreter','latex', 'fontsize',fontsize);
ylabel('CoM $[m]$', 'Interpreter','latex', 'fontsize',fontsize);
xlabel('time[$s$]', 'Interpreter','latex', 'fontsize',fontsize);
title('Online CoM estimation', 'Interpreter','latex', 'fontsize',fontsize);
hold off;
