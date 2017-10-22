clc;
close all;
clear;
format compact;

global a_z b_z D N_kernels x0 x_end tau std_K ...
       USE_GOAL_FILT a_g USE_PHASE_STOP a_px a_py ...
       CAN_SYS_TYPE ...
       tol_stop max_iters dt ...
       Kd Dd ...
       APPLY_DISTURBANCE Fdist_min Fdist_max t1_fdist t2_fdist ...
       fontsize

%% initialize global params
set_params();

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

USE_2nd_order_can_sys = false;

%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data, Ts, 0.01, 0.03);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Plot the training data
%plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data);


%% Set up DMP params
tau = (n_data-1)*Ts;

D = size(yd_data,1); % dimensionality of training data

N_kernels

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


dmp = cell(D,1);
for i=1:D
    dmp{i} = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K, USE_GOAL_FILT, a_g);
end


%% Train the DMP
disp('DMP training...')
tic
train_method = 'LWR'; % train the DMP using LWR (Locally Weighted Regression)
%train_method = 'LS'; % train the DMP using LS (Least Squares)
train_mse = zeros(D,1);
for i=1:D
    [train_mse(i), F_train, Fd_train] = dmp{i}.train(yd_data(i,:), dyd_data(i,:), ddyd_data(i,:), Ts, train_method);      
end
toc

%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g = yd_data(:,end); 
g0 = g;
dg = zeros(D,1);
if (USE_GOAL_FILT), g = y0; end
x = x0;
dx = 0;
if (USE_2nd_order_can_sys)
    u = 0;
else
    u = x;
end
du = 0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;
t = 0;
y_robot = y0;
dy_robot = zeros(D,1);
dz = zeros(D,1);
z = tau*dy + a_py*(y_robot-y);
force_term = zeros(D,1);

Fdist = 0;

% Variables to store the simulation data
Time = [];
dy_data = [];
y_data = [];
dy_robot_data = [];
y_robot_data = [];
z_data = [];
x_data = [];
u_data = [];
Fdist_data = [];
Force_term_data = [];
g_data = cell(D,1);
Psi_data = cell(D,1);
f_data = cell(D,1);
shape_attr_data = cell(D,1);
goal_attr_data = cell(D,1);


tau = 1*tau;
can_sys_ptr.tau = tau;

iters = 0;

for i=1:D
    dmp{i}.g0 = g0(i);
end

disp('DMP simulation...')
tic
while (true)

    %% data logging

    Time = [Time t];
    
    dy_data = [dy_data dy];
    y_data = [y_data y];

    dy_robot_data = [dy_robot_data dy_robot];
    y_robot_data = [y_robot_data y_robot];

    z_data = [z_data z];
        
    x_data = [x_data x];
    u_data = [u_data u];
    
    Fdist_data = [Fdist_data Fdist];
    
    Force_term_data = [Force_term_data force_term];
    
    
    %% DMP simulation
    
    for i=1:D
        
        v_scale = dmp{i}.get_v_scale();% 1 / (tau*dmp{i}.a_s);
        
        Psi = dmp{i}.activation_function(x);
        Psi_data{i} = [Psi_data{i} Psi(:)];
        
        f = dmp{i}.forcing_term(x);
        
        shape_attr = dmp{i}.shape_attractor(x,dmp{i}.g0,y0(i));
        goal_attr = dmp{i}.goal_attractor(y(i),dy(i),g(i));
        
        f_data{i} = [f_data{i} f];
        shape_attr_data{i} = [shape_attr_data{i} shape_attr];
        goal_attr_data{i} = [goal_attr_data{i} goal_attr];
        g_data{i} = [g_data{i} g(i)];
        
        
        force_term = dmp{i}.forcing_term(x)*u*(g0(i)-y0(i));
        
        dz(i) = ( dmp{i}.a_z*(dmp{i}.b_z*(g(i)-y(i))-z) + force_term ) / v_scale;
        
        dy(i) = ( z(i) - a_py*(y_robot(i)-y(i)) ) / v_scale;
        
        dy_robot(i) = dy(i) - (Kd/Dd)*(y_robot(i)-y(i)) + Fdist/Dd; 
 
        % Get DMP output, i.e. the accelaration
        %ddy(i) = dmp.ts^2 * ( dmp.a{i}*(dmp.b{i}*(g(i)-y(i))-dy(i)/dmp.ts) + f(i)*x*(g(i)-y0(i)) ); 
        %ddy(i) = dmp{i}.get_output(y(i),dy(i),g(i),y0(i),x); 
      
    end
    
    if (USE_GOAL_FILT)
        dg = -a_g*(g-g0)/can_sys_ptr.tau;
    else
        dg = 0;
    end
    
    %% Update phase variable
    
    X_in = x;
    if (USE_2nd_order_can_sys)
        X_in = [X_in; u];
    end
        
    X_out = can_sys_ptr.get_derivative(X_in);%, y, y_robot);
    
%     x = X_out(1);
    dx = X_out(1);
    if (length(X_out) > 1)
%         u = X_out(2);
        du = X_out(2);
    else
        du = dx;
    end
    
    %% Update disturbance force
    if (APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    if (USE_PHASE_STOP)
        stop_coeff = 1/(1+a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        du = du*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    err = max(abs(g0-y_robot));
    if (err <= tol_stop), break; end
    
    iters = iters + 1;
    if (iters >= max_iters), break; end
    
    %% Numerical integration
    t = t + dt;
    
    y = y + dy*dt;
    
    z = z + dz*dt;
    
    g = g + dg*dt;
    
    x = x + dx*dt;
    if (x<0), x=0; end % zero crossing can occur due to numberical integration
    
    u = u + du*dt;
    
    y_robot = y_robot + dy_robot*dt;

end
toc

e_track_data = y_data - y_robot_data;

for i=1:D

figure;
subplot(2,1,1);
plot(Time,y_data(i,:)-g0(i));
legend({'$e_{DMP}=p_{DMP}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
subplot(2,1,2);
plot(Time,y_robot_data(i,:)-g0(i));
legend({'$e_{robot}=p_{robot}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);


n_splots = 3;
figure;
subplot(n_splots,1,1);
plot(Time,x_data, Time,u_data);
legend({'$x$','$u$'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,2);
plot(Time,y_robot_data(i,:), Time,y_data(i,:), Time_demo,yd_data(i,:), Time,g_data{i}, '--', Time(end),g0(i),'r*','Markersize',10);
legend({'$p_{robot}$','$p_{DMP}$','$p_{train}$','goal evolution','$p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,3);
plot(Time,Fdist_data);
legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);

n_splots = 4;
figure;
subplot(n_splots,1,1);
plot(Time,Fdist_data);
legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);
xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,2);
plot(Time,Force_term_data);
legend({'Forcing term'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,3);
plot(Time,e_track_data(i,:));
legend({'$e_{track}$'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,4);
plot(Time,dy_robot_data(i,:), Time, dy_data(i,:), Time, z_data(i,:));
legend({'$\dot{y_{robot}}$','$\dot{y_{DMP}}$','$z_{DMP}$'},'Interpreter','latex','fontsize',fontsize);

end

%% Find mean square error between the signals

dist_f = @(s1,s2) norm(s1-s2);
dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data, yd_data, dtw_win, dist_f);
sim_mse = sum(C)/length(C);



%% ========   Plot results  ========
disp('Ploting results...')
tic

%% Plot 'F' training
t = (0:(length(F_train)-1))*Ts;
figure;
subplot(2,1,1);
plot(t,F_train,t,Fd_train);
legend({'$F$','$F_d$'},'Interpreter','latex','fontsize',fontsize);
subplot(2,1,2);
plot(t,F_train-Fd_train);
legend({'$F-F_d$'},'Interpreter','latex','fontsize',fontsize);


%% Plot phase variable evolution
figure;
plot(Time,x_data, Time, u_data);
legend({'x','u'},'Interpreter','latex','fontsize',fontsize);
title('phase variable evolution','Interpreter','latex','fontsize',fontsize);
axis tight;


%% Plot goal evolution
if (USE_GOAL_FILT)
    figure;
    for i=1:D
        subplot(D,1,i);
        plot(Time,g_data{i});
        if (i==1), title('Goal evolution','Interpreter','latex','fontsize',fontsize); end
        axis tight;
    end
end
    
lineWidth = 1.2;
% for i=1:D
%    figure;
%    hold on;
%    plot(Time,shape_attr_data{i},'LineWidth',lineWidth);
%    plot(Time,goal_attr_data{i},'LineWidth',lineWidth);
%    plot(Time, shape_attr_data{i}+goal_attr_data{i}, 'LineWidth',lineWidth);
%    plot(Time, ddy_data(i,:),'LineWidth',lineWidth);
%    legend({'shape-attr','goal-attr','goal+shape','$\ddot{y}$'},'Interpreter','latex','fontsize',fontsize);
%    hold off;
% end

%% Plot DMP simulation and demo pos, vel, accel
lineWidth = 1.1;
plot_signals_and_errorSignal(Time,y_data, Time_demo,yd_data, 'DMP', 'demo', 'Position', lineWidth);
% plot_signals_and_errorSignal(Time,dy_data, Time_demo,dyd_data, 'DMP', 'demo', 'Velocity', lineWidth);
% plot_signals_and_errorSignal(Time,ddy_data, Time_demo,ddyd_data, 'DMP', 'demo', 'Accelaration', lineWidth);


%% Plot shape attractor
% figure;
% for i=1:D
%     subplot(D,1,i);
%     plot(Time, shape_attr_data{i});
%     if (i==1), title('Shape attractor','Interpreter','latex','fontsize',fontsize); end
% end


%% Plot psi activations with respect to phase variable
% for i=1:D
%     plot_psi_activations_and_psiWeightedSum(x_data,Psi_data{i}, f_data{i}, dmp{i}.c, dmp{i}.w);
% end

% % Plot psi activations with respect to time
% for i=1:D
%     figure
%     plot(Time, Psi_data{i});
%     xlabel('time [s]','Interpreter','latex','fontsize',fontsize);
% end

%% Plot 2D or 3D line path of DMP simulation and demo
if (D==2 || D==3)
    plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
end

toc

train_mse
sim_mse
dtw_dist/length(C)