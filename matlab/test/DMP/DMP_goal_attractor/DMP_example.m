clc;
close all;
clear;
format compact;

global a_z b_z D N_kernels x0 x_end tau std_K ...
       USE_GOAL_FILT a_g USE_PHASE_STOP a_px a_py ...
       CAN_SYS_TYPE ...
       tol_stop max_iters dt ...
       Kd Dd ...
       fontsize

%% initialize global params
set_params();


%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();


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
    can_sys_ptr = DMP_lin_canonical_system(x_end, tau, USE_PHASE_STOP, a_px);
elseif (strcmpi(CAN_SYS_TYPE,'exp'))
    can_sys_ptr = DMP_exp_canonical_system(x_end, tau, USE_PHASE_STOP, a_px);
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
if (USE_GOAL_FILT), g = y0; end
x = x0;
dx = 0;
ddy = zeros(D,1);

y = y0;
dy = zeros(D,1);
y_robot = y0;
dy_robot = zeros(D,1);
dz = zeros(D,1);
z = tau*dy + a_py*(y_robot-y);
force_term = zeros(D,1);

Fdist = 0;

% Variables to store the simulation data
Time = [];
ddy_data = [];
dy_data = [];
y_data = [];
dy_robot_data = [];
y_robot_data = [];
z_data = [];
x_data = [];
dx_data = [];
Fdist_data = [];
Force_term_data = [];
g_data = cell(D,1);
Psi_data = cell(D,1);
f_data = cell(D,1);
shape_attr_data = cell(D,1);
goal_attr_data = cell(D,1);

% minimum difference between current and goal position
tau = 1*tau;
can_sys_ptr.tau = tau;

t = 0;

for i=1:D
    dmp{i}.g0 = g0(i);
end

disp('DMP simulation...')
tic
while (true)

    dy_data = [dy_data dy];
    y_data = [y_data y];

    dy_robot_data = [dy_robot_data dy_robot];
    y_robot_data = [y_robot_data y_robot];

    z_data = [z_data z];
        
    x_data = [x_data x];
    
    Fdist_data = [Fdist_data Fdist];
    
    Force_term_data = [Force_term_data force_term];
    
    Time = [Time t];
    
    for i=1:D
        
        v_scale = 1 / (tau*dmp{i}.a_s);
        
        force_term = dmp{i}.forcing_term(x)*x*(g0(i)-y0(i));
        
        dz(i) = v_scale * ( dmp{i}.a_z*(dmp{i}.b_z*(g(i)-y(i))-z) + force_term ) ;
        
        dy(i) = v_scale * ( z(i) - a_py*(y_robot(i)-y(i)) );
        
        dy_robot(i) = dy(i) - (Kd/Dd)*(y_robot(i)-y(i)) + Fdist/Dd;
           
    end
    
    t = t + dt;
    
    y = y + dy*dt;
    
    z = z + dz*dt;
    
    y_robot = y_robot + dy_robot*dt;
        
    x = can_sys_ptr.get_single_step_output(dt, x, y, y_robot);
    
    Fdist = Fdist_fun(t);
    
    if (USE_GOAL_FILT)
        dg = -a_g*(g-g0)/can_sys_ptr.tau;
        g = g + dg*dt;
    end

    %% data logging
    
    err = max(abs(g0-y_robot));
    if (err <= tol_stop), break; end
    
    max_iters = max_iters -1;
    if (max_iters <= 0), break; end
    
    
    
end
toc 

e_track_data = y_data - y_robot_data;

figure;
subplot(2,1,1);
plot(Time,y_data-g0);
legend({'$e_r=p_r-p_T$'},'Interpreter','latex','fontsize',fontsize);
subplot(2,1,2);
plot(Time,y_robot_data-g0);
legend({'$e_p=p-p_T$'},'Interpreter','latex','fontsize',fontsize);


n_splots = 3;
figure;
subplot(n_splots,1,1);
plot(Time,x_data);
legend({'Phase variable $x$'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,2);
plot(Time,y_robot_data,Time,y_data,Time_demo,yd_data,Time(end),g0,'r*','Markersize',10);
legend({'$p$','$p_r$','$p_{train}$','$p_T$'},'Interpreter','latex','fontsize',fontsize);
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
plot(Time,e_track_data);
legend({'$e_{track}$'},'Interpreter','latex','fontsize',fontsize);
subplot(n_splots,1,4);
plot(Time,dy_robot_data, Time, dy_data, Time, z_data);
legend({'$\dot{y}$','$\dot{y_r}$','$z_r$'},'Interpreter','latex','fontsize',fontsize);
