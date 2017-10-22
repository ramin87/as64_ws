clc;
close all;
clear;
format compact;

global cmd_args


%% initialize global params
set_params();

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

USE_2nd_order_can_sys = false;

%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Set up DMP params
tau = (n_data-1)*Ts;

D = size(yd_data,1); % dimensionality of training data

number_of_kernels = cmd_args.N_kernels

if (strcmpi(cmd_args.CAN_SYS_TYPE,'lin'))
    can_sys_ptr = DMP_lin_canonical_system(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'exp'))
    can_sys_ptr = DMP_exp_canonical_system(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = DMP_spring_damper_canonical_system(cmd_args.x_end, tau);
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',cmd_args.CAN_SYS_TYPE);
end


dmp = cell(D,1);
for i=1:D
    dmp{i} = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K, cmd_args.USE_GOAL_FILT, cmd_args.a_g);
end


F_train_data = cell(D,1);
Fd_train_data = cell(D,1);

%% Train the DMP
disp('DMP training...')
tic
train_mse = zeros(D,1);
for i=1:D
    [train_mse(i), F_train, Fd_train] = dmp{i}.train(yd_data(i,:), dyd_data(i,:), ddyd_data(i,:), Ts, cmd_args.train_method);      
    F_train_data{i} = F_train;
    Fd_train_data{i} = Fd_train;
    
%     c =  dmp{i}.c'
%     
%     h =  dmp{i}.h'
%     
%     w =  dmp{i}.w'
    
end
Time_train = (0:(length(F_train_data{1})-1))*Ts;

toc

%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g0 = yd_data(:,end); 
g = g0;
dg = zeros(D,1);
if (cmd_args.USE_GOAL_FILT), g = y0; end
x = cmd_args.x0;
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
z = tau*dy + cmd_args.a_py*(y_robot-y);
force_term = zeros(D,1);

Fdist = 0;

% Variables to store the simulation data
log_data = struct('Time_demo',Time_demo, 'yd_data',yd_data, 'dyd_data',dyd_data, 'ddyd_data',ddyd_data, ...
                  'D',D, 'Ts',Ts, 'g0', g0, ...
                  'Time_train', Time_train, 'F_train_data', F_train_data, 'Fd_train_data', Fd_train_data, ...
                  'Time',[],'y_data',[],'dy_data',[],'y_robot_data',[],'dy_robot_data',[],'z_data',[],'dz_data',[], ...
                  'x_data',[],'u_data',[], 'Fdist_data',[], 'Force_term_data',[], 'g_data', cell(D,1), ...
                  'Psi_data', cell(D,1), 'shape_attr_data', cell(D,1), 'goal_attr_data', cell(D,1));

           
log_data.Time_train = Time_train;
log_data.F_train_data = F_train_data;
log_data.Fd_train_data = Fd_train_data;
log_data.g_data = cell(D,1);
log_data.Psi_data = cell(D,1);
log_data.shape_attr_data = cell(D,1);
log_data.goal_attr_data = cell(D,1);


tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.tau = tau;

iters = 0;
dt = cmd_args.dt;

for i=1:D
    dmp{i}.g0 = g0(i);
end

disp('DMP simulation...')
tic
while (true)

    %% data logging

    log_data.Time = [log_data.Time t];
    
    log_data.dy_data = [log_data.dy_data dy];
    log_data.y_data = [log_data.y_data y];

    log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];
    log_data.y_robot_data = [log_data.y_robot_data y_robot];

    log_data.z_data = [log_data.z_data z];
    log_data.dz_data = [log_data.dz_data dz];
        
    log_data.x_data = [log_data.x_data x];
    log_data.u_data = [log_data.u_data u];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data force_term];
    
    
    %% DMP simulation
    
    for i=1:D
        
        v_scale = dmp{i}.get_v_scale();% 1 / (tau*dmp{i}.a_s);
        
        Psi = dmp{i}.activation_function(x);
        log_data.Psi_data{i} = [log_data.Psi_data{i} Psi(:)];

        shape_attr = dmp{i}.shape_attractor(x,g0(i),y0(i));
        goal_attr = dmp{i}.goal_attractor(y(i),dy(i),g(i));
        
        log_data.shape_attr_data{i} = [log_data.shape_attr_data{i} shape_attr];
        log_data.goal_attr_data{i} = [log_data.goal_attr_data{i} goal_attr];
        log_data.g_data{i} = [log_data.g_data{i} g(i)];
        
        
        force_term(i) = dmp{i}.forcing_term(x)*u*(g0(i)-y0(i));
        
        dz(i) = ( dmp{i}.a_z*(dmp{i}.b_z*(g(i)-y(i))-z(i)) + force_term(i) ) / v_scale;
        
        dy(i) = ( z(i) - cmd_args.a_py*(y_robot(i)-y(i)) ) / v_scale;
        
        dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd; 
 
        % Get DMP output, i.e. the accelaration
        %ddy(i) = dmp.ts^2 * ( dmp.a{i}*(dmp.b{i}*(g(i)-y(i))-dy(i)/dmp.ts) + f(i)*x*(g(i)-y0(i)) ); 
        %ddy(i) = dmp{i}.get_output(y(i),dy(i),g(i),y0(i),x); 
      
    end
    
    if (cmd_args.USE_GOAL_FILT)
        dg = -cmd_args.a_g*(g-g0)/can_sys_ptr.tau;
    else
        dg = zeros(size(g));
    end
    
    %% Update phase variable
    
    X_in = x;
    if (USE_2nd_order_can_sys)
        X_in = [X_in; u];
    end
        
    X_out = can_sys_ptr.get_derivative(X_in);
    
%     x = X_out(1);
    dx = X_out(1);
    if (length(X_out) > 1)
%         u = X_out(2);
        du = X_out(2);
    else
        du = dx;
    end
    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        du = du*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    err = max(abs(g0-y_robot));
    if (err <= cmd_args.tol_stop), break; end
    
    iters = iters + 1;
    if (iters >= cmd_args.max_iters), break; end
    
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

save dmp_results.mat log_data cmd_args;
...
%     Time_demo yd_data dyd_data ddyd_data ...
%     D Ts ...
%     Time y_data dy_data z_data dz_data x_data u_data Force_term_data ...
%     Fdist_data ...
%     g0 g_data ...
%     y_robot_data dy_robot_data ...
%     Time_train F_train_data Fd_train_data
    

%% Find mean square error between the signals

y_data = log_data.y_data;
yd_data = log_data.yd_data;

dist_f = @(s1,s2) norm(s1-s2);
dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data, yd_data, dtw_win, dist_f);
sim_mse = sum(C)/length(C);


train_mse
sim_mse
dtw_dist/length(C)
