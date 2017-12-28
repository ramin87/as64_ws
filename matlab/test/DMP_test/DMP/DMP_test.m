clc;
% close all;
clear;
format compact;

global cmd_args

%% initialize cmd params
cmd_args = get_cmd_args();

% check if the cmd args are constistent

if (strcmpi(cmd_args.DMP_TYPE, 'DMP-Shannon'))
    if (cmd_args.u_end < 0.9)
        msg = sprintf('\ncmd_args.u_end = %f.\ncmd_args.DMP_TYPE = %s \ncmd_args.u_end should have a value close to 1.0\n', cmd_args.u_end, cmd_args.DMP_TYPE);
        warning(msg);
    end
    
    if (~strcmpi(cmd_args.CAN_FUN_TYPE, 'sigmoid'))
        msg = sprintf('\ncmd_args.CAN_FUN_TYPE = %s.\ncmd_args.DMP_TYPE = %s \ncmd_args.CAN_FUN_TYPE should be %s\n', cmd_args.CAN_FUN_TYPE, cmd_args.DMP_TYPE, 'sigmoid');
        warning(msg);
    end
else
    if (cmd_args.u_end > 0.1)
        msg = sprintf('\ncmd_args.u_end = %f.\ncmd_args.DMP_TYPE = %s \ncmd_args.u_end should have a value lower than 0.1\n', cmd_args.u_end, cmd_args.DMP_TYPE);
        warning(msg);
    end
end


if (~cmd_args.USE_PHASE_STOP)
    cmd_args.a_py = 0;
end

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();


%% Load demos and process demos
load data/data.mat data Qd_data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_data(data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);
% [Qd_data, v_rot_d_data, dv_rot_d_data] = process_Q_data(Q_data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of training points

%% Set up DMP params
tau = (n_data-1)*Ts;

% Init canonical system
can_sys_ptr = CanonicalSystem();

% Optionally, one can set the clock's starting and end value, but in this case 'init' must be called again
can_sys_ptr.can_clock.x0 = cmd_args.x0;
can_sys_ptr.can_clock.x_end = cmd_args.x_end;

% Optionally, one can set the steepness of the sigmoid, but in this case 'init' must be called again
if (strcmpi(cmd_args.CAN_FUN_TYPE,'sigmoid'))
    can_sys_ptr.can_fun.a_u = 500;
end

can_sys_ptr.init(cmd_args.CAN_CLOCK_TYPE, cmd_args.CAN_FUN_TYPE, tau, cmd_args.u_end, cmd_args.u0);


extraArgNames = cell(0);
extraArgValues = cell(0);
extraArgNames{1} = 'k_trunc_kernel';     extraArgValues{1} = cmd_args.k_trunc_kernel;
extraArgNames{2} = 'Wmin';               extraArgValues{2} = cmd_args.Wmin;
extraArgNames{3} = 'Freq_min';           extraArgValues{3} = cmd_args.Freq_min;
extraArgNames{4} = 'Freq_max';           extraArgValues{4} = cmd_args.Freq_max;
extraArgNames{5} = 'P1_min';             extraArgValues{5} = cmd_args.P1_min;

dmp = cell(D,1);
for i=1:D
    if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
       dmp{i} = DMP();
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
        dmp{i} = DMP_bio();
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
        dmp{i} = DMP_plus();
%         dmp{i}.k_trunc_kernel = cmd_args.k_trunc_kernel;
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-Shannon'))
        dmp{i} = DMP_Shannon();
%         dmp{i}.Wmin = cmd_args.Wmin;
%         dmp{i}.Freq_min = cmd_args.Freq_min;
%         dmp{i}.Freq_max = cmd_args.Freq_max;
%         dmp{i}.P1_min = cmd_args.P1_min;
    else
        error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
    end
    
    dmp{i}.init(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K, extraArgNames, extraArgValues);
end

F_offline_train_data = [];
Fd_offline_train_data = [];
Time_offline_train = [];
offline_train_mse = [];
online_train_mse = [];
%% Train the DMP
if (cmd_args.OFFLINE_DMP_TRAINING_enable)
    disp('DMP training...')
    tic
    offline_train_mse = zeros(D,1); 
    n_data = size(yd_data,2);
    Time = (0:n_data-1)*Ts;
    for i=1:D

        y0 = yd_data(i,1);
        g0 = yd_data(i,end);

        ind = 1:n_data;
    %     ind = randperm(n_data);
        T = Time(ind);
        yd = yd_data(i,ind);
        dyd = dyd_data(i,ind);
        ddyd = ddyd_data(i,ind);

        dmp{i}.set_training_params(cmd_args.train_method, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(T, yd, dyd, ddyd, y0, g0);      

        F_offline_train_data = [F_offline_train_data; F_train];
        Fd_offline_train_data = [Fd_offline_train_data; Fd_train];

    end
    Time_offline_train = (0:(size(F_offline_train_data,2)-1))*Ts;

    toc
end

number_of_kernels = dmp{1}.N_kernels
n_data


%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g0 = cmd_args.goal_scale*yd_data(:,end); 
g = g0; 
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;
dg = zeros(D,1);
x = cmd_args.x0;
dx = 0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;
t = 0;
y_robot = y0;
dy_robot = zeros(D,1);
ddy_robot = zeros(D,1);
dz = zeros(D,1);
z = zeros(D,1);
scaled_forcing_term = zeros(D,1);
shape_attr = zeros(D,1);
goal_attr = zeros(D,1);

P_lwr = cell(D,1);
for i=1:D
    P_lwr{i} = ones(cmd_args.N_kernels,1)*cmd_args.RLWR_P;
end
F = zeros(D,1);
Fd = zeros(D,1);

Fdist = 0;

log_data = get_logData_struct();   

log_data.dmp = dmp;

log_data.Time_demo = Time_demo;
log_data.yd_data = yd_data;
log_data.dyd_data = dyd_data;
log_data.ddyd_data = ddyd_data;

log_data.D = D;
log_data.Ts = Ts;
log_data.g0 = g0;

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_offline_train_data;
log_data.Fd_offline_train_data = Fd_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data.Psi_data = cell(D,1);
log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);
log_data.shape_attr_data = [];
log_data.goal_attr_data = [];


tau0 = can_sys_ptr.get_tau();
tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.set_tau(tau);

iters = 0;

if (cmd_args.ONLINE_DMP_UPDATE_enable)
    dt = Ts; % sync sim with training data
else
    dt = cmd_args.dt;
end

disp('DMP simulation...')
tic
while (true)

    %% data logging

    log_data.Time = [log_data.Time t];
    
    log_data.y_data = [log_data.y_data y];
    log_data.dy_data = [log_data.dy_data dy];   
    log_data.z_data = [log_data.z_data z];
    log_data.dz_data = [log_data.dz_data dz];
        
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_robot_data = [log_data.y_robot_data y_robot];
    log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];
    log_data.ddy_robot_data = [log_data.ddy_robot_data ddy_robot];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data g];
    
    log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];
  
    
    %% DMP simulation

    %% Position DMP
    
    for i=1:D
        
        if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
            
            log_data.DMP_w{i} = [log_data.DMP_w{i} dmp{i}.w];
            log_data.P_lwr{i} = [log_data.P_lwr{i} P_lwr{i}];
            
            yd = yd_data(i,iters+1);
            dyd = dyd_data(i,iters+1);
            ddyd = ddyd_data(i,iters+1);
            P_lwr{i} = dmp{i}.update_weights(x, yd, dyd, ddyd, y0(i), g(i), P_lwr{i}, cmd_args.RLWR_lambda); 

            F(i) = dmp{i}.calc_Fd(x, y(i), dy(i), ddy(i), y0(i), g(i));
            Fd(i) = dmp{i}.calc_Fd(x, yd, dyd, ddyd, y0(i), g(i));
            
        end
        
        Psi = dmp{i}.activation_function(x);
        log_data.Psi_data{i} = [log_data.Psi_data{i} Psi(:)];

        shape_attr(i) = dmp{i}.shape_attractor(x, y0(i), g(i));
        goal_attr(i) = dmp{i}.goal_attractor(x, y(i), dy(i), g(i));
        scaled_forcing_term(i) = dmp{i}.forcing_term(x)*dmp{i}.forcing_term_scaling(x, y0(i), g(i));
        
        y_c = cmd_args.a_py*(y_robot(i)-y(i));
        z_c = 0;
        
        [dy(i), dz(i)] = dmp{i}.get_states_dot(x, y(i), z(i), y0(i), g(i), y_c, z_c);
        
        ddy(i) = dz(i)/dmp{i}.get_v_scale();
        ddy_robot(i) = ddy(i) + inv(cmd_args.Md) * ( - cmd_args.Dd*(dy_robot(i) - dy(i)) - cmd_args.Kd*(y_robot(i)-y(i)) + Fdist ); 
        
        
    end
    
    if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
        log_data.Time_online_train = [log_data.Time_online_train t];
        log_data.F_online_train_data = [log_data.F_online_train_data F];
        log_data.Fd_online_train_data = [log_data.Fd_online_train_data Fd];
    end
    
    %% Goal change
    if (cmd_args.ONLINE_GOAL_CHANGE_ENABLE)
        if (ind_g_chage <= N_g_change)
            if (abs((t-cmd_args.time_goal_change(ind_g_chage))) < dt/2.0)
%                 disp('Goal change')
%                 t
                g0 = cmd_args.goal_change(ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg = cmd_args.a_g*(g0-g)/can_sys_ptr.get_tau();
    else
        dg = zeros(size(dg));
    end
    
    %% Update phase variable

    dx = can_sys_ptr.get_phaseVar_dot(x);
    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
    
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        dg = dg*stop_coeff;
        
%         can_sys_ptr.set_tau(tau0*stop_coeff);
    end
    
    %% Stopping criteria
    err_p = max(abs(g0-y_robot));
    if (err_p <= cmd_args.tol_stop ...
        && t>=tau)
        break; 
    end
%     t
%     err_p
    
    iters = iters + 1;
    if (t>=tau && iters>=cmd_args.max_iters)
        warning('Iteration limit reached. Stopping simulation...\n');
        break;
    end
    
    %% Numerical integration
    t = t + dt;
    
    x = x + dx*dt;
    
    y = y + dy*dt;
    z = z + dz*dt;
    
    y_robot = y_robot + dy_robot*dt;
    dy_robot = dy_robot + ddy_robot*dt;
    
    g = g + dg*dt;

end
toc

log_data.u_data = can_sys_ptr.get_shapeVar(log_data.x_data);

save data/dmp_results.mat log_data cmd_args;
    

%% Find mean square error between the signals

if (cmd_args.ONLINE_DMP_UPDATE_enable)
   online_train_mse = zeros(D,1);   
   for i=1:D
       N = length(log_data.F_online_train_data(i,:));
       online_train_mse(i) = 0;
       A = abs(log_data.Fd_online_train_data(i,:)-log_data.F_online_train_data(i,:));
       B = max(abs([log_data.Fd_online_train_data(i,:); log_data.F_online_train_data(i,:)])) + eps;
       online_train_mse(i) = sum(A ./ B) / length(A);
   end
end

if (cmd_args.OFFLINE_DMP_TRAINING_enable)
   offline_train_mse = zeros(D,1);
   for i=1:D
       N = length(log_data.F_offline_train_data(i,:));
%        offline_train_mse(i) = 0;
%        A = abs(log_data.Fd_offline_train_data(i,:)-log_data.F_offline_train_data(i,:));
%        B = max(abs([log_data.Fd_offline_train_data(i,:); log_data.F_offline_train_data(i,:)])) + eps;
%        offline_train_mse(i) = sum(A ./ B) / length(A);
       
       offline_train_mse(i) = norm(log_data.Fd_offline_train_data(i,:)-log_data.F_offline_train_data(i,:))/N;
   end
end

Time = log_data.Time;
Time_demo = log_data.Time_demo;
y_data = log_data.y_data;
yd_data = log_data.yd_data;

y_data2 = cell(D,1);
yd_data2 = cell(D,1);

sim_mse = zeros(D,1);
for i=1:D
   [~, y_data2{i}, yd_data2{i}] = temp_align_signals(Time, y_data(i,:), Time_demo, yd_data(i,:)); 
   sim_mse(i) = norm(y_data2{i} - yd_data2{i});
end

sim_mse_dtw = zeros(D,1);
dist_f = @(s1,s2) norm(s1-s2);
[Time1, z1, Time2, z2] = spatial_align_signals(Time, y_data, Time_demo, yd_data, dist_f);
for i=1:D
   sim_mse_dtw(i) = norm(z1(i,:) - z2(i,:))/length(z1(i,:));
end

offline_train_mse
online_train_mse
sim_mse
sim_mse_dtw


% c = dmp{1}.c'
% h = dmp{1}.h'
% w = dmp{1}.w;
% 
% fprintf('%.6f\n',w)
