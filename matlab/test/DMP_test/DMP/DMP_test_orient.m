clc;
% close all;
clear;
format compact;

global cmd_args

%% initialize cmd params
cmd_args = get_cmd_args();

if (~cmd_args.USE_PHASE_STOP)
    cmd_args.a_py = 0;
end

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

%% Load demos and process demos
load data/data.mat data Qd_data Ts

% calculate numerically the 1st and 2nd derivatives
[Qd_data, v_rot_d_data, dv_rot_d_data] = process_Q_data(Qd_data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(Qd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(v_rot_d_data,1); % dimensionality of training data


%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data


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


extraArgNames = {'k_trunc_kernel', 'Wmin', 'Freq_min', 'Freq_max', 'P1_min'};
extraArgValues = {cmd_args.k_trunc_kernel, cmd_args.Wmin, cmd_args.Freq_min, cmd_args.Freq_max, cmd_args.P1_min};

dmpo = DMP_orient();
dmpo.init(cmd_args.DMP_TYPE, cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_scale_factor, extraArgNames, extraArgValues);


Time_offline_train = [];
F_o_offline_train_data = [];
Fd_o_offline_train_data = [];
offline_train_o_mse = [];
%% Train the DMP
if (cmd_args.OFFLINE_DMP_TRAINING_enable)
    disp('DMP training...')
    tic
    
    Time = (0:n_data-1)*Ts;

    Q0 = Qd_data(:,1);
    Qg = Qd_data(:,end);
    
    trainParamsName = {'lambda', 'P_cov'};
    trainParamsValue = {cmd_args.lambda, cmd_args.P_cov};
    dmpo.set_training_params(cmd_args.train_method, trainParamsName, trainParamsValue);
    [offline_train_o_mse, F_o_offline_train_data, Fd_o_offline_train_data] = dmpo.train(Time, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg);   
    
    Time_offline_train = (0:(size(F_o_offline_train_data,2)-1))*Ts;
    
    toc
end


%% DMP simulation
% set initial values
x = cmd_args.x0;
dx = 0;
Q0 = Qd_data(:,1);
Qg0 = Qd_data(:,end);
Qg = Qg0;
Qg2 = Qg;
dg = zeros(D,1);
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;

v_rot = zeros(D,1);
dv_rot = zeros(D,1);
Q = Q0;
t = 0;
Q_robot = Q0;
v_rot_robot = zeros(D,1);
dv_rot_robot = zeros(D,1);
deta = zeros(D,1);
eta = zeros(D,1);
scaled_forcing_term = zeros(D,1);
shape_attr = zeros(D,1);
goal_attr = zeros(D,1);

F = zeros(D,1);
Fd = zeros(D,1);

Fdist = 0;

log_data = get_logData_struct(); 
log_data = get_logData_struct();   
log_data.dmp = cell(D,1);
for i=1:D, log_data.dmp{i} = dmpo.dmp{i}; end

log_data.Time_demo = Time_demo;
log_data.yd_data = quat2Vel(repmat(Qd_data(:,end),1,size(Qd_data,2)), Qd_data);
log_data.dyd_data = v_rot_d_data;
log_data.ddyd_data = dv_rot_d_data;

log_data.D = D;
log_data.Ts = Ts;
log_data.g0 = quat2Vel(Qg, Qg);

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_o_offline_train_data;
log_data.Fd_offline_train_data = Fd_o_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_o_offline_train_data;
log_data.Fd_offline_train_data = Fd_o_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data.Psi_data = cell(D,1);
log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);
log_data.shape_attr_data = [];
log_data.goal_attr_data = [];

tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.set_tau(tau);

iters = 0;

dt = cmd_args.dt;

disp('DMP simulation...')
tic
while (true)

    %% data logging
 
    log_data.Time = [log_data.Time t];
    
    log_data.y_data = [log_data.y_data quat2Vel(Qg0,Q)];
    log_data.dy_data = [log_data.dy_data v_rot];   
    log_data.ddy_data = [log_data.ddy_data dv_rot];   
    log_data.z_data = [log_data.z_data eta];
    log_data.dz_data = [log_data.dz_data deta];
        
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_robot_data = [log_data.y_robot_data quat2Vel(Qg0,Q_robot)];
    log_data.dy_robot_data = [log_data.dy_robot_data v_rot_robot];
    log_data.ddy_robot_data = [log_data.ddy_robot_data dv_rot_robot];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data quat2Vel(Qg0,Qg)];
    
    log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];

    
    %% DMP simulation

    %% Orientation DMP
    scaled_forcing_term = dmpo.forcing_term(x).*dmpo.forcing_term_scaling(x, Q0, Qg);
%     scaled_forcing_term =  dmpo.shape_attractor(x, Q0, Qg);
    
    Q_c = cmd_args.a_py*quatLog(quatProd(Q_robot,quatInv(Q)));
    eta_c = 0;
    [dQ, deta] = dmpo.get_states_dot(x, Q, eta, Q0, Qg, Q_c, eta_c);
    v_rot_temp = 2*quatProd(dQ,quatInv(Q));
    
    v_rot = v_rot_temp(2:4);
    dv_rot = deta / dmpo.get_v_scale();    
    dv_rot_robot = dv_rot + inv(cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - cmd_args.Kd_o*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist ); 
    
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg = cmd_args.a_g*quatLog(quatProd(Qg2,quatInv(Qg)))/can_sys_ptr.get_tau();
    else
        Qg = Qg2;
        dg = zeros(size(dg));
    end
    
    
    %% Goal change
    if (cmd_args.ONLINE_GOAL_CHANGE_ENABLE)
        if (ind_g_chage <= N_g_change)
            if (abs((t-cmd_args.time_goal_change(ind_g_chage))) < dt/2.0)
%                 disp('Goal change')
%                 t
                Qg2 = cmd_args.orient_goal_change(:,ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Update phase variable
        
    dx = can_sys_ptr.get_phaseVar_dot(x);

    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(quatLog(quatProd(Q_robot,quatInv(Q))))^2);
        
        dx = dx*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    %% Stopping criteria
    err_o = norm(quatLog(quatProd(Qg,quatInv(Q_robot))));
    if (err_o<cmd_args.orient_tol_stop && t>=tau) break; end

%     t
%     err_o
    
    iters = iters + 1;
    if (t>=tau && iters>=cmd_args.max_iters)
        warning('Iteration limit reached. Stopping simulation...\n');
        break; 
    end
    
    %% Numerical integration
    t = t + dt; 
    
    x = x + dx*dt;
    
    Q = quatProd(quatExp(v_rot*dt), Q);
    eta = eta + deta*dt;
    
    Q_robot = quatProd(quatExp(v_rot_robot*dt),Q_robot);
    v_rot_robot = v_rot_robot + dv_rot_robot*dt;
    
    Qg = quatProd(quatExp(dg*dt),Qg);

end
toc

log_data.u_data = can_sys_ptr.get_shapeVar(log_data.x_data);

save data/dmp_results.mat log_data cmd_args;
    

%% Find mean square error between the signals
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

offline_train_o_mse
% online_train_o_mse
sim_mse
sim_mse_dtw

