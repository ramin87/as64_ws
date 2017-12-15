clc;
close all;
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

USE_2nd_order_can_sys = false;
use_DMP_SHANNON = false;

%% Load demos and process demos
load data/data.mat data Q_data Ts

% calculate numerically the 1st and 2nd derivatives
[Qd_data, v_rot_d_data, dv_rot_d_data] = process_Q_data(Q_data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(Qd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(v_rot_d_data,1); % dimensionality of training data

%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data


if (strcmpi(cmd_args.CAN_SYS_TYPE,'lin'))
    can_sys_ptr = LinCanonicalSystem();
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'exp'))
    can_sys_ptr = ExpCanonicalSystem();
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = SpringDamperCanonicalSystem();
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',cmd_args.CAN_SYS_TYPE);
end

can_sys_ptr.init(cmd_args.x_end, tau, cmd_args.x0);

% dmp = cell(D,1);
% for i=1:D
%     
%     if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
%        dmp{i} = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
%     elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
%         dmp{i} = DMP_bio(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
%     elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
%         dmp{i} = DMP_plus(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
%         dmp{i}.k_trunc_kernel = cmd_args.k_trunc_kernel;
%     elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-Shannon'))
%         dmp{i} = DMP_Shannon(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
%         dmp{i}.Wmin = cmd_args.Wmin;
%         dmp{i}.Freq_min = cmd_args.Freq_min;
%         use_DMP_SHANNON = true;
%     else
%         error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
%     end
%     
% end

dmpo = DMPo(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);


Time_offline_train = [];
F_o_offline_train_data = [];
Fd_o_offline_train_data = [];
offline_train_o_mse = [];
%% Train the DMP
if (cmd_args.OFFLINE_DMP_TRAINING_enable)
    disp('DMP training...')
    tic
    
    Time = (0:n_data-1)*Ts;

    Q0 = Q_data(:,1);
    Qg0 = Q_data(:,end);
    dmpo.set_training_params(cmd_args.USE_GOAL_FILT, cmd_args.a_g, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
    [offline_train_o_mse, F_o_offline_train_data, Fd_o_offline_train_data] = dmpo.train(Time, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg0, cmd_args.train_method);   
    
    Time_offline_train = (0:(size(F_o_offline_train_data,2)-1))*Ts;
    
    toc
end


%% DMP simulation
% set initial values
x = cmd_args.x0;
dx = 0;
if (USE_2nd_order_can_sys)
    u = 0;
else
    u = x;
end
du = 0;

Q0 = Q_data(:,1);
Qg0 = Q_data(:,end);
Qg = Qg0;
% if (cmd_args.USE_GOAL_FILT), Qg = Q0; end

dv_rot = zeros(3,1);
v_rot = zeros(3,1);
Q = Q0;
t = 0;
Q_robot = Q0;
v_rot_robot = zeros(3,1);
deta = zeros(3,1);
eta = zeros(3,1);
scaled_forcing_term = zeros(D,1);
shape_attr = zeros(D,1);
goal_attr = zeros(D,1);

F = zeros(D,1);
Fd = zeros(D,1);

Fdist = 0;

log_data = get_logData_struct(); 
log_data_o = get_logData_struct();   
log_data_o.dmp = cell(3,1);
for i=1:3, log_data_o.dmp{i} = dmpo; end

log_data_o.Time_demo = Time_demo;
log_data_o.yd_data = quat2Vel(repmat(Qd_data(:,end),1,size(Qd_data,2)), Qd_data);
log_data_o.dyd_data = v_rot_d_data;
log_data_o.ddyd_data = dv_rot_d_data;

log_data_o.D = 3;
log_data_o.Ts = Ts;
log_data_o.g0 = quat2Vel(Qg0, Qg0);

log_data_o.Time_offline_train = Time_offline_train;
log_data_o.F_offline_train_data = F_o_offline_train_data;
log_data_o.Fd_offline_train_data = Fd_o_offline_train_data;

log_data_o.Time_online_train = [];
log_data_o.F_online_train_data = [];
log_data_o.Fd_online_train_data = [];

log_data_o.Time_offline_train = Time_offline_train;
log_data_o.F_offline_train_data = F_o_offline_train_data;
log_data_o.Fd_offline_train_data = Fd_o_offline_train_data;

log_data_o.Time_online_train = [];
log_data_o.F_online_train_data = [];
log_data_o.Fd_online_train_data = [];

log_data_o.Psi_data = cell(D,1);
log_data_o.P_lwr = cell(D,1);
log_data_o.DMP_w = cell(D,1);
log_data_o.shape_attr_data = [];
log_data_o.goal_attr_data = [];

tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.set_tau(tau);

iters = 0;

dt = cmd_args.dt;


ind = zeros(1, size(Q_data,2));
for i=1:size(Q_data,2)
    ind(i) = (abs(norm(Q_data(:,i))-1) > 1e-15);
    if (ind(i))
        norm_Q_i = norm(Q_data(:,i))
    end
end
length(find(ind))

disp('DMP simulation...')
tic
while (true)

    %% data logging
 
    log_data_o.Time = [log_data_o.Time t];
    
    log_data_o.y_data = [log_data_o.y_data quat2Vel(Qg0,Q)];
    log_data_o.dy_data = [log_data_o.dy_data v_rot];   
    log_data_o.z_data = [log_data_o.z_data eta];
    log_data_o.dz_data = [log_data_o.dz_data deta];
        
    log_data_o.x_data = [log_data_o.x_data x];
    log_data_o.u_data = [log_data_o.u_data u];
    
    log_data_o.y_robot_data = [log_data_o.y_robot_data quat2Vel(Qg0,Q_robot)];
    log_data_o.dy_robot_data = [log_data_o.dy_robot_data v_rot_robot];
    
    log_data_o.Fdist_data = [log_data_o.Fdist_data Fdist];
    
    log_data_o.Force_term_data = [log_data_o.Force_term_data scaled_forcing_term];
    
    log_data_o.g_data = [log_data_o.g_data quat2Vel(Qg0,Qg)];
    
    log_data_o.shape_attr_data = [log_data_o.shape_attr_data shape_attr];
    log_data_o.goal_attr_data = [log_data_o.goal_attr_data goal_attr];

    
    %% DMP simulation

    %% Orientation DMP
    scaled_forcing_term = dmpo.forcing_term(x).*dmpo.forcing_term_scaling(u, Q0, Qg0);
    
    Q_c = cmd_args.a_py*quatLog(quatProd(Q_robot,quatInv(Q)));
    eta_c = 0;
    [dQ, deta] = dmpo.get_states_dot(Q, eta, x, u, Q0, Qg0, Qg, Q_c, eta_c);
    v_rot_temp = 2*quatProd(dQ,quatInv(Q));
    
%     if (abs(v_rot_temp(1)) > 1e-20)
%         v_rot_temp(1)
%         warning('v_rot(1) ~= 0');
%     end
%     if (abs(norm(Q)-1) > 1e-20)
%         t
%         norm_Q = norm(Q)
%         warning('norm_Q ~= 1');
%         pause
%     end
    
    v_rot = v_rot_temp(2:4);
    
    v_rot_robot = v_rot - (cmd_args.Kd_o/cmd_args.Dd_o)*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist/cmd_args.Dd;
    
    %% Update phase variable
    
    X_in = x;
    if (USE_2nd_order_can_sys)
        X_in = [X_in; u];
    end
        
    X_out = can_sys_ptr.get_derivative(X_in);
    
    dx = X_out(1);
    if (length(X_out) > 1)
        du = X_out(2);
    else
        du = dx;
    end
    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
%         stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
%         
%         dx = dx*stop_coeff;
%         du = du*stop_coeff;
%         dg = dg*stop_coeff;
    end
    
    %% Stopping criteria
    err_o = norm(quatLog(quatProd(Qg0,quatInv(Q_robot))));
    if (err_o < cmd_args.orient_tol_stop), break; end

%     t
%     err_o
    
    iters = iters + 1;
    if (iters >= cmd_args.max_iters), break; end
    
    %% Numerical integration
    t = t + dt;
    
%     y = y + dy*dt;
    
    Q = quatProd(quatExp(v_rot*dt), Q);
    
%     z = z + dz*dt;
    
    eta = eta + deta*dt;
    
%     g = g + dg*dt;
    
    x = x + dx*dt;
    
    u = u + du*dt;
    if (u<0), u=0; end
    
%     y_robot = y_robot + dy_robot*dt;
    Q_robot = quatProd(quatExp(v_rot_robot*dt),Q_robot);

end
toc

save data/dmp_results.mat log_data log_data_o cmd_args;
    

%% Find mean square error between the signals

% y_data = log_data.y_data;
% yd_data = log_data.yd_data;

% dist_f = @(s1,s2) norm(s1-s2);
% dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
% [dtw_dist, ind_y, ind_yd, C] = dtw(y_data, yd_data, dtw_win, dist_f);
% sim_mse = sum(C)/length(C);

offline_train_o_mse
% sim_mse
% dtw_dist/length(C)

% c = dmp{1}.c'
% h = dmp{1}.h'
% w = dmp{1}.w;
% 
% fprintf('%.6f\n',w)
