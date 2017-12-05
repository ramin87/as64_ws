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
[yd_data, dyd_data, ddyd_data] = process_data(data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);
[Qd_data, v_rot_d_data, dv_rot_d_data] = process_Q_data(Q_data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of training points

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

dmp = cell(D,1);
for i=1:D
    
    if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
       dmp{i} = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
        dmp{i} = DMP_bio(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
        dmp{i} = DMP_plus(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
        dmp{i}.k_trunc_kernel = cmd_args.k_trunc_kernel;
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-Shannon'))
        dmp{i} = DMP_Shannon(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
        dmp{i}.Wmin = cmd_args.Wmin;
        dmp{i}.Freq_min = cmd_args.Freq_min;
        use_DMP_SHANNON = true;
    else
        error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
    end
    
end

dmpo = DMPo(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);

F_offline_train_data = [];
Fd_offline_train_data = [];
Time_offline_train = [];
offline_train_mse = [];
online_train_mse = [];

F_o_offline_train_data = [];
Fd_o_offline_train_data = [];
offline_train_o_mse = [];
online_train_o_mse = [];
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

        dmp{i}.set_training_params(cmd_args.USE_GOAL_FILT, cmd_args.a_g, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(T, yd, dyd, ddyd, y0, g0, cmd_args.train_method);      

        F_offline_train_data = [F_offline_train_data; F_train];
        Fd_offline_train_data = [Fd_offline_train_data; Fd_train];

    end
    Time_offline_train = (0:(size(F_offline_train_data,2)-1))*Ts;
    
    Q0 = Q_data(:,1);
    Qg0 = Q_data(:,end);
    dmpo.set_training_params(cmd_args.USE_GOAL_FILT, cmd_args.a_g, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
    [offline_train_o_mse, F_o_offline_train_data, Fd_o_offline_train_data] = dmpo.train(T, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg0, cmd_args.train_method);   
    
    toc
end


%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g0 = cmd_args.goal_scale*yd_data(:,end); 
g = g0; 
if (cmd_args.USE_GOAL_FILT), g = y0; end
dg = zeros(D,1);
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

ddy = zeros(D,1);
dv_rot = zeros(3,1);
dy = zeros(D,1);
v_rot = zeros(3,1);
y = y0;
Q = Q0;
t = 0;
y_robot = y0;
Q_robot = Q0;
dy_robot = zeros(D,1);
v_rot_robot = zeros(3,1);
dz = zeros(D,1);
deta = zeros(3,1);
z = zeros(D,1);
eta = zeros(3,1);
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
log_data_o = get_logData_struct();   

log_data.dmp = dmp;
log_data_o.dmp = cell(3,1);
for i=1:3, log_data_o.dmp{i} = dmpo; end

log_data.Time_demo = Time_demo;
log_data.yd_data = yd_data;
log_data.dyd_data = dyd_data;
log_data.ddyd_data = ddyd_data;

log_data_o.Time_demo = Time_demo;
log_data_o.yd_data = quat2Vel(repmat(Qd_data(:,end),1,size(Qd_data,2)), Qd_data);
log_data_o.dyd_data = v_rot_d_data;
log_data_o.ddyd_data = dv_rot_d_data;

log_data.D = D;
log_data.Ts = Ts;
log_data.g0 = g0;

log_data_o.D = 3;
log_data_o.Ts = Ts;
log_data_o.g0 = quat2Vel(Qg0, Qg0);

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_offline_train_data;
log_data.Fd_offline_train_data = Fd_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data_o.Time_offline_train = Time_offline_train;
log_data_o.F_offline_train_data = F_o_offline_train_data;
log_data_o.Fd_offline_train_data = Fd_o_offline_train_data;

log_data_o.Time_online_train = [];
log_data_o.F_online_train_data = [];
log_data_o.Fd_online_train_data = [];

log_data.Psi_data = cell(D,1);
log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);
log_data.shape_attr_data = [];
log_data.goal_attr_data = [];


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
    log_data.u_data = [log_data.u_data u];
    
    log_data.y_robot_data = [log_data.y_robot_data y_robot];
    log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data g];
    
    log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];
    
    
    
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

    %% Position DMP
    
    for i=1:D
        
        if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
            
            log_data.DMP_w{i} = [log_data.DMP_w{i} dmp{i}.w];
            log_data.P_lwr{i} = [log_data.P_lwr{i} P_lwr{i}];
            
            yd = yd_data(i,iters+1);
            dyd = dyd_data(i,iters+1);
            ddyd = ddyd_data(i,iters+1);
            P_lwr{i} = dmp{i}.update_weights(x, u, yd, dyd, ddyd, y0(i), g0(i), g(i), P_lwr{i}, cmd_args.RLWR_lambda); 

            F(i) = dmp{i}.calc_Fd(y(i), dy(i), ddy(i), u, y0(i), g0(i), g(i));
            Fd(i) = dmp{i}.calc_Fd(yd, dyd, ddyd, u, y0(i), g0(i), g(i));
            
        end
        
        Psi = dmp{i}.activation_function(x);
        log_data.Psi_data{i} = [log_data.Psi_data{i} Psi(:)];

        shape_attr(i) = dmp{i}.shape_attractor(x,u,g0(i),y0(i));
        goal_attr(i) = dmp{i}.goal_attractor(y(i),dy(i),g(i));
        scaled_forcing_term(i) = dmp{i}.forcing_term(x)*dmp{i}.forcing_term_scaling(u, y0(i), g0(i));
        
        y_c = cmd_args.a_py*(y_robot(i)-y(i));
        z_c = 0;
        
        [dy(i), dz(i)] = dmp{i}.get_states_dot(y(i), z(i), x, u, y0(i), g0(i), g(i), y_c, z_c);
        
        dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd; 
    end
    
    if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
        log_data.Time_online_train = [log_data.Time_online_train t];
        log_data.F_online_train_data = [log_data.F_online_train_data F];
        log_data.Fd_online_train_data = [log_data.Fd_online_train_data Fd];
    end
    
    
    if (cmd_args.USE_GOAL_FILT)
        dg = -cmd_args.a_g*(g-g0)/can_sys_ptr.tau;
    else
        dg = zeros(size(g));
    end
    
    %% Orientation DMP
    Q_c = cmd_args.a_py*quatLog(quatProd(Q_robot,quatInv(Q)));
    eta_c = 0;
    [dQ, deta] = dmpo.get_states_dot(Q, eta, x, u, Q0, Qg0, Qg, Q_c, eta_c);
    v_rot_temp = 2*quatProd(dQ,quatInv(Q));
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
        stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        du = du*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    %% Stopping criteria
    err_p = max(abs(g0-y_robot));
    err_o = norm(quatLog(quatProd(Qg0,quatInv(Q_robot))));
    if (err_p <= cmd_args.tol_stop  &&  err_o < cmd_args.orient_tol_stop), break; end
    t
    err_p
%     err_o

%     if (err_p<0.03)
%         dy
%         dz
%         scaled_forcing_term
%         pause
%     end
    
    iters = iters + 1;
    if (iters >= cmd_args.max_iters), break; end
    
    %% Numerical integration
    t = t + dt;
    
    y = y + dy*dt;
    
    Q = quatProd(quatExp(v_rot*dt), Q);
    
    z = z + dz*dt;
    
    eta = eta + deta*dt;
    
    g = g + dg*dt;
    
    x = x + dx*dt;
    % allow the time to evolve
    %if (x<0), x=0; end % zero crossing can occur due to numberical integration
    
%     if (USE_2nd_order_can_sys)
%         u = u + du*dt;
%     else
%         u = x;
%     end
    u = u + du*dt;
    if (u<0), u=0; end
    
    y_robot = y_robot + dy_robot*dt;
    Q_robot = quatProd(quatExp(v_rot_robot*dt),Q_robot);

end
toc

save data/dmp_results.mat log_data log_data_o cmd_args;
    

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

y_data = log_data.y_data;
yd_data = log_data.yd_data;

dist_f = @(s1,s2) norm(s1-s2);
dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data, yd_data, dtw_win, dist_f);
sim_mse = sum(C)/length(C);

offline_train_mse
online_train_mse
sim_mse
dtw_dist/length(C)

% c = dmp{1}.c'
% h = dmp{1}.h'
% w = dmp{1}.w;
% 
% fprintf('%.6f\n',w)
