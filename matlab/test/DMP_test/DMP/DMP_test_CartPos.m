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
load data/data.mat data Yd_data Yd_data Ts

% calculate numerically the 1st and 2nd derivatives
[Yd_data, dYd_data, ddYd_data] = process_data(Yd_data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(Yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(dYd_data,1); % dimensionality of training data


%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data


[canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, dmp_vec] = get_canClock_gatingFuns_DMP(cmd_args, D, tau);

dmpCartPos = DMP_CartPos();
dmpCartPos.init(dmp_vec);

Time_offline_train = [];
F_p_offline_train_data = [];
Fd_p_offline_train_data = [];
offline_train_p_mse = [];
%% Train the DMP
if (cmd_args.OFFLINE_DMP_TRAINING_enable)
    disp('DMP training...')
    tic
    
    Time = (0:n_data-1)*Ts;

    Y0 = Yd_data(:,1);
    Yg = Yd_data(:,end);
    
    trainParamsName = {'lambda', 'P_cov'};
    trainParamsValue = {cmd_args.lambda, cmd_args.P_cov};
    dmpCartPos.set_training_params(cmd_args.train_method, trainParamsName, trainParamsValue);
    [offline_train_p_mse, F_p_offline_train_data, Fd_p_offline_train_data] = dmpCartPos.train(Time, Yd_data, dYd_data, ddYd_data, Y0, Yg);   
    
    Time_offline_train = (0:(size(F_p_offline_train_data,2)-1))*Ts;
    
    toc
end

for i=1:D
    N_kernels_i = dmpCartPos.dmp{i}.N_kernels
end


%% DMP simulation
% set initial values
x = 0.0;
dx = 0.0;
Y0 = Yd_data(:,1);
Yg0 = Yd_data(:,end);
Yg = Yg0;
Yg2 = Yg;
dg = zeros(D,1);
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;

dY = zeros(D,1);
ddY = zeros(D,1);
Y = Y0;
t = 0.0;
Y_robot = Y0;
dY_robot = zeros(D,1);
ddY_robot = zeros(D,1);
dZ = zeros(D,1);
Z = zeros(D,1);
scaled_forcing_term = zeros(D,1);
shape_attr = zeros(D,1);
goal_attr = zeros(D,1);

F = zeros(D,1);
Fd = zeros(D,1);

Fdist = 0;

log_data = get_logData_struct();   
log_data.dmp = cell(D,1);
for i=1:D, log_data.dmp{i} = dmpCartPos.dmp{i}; end

log_data.Time_demo = Time_demo;
log_data.yd_data = Yd_data;
log_data.dyd_data = dYd_data;
log_data.ddyd_data = ddYd_data;

log_data.D = D;
log_data.Ts = Ts;
log_data.g0 = Yg;

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_p_offline_train_data;
log_data.Fd_offline_train_data = Fd_p_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data.Time_offline_train = Time_offline_train;
log_data.F_offline_train_data = F_p_offline_train_data;
log_data.Fd_offline_train_data = Fd_p_offline_train_data;

log_data.Time_online_train = [];
log_data.F_online_train_data = [];
log_data.Fd_online_train_data = [];

log_data.Psi_data = cell(D,1);
log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);
log_data.shape_attr_data = [];
log_data.goal_attr_data = [];

tau = cmd_args.tau_sim_scale*tau;
canClock_ptr.set_tau(tau);

iters = 0;

dt = cmd_args.dt;

disp('DMP simulation...')
tic
while (true)

    %% data logging
 
    log_data.Time = [log_data.Time t];
    
    log_data.y_data = [log_data.y_data Y];
    log_data.dy_data = [log_data.dy_data dY];   
    log_data.ddy_data = [log_data.ddy_data ddY];   
    log_data.z_data = [log_data.z_data Z];
    log_data.dz_data = [log_data.dz_data dZ];
        
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_robot_data = [log_data.y_robot_data Y_robot];
    log_data.dy_robot_data = [log_data.dy_robot_data dY_robot];
    log_data.ddy_robot_data = [log_data.ddy_robot_data ddY_robot];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data Yg];
    
    log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];

    
    X = ones(3,1)*x;
    
    %% DMP simulation

    %% Orientation DMP
    scaled_forcing_term = dmpCartPos.forcing_term(X).*dmpCartPos.forcing_term_scaling(Y0, Yg);
%     scaled_forcing_term =  dmpo.shape_attractor(x, Q0, Qg);
    
    Y_c = cmd_args.a_py*(Y_robot - Y);
    Z_c = zeros(D,1);
    [dY, dZ] = dmpCartPos.get_states_dot(X, Y, Z, Y0, Yg, Y_c, Z_c);

    ddY = dZ ./ dmpCartPos.get_v_scale();    
    ddY_robot = ddY + inv(cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot - Y) + Fdist ); 
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg = cmd_args.a_g*(Yg2 - Yg)/canClock_ptr.get_tau();
    else
        Yg = Yg2;
        dg = zeros(size(dg));
    end
    
    
    %% Goal change
    if (cmd_args.ONLINE_GOAL_CHANGE_ENABLE)
        if (ind_g_chage <= N_g_change)
            if (abs((t-cmd_args.time_goal_change(ind_g_chage))) < dt/2.0)
%                 disp('Goal change')
%                 t
                Yg2 = cmd_args.CartPos_goal_change(:,ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Update phase variable
        
    dx = canClock_ptr.get_phase_dot(x);

    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(Y_robot - Y)^2);
        
        dx = dx*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    %% Stopping criteria
    err_p = norm(Yg - Y_robot);
    if (err_p<cmd_args.orient_tol_stop && t>=tau) break; end

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
    
    Y = Y + dY*dt;
    Z = Z + dZ*dt;
    
    Y_robot = Y_robot + dY_robot*dt;
    dY_robot = dY_robot + ddY_robot*dt;
    
    Yg = Yg + dg*dt;
    

end
toc

log_data.shapeAttrGating_data = shapeAttrGating_ptr.get_output(log_data.x_data);
log_data.goalAttrGating_data = goalAttrGating_ptr.get_output(log_data.x_data);

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

offline_train_p_mse
% online_train_p_mse
sim_mse
sim_mse_dtw

