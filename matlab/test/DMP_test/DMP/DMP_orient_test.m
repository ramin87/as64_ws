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

%% Load orientation data
load data/orient_data.mat Orient_data Time

Qd_data = Orient_data{1};
v_rot_d_data = Orient_data{2};
dv_rot_d_data = Orient_data{3};
Ts = min(diff(Time));
         
Do = size(v_rot_d_data,1); % dimensionality of training data
n_data = size(v_rot_d_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data

[canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp_vec] = get_canClock_gatingFuns_DMP(cmd_args, Do, tau);

dmpo = DMP_orient();
dmpo.init(dmp_vec);


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
    dmpo.setTrainingParams(cmd_args.trainMethod, trainParamsName, trainParamsValue);
    [offline_train_o_mse, F_o_offline_train_data, Fd_o_offline_train_data] = dmpo.train(Time, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg);   
    
    Time_offline_train = (0:(size(F_o_offline_train_data,2)-1))*Ts;
    
    toc
end

for i=1:Do
    N_kernels_i = dmpo.dmp{i}.N_kernels
end

%% DMP simulation
% set initial values
x = 0.0;
dx = 0.0;
Q0 = Qd_data(:,1);
Qg0 = Qd_data(:,end);
Qg = Qg0;
Qg2 = Qg;
dg_o = zeros(Do,1);
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;

v_rot = zeros(Do,1);
dv_rot = zeros(Do,1);
Q = Q0;
t = 0.0;
Q_robot = Q0;
v_rot_robot = zeros(Do,1);
dv_rot_robot = zeros(Do,1);
deta = zeros(Do,1);
eta = zeros(Do,1);

F_o = zeros(Do,1);
Fd_o = zeros(Do,1);

Fdist_o = zeros(Do,1);

log_data = get_logData_struct();   
log_data.dmp = cell(Do,1);
for i=1:Do, log_data.dmp{i} = dmpo.dmp{i}; end

log_data.Time_demo = Time_demo;
log_data.yd_data = quat2qpos(Qd_data);
log_data.dyd_data = v_rot_d_data;
log_data.ddyd_data = dv_rot_d_data;

log_data.D = Do;
log_data.Ts = Ts;
log_data.g0 = quat2qpos(Qg);

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


log_data.P_lwr = cell(Do,1);
log_data.DMP_w = cell(Do,1);

tau = cmd_args.tau_sim_scale*tau;
canClockPtr.setTau(tau);

iters = 0;

dt = cmd_args.dt;

disp('DMP simulation...')
tic
while (true)

    %% data logging
 
    log_data.Time = [log_data.Time t];
    
    log_data.y_data = [log_data.y_data quat2qpos(Q)];
    log_data.dy_data = [log_data.dy_data v_rot];   
    log_data.ddy_data = [log_data.ddy_data dv_rot];   
    log_data.z_data = [log_data.z_data eta];
    log_data.dz_data = [log_data.dz_data deta];
        
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_robot_data = [log_data.y_robot_data quat2qpos(Q_robot)];
    log_data.dy_robot_data = [log_data.dy_robot_data v_rot_robot];
    log_data.ddy_robot_data = [log_data.ddy_robot_data dv_rot_robot];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist_o];
    
    log_data.g_data = [log_data.g_data quat2qpos(Qg)];

    X = ones(Do,1)*x;
    %% DMP simulation

    %% Orientation DMP
    
    Q_c = cmd_args.a_py*quatLog(quatProd(Q_robot,quatInv(Q)));
    eta_c = 0;
    [dQ, deta] = dmpo.getStatesDot(X, Q, eta, Q0, Qg, Q_c, eta_c);
    v_rot_temp = 2*quatProd(dQ,quatInv(Q));
    
    v_rot = v_rot_temp(2:4);
    dv_rot = deta ./ dmpo.get_v_scale();    
    dv_rot_robot = dv_rot + inv(cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - cmd_args.Kd_o*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist_o ); 
    
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg_o = cmd_args.a_g*quatLog(quatProd(Qg2,quatInv(Qg)))/canClockPtr.getTau();
    else
        Qg = Qg2;
        dg_o = zeros(size(dg_o));
    end
    
    
    %% Goal change
    if (cmd_args.ONLINE_GOAL_CHANGE_ENABLE)
        if (ind_g_chage <= N_g_change)
            if (abs((t-cmd_args.time_goal_change(ind_g_chage))) < dt/2.0)
                Qg2 = cmd_args.orient_goal_change(:,ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Update phase variable
        
    dx = canClockPtr.getPhaseDot(x);

    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist_o = Fdist_fun(t, Do);
    end
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(quatLog(quatProd(Q_robot,quatInv(Q))))^2);
        
        dx = dx*stop_coeff;
        dg_o = dg_o*stop_coeff;
    end
    
    %% Stopping criteria
    err_o = norm(quatLog(quatProd(Qg,quatInv(Q_robot))));
    if (err_o<cmd_args.orient_tol_stop && t>=tau) break; end
    
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
    
    Qg = quatProd(quatExp(dg_o*dt),Qg);

end
toc

save data/dmp_results.mat log_data cmd_args;
    

%% Find mean square error between the signals
Time = log_data.Time;
Time_demo = log_data.Time_demo;
y_data = log_data.y_data;
yd_data = log_data.yd_data;

[sim_mse, sim_mse_dtw] = get_sim_mse_with_temp_and_spatial_align(Time, y_data, Time_demo, yd_data);

offline_train_o_mse
% online_train_o_mse
sim_mse
sim_mse_dtw

