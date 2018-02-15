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

%% Load Cartesian position data
load data/CartPos_data.mat CartPos_data Time

Yd_data = CartPos_data{1};
dYd_data = CartPos_data{2};
ddYd_data = CartPos_data{3};
Ts = min(diff(Time));

Dp = size(Yd_data,1); % dimensionality of training data
n_data = size(Yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data


[canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp_vec] = get_canClock_gatingFuns_DMP(cmd_args, Dp, tau);

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
    dmpCartPos.setTrainingParams(cmd_args.trainMethod, trainParamsName, trainParamsValue);
    [offline_train_p_mse, F_p_offline_train_data, Fd_p_offline_train_data] = dmpCartPos.train(Time, Yd_data, dYd_data, ddYd_data, Y0, Yg);   
    
    Time_offline_train = (0:(size(F_p_offline_train_data,2)-1))*Ts;
    
    toc
end

for i=1:Dp
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
dg_p = zeros(Dp,1);
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;

dY = zeros(Dp,1);
ddY = zeros(Dp,1);
Y = Y0;
t = 0.0;
Y_robot = Y0;
dY_robot = zeros(Dp,1);
ddY_robot = zeros(Dp,1);
dZ = zeros(Dp,1);
Z = zeros(Dp,1);

F_p = zeros(Dp,1);
Fd_p = zeros(Dp,1);

Fdist_p = zeros(Dp,1);

log_data = get_logData_struct();   
log_data.dmp = cell(Dp,1);
for i=1:Dp, log_data.dmp{i} = dmpCartPos.dmp{i}; end

log_data.Time_demo = Time_demo;
log_data.yd_data = Yd_data;
log_data.dyd_data = dYd_data;
log_data.ddyd_data = ddYd_data;

log_data.D = Dp;
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

log_data.P_lwr = cell(Dp,1);
log_data.DMP_w = cell(Dp,1);
log_data.DMP_c = cell(Dp,1);
log_data.DMP_h = cell(Dp,1);
for i=1:Dp
    log_data.DMP_w{i} = dmp_vec{i}.w;
    log_data.DMP_c{i} = dmp_vec{i}.c;
    log_data.DMP_h{i} = dmp_vec{i}.h;
end

tau = cmd_args.tau_sim_scale*tau;
canClockPtr.setTau(tau);

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
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist_p];
    
    log_data.g_data = [log_data.g_data Yg];
    
    X = ones(Dp,1)*x;
    
    %% DMP simulation

    %% Cartesian Position DMP
    
    Y_c = cmd_args.a_py*(Y_robot - Y);
    Z_c = zeros(Dp,1);
    [dY, dZ] = dmpCartPos.getStatesDot(X, Y, Z, Y0, Yg, Y_c, Z_c);

    ddY = dZ ./ dmpCartPos.get_v_scale();    
    ddY_robot = ddY + inv(cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot - Y) + Fdist_p ); 
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg_p = cmd_args.a_g*(Yg2 - Yg)/canClockPtr.getTau();
    else
        Yg = Yg2;
        dg_p = zeros(size(dg_p));
    end
    
    
    %% Goal change
    if (cmd_args.ONLINE_GOAL_CHANGE_ENABLE)
        if (ind_g_chage <= N_g_change)
            if (abs((t-cmd_args.time_goal_change(ind_g_chage))) < dt/2.0)
                Yg2 = cmd_args.CartPos_goal_change(:,ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Update phase variable
        
    dx = canClockPtr.getPhaseDot(x);

    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist_p = Fdist_fun(t, Dp);
    end
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(Y_robot - Y)^2);
        
        dx = dx*stop_coeff;
        dg_p = dg_p*stop_coeff;
    end
    
    %% Stopping criteria
    err_p = norm(Yg - Y_robot);
    if (err_p<cmd_args.tol_stop && t>=tau), break; end
    
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
    
    Yg = Yg + dg_p*dt;
    

end
toc

save data/dmp_results.mat log_data cmd_args;
    

%% Find mean square error between the signals
Time = log_data.Time;
Time_demo = log_data.Time_demo;
y_data = log_data.y_data;
yd_data = log_data.yd_data;

[sim_mse, sim_mse_dtw] = get_sim_mse_with_temp_and_spatial_align(Time, y_data, Time_demo, yd_data);

offline_train_p_mse
% online_train_p_mse
sim_mse
sim_mse_dtw



