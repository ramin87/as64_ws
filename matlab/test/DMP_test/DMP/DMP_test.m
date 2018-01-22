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


%% Load data
load data/data.mat data Time

yd_data = data{1};
dyd_data = data{2};
ddyd_data = data{3};
Ts = min(diff(Time));
         
D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

%% Set up DMP params
tau = Time_demo(end);

number_of_kernels = cmd_args.N_kernels
n_data

[canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp] = get_canClock_gatingFuns_DMP(cmd_args, D, tau);

F_offline_train_data = [];
Fd_offline_train_data = [];
Time_offline_train = [];
offline_train_mse = [];
online_train_mse = [];
trainParamsName = {'lambda', 'P_cov'};
trainParamsValue = {cmd_args.lambda, cmd_args.P_cov};
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

        dmp{i}.setTrainingParams(cmd_args.trainMethod, trainParamsName, trainParamsValue);
        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(T, yd, dyd, ddyd, y0, g0);      
            
        F_offline_train_data = [F_offline_train_data; F_train];
        Fd_offline_train_data = [Fd_offline_train_data; Fd_train];

    end
    Time_offline_train = (0:(size(F_offline_train_data,2)-1))*Ts;

    toc
end


number_of_kernels = dmp{1}.N_kernels
n_data

offline_train_mse


%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g0 = cmd_args.goal_scale*yd_data(:,end); 
g = g0; 
g2 = g0;
N_g_change = length(cmd_args.time_goal_change);
ind_g_chage = 1;
dg = zeros(D,1);
x = 0.0;
dx = 0.0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;
t = 0.0;
y_robot = y0;
dy_robot = zeros(D,1);
ddy_robot = zeros(D,1);
dz = zeros(D,1);
z = zeros(D,1);

P_lwr = cell(D,1);
for i=1:D
    P_lwr{i} = ones(cmd_args.N_kernels,1)*cmd_args.P_cov;
end
F = zeros(D,1);
Fd = zeros(D,1);

Fdist = zeros(D,1);

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

log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);
for i=1:D
    log_data.DMP_w{i} = dmp{i}.w;
    log_data.DMP_c{i} = dmp{i}.c;
    log_data.DMP_h{i} = dmp{i}.h;
end

tau0 = canClockPtr.getTau();
tau = cmd_args.tau_sim_scale*tau;
canClockPtr.setTau(tau);

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
    
    log_data.g_data = [log_data.g_data g];
  
    
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

            F(i) = dmp{i}.calcFd(x, y(i), dy(i), ddy(i), y0(i), g(i));
            Fd(i) = dmp{i}.calcFd(x, yd, dyd, ddyd, y0(i), g(i));
            
        end
        
        y_c = cmd_args.a_py*(y_robot(i)-y(i));
        z_c = 0;
        
        [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);
        
        ddy(i) = dz(i)/dmp{i}.get_v_scale();
        ddy_robot(i) = ddy(i) + inv(cmd_args.Md) * ( - cmd_args.Dd*(dy_robot(i) - dy(i)) - cmd_args.Kd*(y_robot(i)-y(i)) + Fdist(i) ); 
        
        
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
                g2 = cmd_args.goal_change(ind_g_chage);
                ind_g_chage = ind_g_chage + 1;
            end
        end
        
    end
    
    %% Goal filtering
    if (cmd_args.USE_GOAL_FILT)
        dg = cmd_args.a_g*(g2-g)/canClockPtr.getTau();
    else
        g = g2;
        dg = zeros(size(dg));
    end
    
    %% Update phase variable

    dx = canClockPtr.getPhaseDot(x);
    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t, D);
    end
    
     
    %% Phase stopping
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        dg = dg*stop_coeff;
        
%         canClockPtr.setTau(tau0*stop_coeff);
    end
    
    %% Stopping criteria
    err_p = max(abs(g2-y_robot));
    if (err_p <= cmd_args.tol_stop ...
        && t>=tau)
        break; 
    end
    
    iters = iters + 1;
    if (t>=tau && iters>=cmd_args.max_iters)
        warning('Iteration limit reached. Stopping simulation...\n');
        break;
    end
    
%     if (t>0.4 && t<2.2)
%         dy_robot = 0;
%         ddy_robot = 0;
%     end
    
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

[sim_mse, sim_mse_dtw] = get_sim_mse_with_temp_and_spatial_align(Time, y_data, Time_demo, yd_data);

offline_train_mse
online_train_mse
sim_mse
sim_mse_dtw


% c = dmp{1}.c'
% h = dmp{1}.h'
% w = dmp{1}.w;
% 
% fprintf('%.6f\n',w)
