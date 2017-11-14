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

%% Load demos and process demos
load data/data.mat data Ts

% Fs = 1/Ts;
% L = size(data,2);
% X = data;
% Y = fft(X);
% P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/2))/L;
% figure;
% plot(f,P1) ;
% title('Single-Sided Amplitude Spectrum of X(t)');
% xlabel('f (Hz)');
% ylabel('|P1(f)|');
% 
% tau = Ts*L;
% 
% cmd_args.N_kernels = round(2*tau*60);

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of training points

%% Set up DMP params
tau = (n_data-1)*Ts;

number_of_kernels = cmd_args.N_kernels
n_data

if (strcmpi(cmd_args.CAN_SYS_TYPE,'lin'))
    can_sys_ptr = LinCanonicalSystem(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'exp'))
    can_sys_ptr = ExpCanonicalSystem(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = SpringDamperCanonicalSystem(cmd_args.x_end, tau);
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',cmd_args.CAN_SYS_TYPE);
end


dmp = cell(D,1);
for i=1:D
    
    if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
       dmp{i} = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
        dmp{i} = DMP_bio(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
        dmp{i} = DMP_plus(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    else
        error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
    end
    
end


F_train_data = [];
Fd_train_data = [];
Time_train = [];
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

        dmp{i}.set_training_params(cmd_args.USE_GOAL_FILT, cmd_args.a_g, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(T, yd, dyd, ddyd, y0, g0, cmd_args.train_method);      

        F_train_data = [F_train_data; F_train];
        Fd_train_data = [Fd_train_data; Fd_train];

    end
    Time_train = (0:(size(F_train_data,2)-1))*Ts;
    
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
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;
t = 0;
y_robot = y0;
dy_robot = zeros(D,1);
dz = zeros(D,1);
z = zeros(D,1); %tau*dy + cmd_args.a_py*(y_robot-y);
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
log_data.Time_train = Time_train;
log_data.F_train_data = F_train_data;
log_data.Fd_train_data = Fd_train_data;
log_data.Time_online_train = [];
log_data.F_train_online_data = [];
log_data.Fd_train_online_data = [];
log_data.Psi_data = cell(D,1);
log_data.P_lwr = cell(D,1);
log_data.DMP_w = cell(D,1);

% for some strange reason I have to pass cell array explicitly here. In the
% initialization of the structure above, cell array are rendered as []...?
log_data.shape_attr_data = cell(D,1);
log_data.goal_attr_data = cell(D,1);


tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.set_tau(tau);

iters = 0;

if (cmd_args.ONLINE_DMP_UPDATE_enable)
    dt = Ts; % sync sim with training data
else
    dt = cmd_args.dt;
end

temp_data = [];

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
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data g];
    
    %log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    %log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];
    
    %% DMP simulation

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

        %shape_attr(i) = dmp{i}.shape_attractor(x,u,g0(i),y0(i));
        %goal_attr(i) = dmp{i}.goal_attractor(y(i),dy(i),g(i));

        scaled_forcing_term(i) = dmp{i}.forcing_term(x)*dmp{i}.forcing_term_scaling(u, y0(i), g0(i));
        
        y_c = cmd_args.a_py*(y_robot(i)-y(i));
        z_c = 0;
        
        [dy(i), dz(i)] = dmp{i}.get_states_dot(y(i), z(i), x, u, y0(i), g0(i), g(i), y_c, z_c);
        
        dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd; 
    end
    
    if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
        log_data.Time_online_train = [log_data.Time_online_train t];
        log_data.F_train_online_data = [log_data.F_train_online_data F];
        log_data.Fd_train_online_data = [log_data.Fd_train_online_data Fd];
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
    
    if (USE_2nd_order_can_sys)
        u = u + du*dt;
    else
        u = x;
    end
    
    y_robot = y_robot + dy_robot*dt;

end
toc

save data/dmp_results.mat log_data cmd_args;
    

%% Find mean square error between the signals

if (cmd_args.ONLINE_DMP_UPDATE_enable)
   online_train_mse = zeros(D,1);
   for i=1:D
       n = length(log_data.F_train_online_data(i,:));
       online_train_mse(i) = norm(log_data.F_train_online_data(i,:)-log_data.Fd_train_online_data(i,:))/n;
   end
end

if (cmd_args.OFFLINE_DMP_TRAINING_enable)
   offline_train_mse = zeros(D,1);
   for i=1:D
       n = length(log_data.F_train_data(i,:));
       offline_train_mse(i) = norm(log_data.F_train_data(i,:)-log_data.Fd_train_data(i,:))/n;
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
