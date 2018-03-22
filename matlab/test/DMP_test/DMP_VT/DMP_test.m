clc;
% close all;
clear;
format compact;

global cmd_args

%% initialize cmd params
cmd_args = get_cmd_args();

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

%% Load data
load data/data.mat Ts Time_data Y_data dY_data ddY_data

human = HumanModel(cmd_args.y0, cmd_args.g, cmd_args.tau, cmd_args.a6, cmd_args.a7, cmd_args.Kh, cmd_args.Dh);

%% Set up DMP params
number_of_kernels = cmd_args.N_kernels
% n_data

%% ========================================================
%% Init canonical clock goal and shape attractor gating functions and dmp
canClockPtr = LinCanonicalClock(1.0);
shapeAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
goalAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
dmp = DMP_VT(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, cmd_args.kernelStdScaling);


trainParamsName = {'lambda', 'P_cov'};
trainParamsValue = {cmd_args.lambda, cmd_args.P_cov};
%% Train the DMP
disp('DMP training...')
tic
dmp.setTrainingParams(cmd_args.trainMethod, trainParamsName, trainParamsValue);
dmp.trainMulti(Time_data, Y_data, dY_data, ddY_data);      
toc

%% DMP simulation
% set initial values
y0 = cmd_args.y0;
g0 = cmd_args.g; 
g = g0; 
x = 0.0;
dx = 0.0;
ddy = 0.0;
dy = 0.0;
y = y0;
t = 0.0;
y_robot = y0;
dy_robot = 0.0;
ddy_robot = 0.0;
dz = 0.0;
z = 0.0;

% P_lwr = cell(D,1);
% for i=1:D
%     P_lwr{i} = ones(cmd_args.N_kernels,1)*cmd_args.P_cov;
% end
P_lwr = cmd_args.P_cov;
lambda = cmd_args.lambda;

tau0 = canClockPtr.getTau();
tau = cmd_args.tau_sim_scale*tau0;
canClockPtr.setTau(tau);
dt = cmd_args.dt;

iters = 0;

log_data = struct('Time',[],'y_data',[],'dy_data',[],'z_data',[],'dz_data',[],'x_data',[], ...
                   'yh_data',[],'dyh_data',[],'ddyh_data',[],...
                   'Fh_data',[]);

disp('DMP simulation...')
tic
while (true)

    [yh, dyh, ddyh] = getHumanRef(human, t);
    Fh = human.K*(yh-y) + human.D*(dyh-dy);
    
    %% data logging

    log_data.Time = [log_data.Time t];
    
    log_data.y_data = [log_data.y_data y];
    log_data.dy_data = [log_data.dy_data dy];   
    log_data.z_data = [log_data.z_data z];
    log_data.dz_data = [log_data.dz_data dz];
        
    log_data.x_data = [log_data.x_data x];
    
    log_data.yh_data = [log_data.yh_data yh];
    log_data.dyh_data = [log_data.dyh_data dyh];   
    log_data.ddyh_data = [log_data.ddyh_data ddyh];
    
    log_data.Fh_data = [log_data.Fh_data Fh];
    
%     log_data.y_robot_data = [log_data.y_robot_data y_robot];
%     log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];
%     log_data.ddy_robot_data = [log_data.ddy_robot_data ddy_robot];

    Y_c = 0.0;
    Z_c = 0; %40*Fh;
    
    %% DMP simulation
    [dy, dz] = dmp.getStatesDot(x, y, z, y0, g, Y_c, Z_c);

    ddy = dz/dmp.get_v_scale();
    ddy_robot = ddy + inv(cmd_args.Md) * ( - cmd_args.Dd*(dy_robot - dy) - cmd_args.Kd*(y_robot-y) + 0 ); 
    
    %% Update phase variable
    dx = canClockPtr.getPhaseDot(x);

    
    %% Stopping criteria
    err_p = max(abs(g-y_robot));
    if (err_p <= cmd_args.tol_stop ...
        && t>=tau)
        break; 
    end
    
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
    
    P_lwr = dmp.update_weights(x, cmd_args.K_f_err*Fh, 0.0, 0.0, P_lwr, lambda);

end
toc

figure;
subplot(4,1,1);
plot(log_data.Time, log_data.y_data, 'b-', log_data.Time, log_data.yh_data, 'g--', 'LineWidth',2);
legend({'DMP','Human'}, 'FontSize',14, 'Interpreter','latex');
ylabel('[$m$]', 'FontSize',14, 'Interpreter','latex');
axis tight;
subplot(4,1,2);
plot(log_data.Time, log_data.z_data, 'b-', log_data.Time, log_data.dyh_data, 'g--', 'LineWidth',2);
ylabel('[$m/s$]', 'FontSize',14, 'Interpreter','latex');
axis tight;
subplot(4,1,3);
plot(log_data.Time, log_data.dz_data, 'b-', log_data.Time, log_data.ddyh_data, 'g--', 'LineWidth',2);
ylabel('[$m/s^2$]', 'FontSize',14, 'Interpreter','latex');
axis tight;
subplot(4,1,4);
plot(log_data.Time, log_data.Fh_data, 'LineWidth',2);
title('Force exerted by human', 'FontSize',14, 'Interpreter','latex');
xlabel('time [$s$]', 'FontSize',14, 'Interpreter','latex');
axis tight;
