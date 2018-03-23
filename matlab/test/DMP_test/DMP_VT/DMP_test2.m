clc;
% close all;
clear;
format compact;

global cmd_args grav

grav = 9.81;

%% ========================================================
%% initialize cmd params
cmd_args = get_cmd_args();

%% ========================================================
%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

%% ========================================================
%% Set up Reference model
refModel = RefModel(cmd_args.y0_ref, cmd_args.g_ref, cmd_args.tau_ref, cmd_args.a6, cmd_args.a7);

%% ========================================================
%% Load training data
load data/data.mat Ts Time_data Y_data dY_data ddY_data

%% ========================================================
%% Set up DMP params
number_of_kernels = cmd_args.N_kernels

%% ========================================================
%% Init canonical clock goal and shape attractor gating functions and dmp
canClockPtr = LinCanonicalClock(1.0);
shapeAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
goalAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
dmp = DMP_VT(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, cmd_args.kernelStdScaling);

%% ========================================================
%% Train the DMP
trainParamsName = {'lambda', 'P_cov'};
trainParamsValue = {cmd_args.lambda, cmd_args.P_cov};
disp('DMP training...')
tic
dmp.setTrainingParams(cmd_args.trainMethod, trainParamsName, trainParamsValue);
dmp.trainMulti(Time_data, Y_data, dY_data, ddY_data);      
toc

% dmp.w = zeros(size(dmp.w));

canClockPtr.setTau(1.0);

%% ========================================================
%% ========================================================
%% DMP simulation

%% set initial values
y0 = cmd_args.y0_ref;
g0 = cmd_args.g_ref; 
g = g0; 
x = 0.0;
dx = 0.0;
t = 0.0;

y_dmp = y0;
dy_dmp = 0.0;
ddy_dmp = 0.0;
z = 0;
dz = 0;

Fgrav = cmd_args.Mo*grav;
Fgrav_h_d = cmd_args.human_load_p*Fgrav;
Fgrav_r_d = cmd_args.robot_load_p*Fgrav;

y_robot = y0;
dy_robot = 0.0;
ddy_robot = 0.0;
u_robot = 0.0;
F_robot_ext = -Fgrav_r_d;

y_human = y0;
dy_human = 0.0;
ddy_human = 0.0;
u_human = 0.0;
F_human_ext = -Fgrav_h_d;

F_err = 0.0;

P_lwr = cmd_args.P_cov*ones(dmp.N_kernels,1);
lambda = cmd_args.lambda;

tau0 = canClockPtr.getTau();
tau = cmd_args.tau_sim_scale*tau0;
canClockPtr.setTau(tau);
dt = cmd_args.dt;

iters = 0;

% dmp.w = zeros(size(dmp.w));

log_data = struct('Time',[], ...,
                  'y_dmp_data',[],'dy_dmp_data',[],'ddy_dmp_data',[],'x_data',[], 'w_dmp_data',[], ...
                  'P_lwr_data',[], ...
                  'y_robot_data',[],'dy_robot_data',[],'ddy_robot_data',[], 'u_robot_data',[], 'F_robot_ext_data',[], ...
                  'y_ref_data',[],'dy_ref_data',[],'ddy_ref_data',[], ...
                  'y_human_data',[],'dy_human_data',[],'ddy_human_data',[], 'u_human_data',[], 'F_human_ext_data',[],  ...
                  'F_err_data',[], 'Fgrav_r_d_data',[], 'Fgrav_h_d_data',[]);
        
%% Simulation
disp('Simulation...')
tic
while (true)

    %% get the reference model trajectory
    [y_ref, dy_ref, ddy_ref] = refModel.getRef(t);
     
    %% data logging

    log_data.Time = [log_data.Time t];
    
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_dmp_data = [log_data.y_dmp_data y_dmp];
    log_data.dy_dmp_data = [log_data.dy_dmp_data dy_dmp];   
    log_data.ddy_dmp_data = [log_data.ddy_dmp_data ddy_dmp];
    
    log_data.y_robot_data = [log_data.y_robot_data y_robot];
    log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];   
    log_data.ddy_robot_data = [log_data.ddy_robot_data ddy_robot];
    
    log_data.y_human_data = [log_data.y_human_data y_human];
    log_data.dy_human_data = [log_data.dy_human_data dy_human];   
    log_data.ddy_human_data = [log_data.ddy_human_data ddy_human];
        
    log_data.y_ref_data = [log_data.y_ref_data y_ref];
    log_data.dy_ref_data = [log_data.dy_ref_data dy_ref];   
    log_data.ddy_ref_data = [log_data.ddy_ref_data ddy_ref];
    
    log_data.u_human_data = [log_data.u_human_data u_human];
    log_data.F_human_ext_data = [log_data.F_human_ext_data F_human_ext];
    
    log_data.u_robot_data = [log_data.u_robot_data u_robot];
    log_data.F_robot_ext_data = [log_data.F_robot_ext_data F_robot_ext];
    
    log_data.F_err_data = [log_data.F_err_data F_err];
    
    log_data.w_dmp_data = [log_data.w_dmp_data dmp.w];
    
    log_data.P_lwr_data = [log_data.P_lwr_data P_lwr];
    
    log_data.Fgrav_r_d_data = [log_data.Fgrav_r_d_data Fgrav_r_d];
    log_data.Fgrav_h_d_data = [log_data.Fgrav_h_d_data Fgrav_h_d];
    
    %% DMP model simulation
    Y_c = 0.0;
    Z_c = 0.0; %0.05*F_err;
    [dy_dmp, dz] = dmp.getStatesDot(x, y_dmp, z, y0, g, Y_c, Z_c);
    dx = canClockPtr.getPhaseDot(x);
    ddy_dmp = dz/dmp.get_v_scale();
    
    %% Robot and Human model simulation
    
    % desired load carried by human
    if (cmd_args.const_wh_error)
        a_h = cmd_args.human_load_p + cmd_args.human_load_p_var;
    else
        a_h = cmd_args.human_load_p + (2*rand()-1)*cmd_args.human_load_p_var;
    end
    Fgrav_h_d = Fgrav*a_h; % simulates the scenario where the human  carries a varying load because he cannot estimate well how much force he applies
    Fgrav_h = Fgrav_h_d;
    
    % desired load carried by robot
    Fgrav_r_d = cmd_args.robot_load_p*Fgrav;
    
    % Force exerted by the robot
    u_robot = cmd_args.Kr*(y_dmp-y_robot) + cmd_args.Dr*dy_dmp + cmd_args.Mr*ddy_dmp;
    
    % Force exerted by the human
    u_human = cmd_args.Kh*(y_ref-y_human) + cmd_args.Dh*dy_ref + cmd_args.Mh*ddy_ref + Fgrav_h_d;
    
%     Fc = cmd_args.Kc*(y_robot-y_human);
%     Fc = Fgrav_h - u_human + cmd_args.Kh*(y_dmp-y_human) + cmd_args.Dh*dy_dmp + cmd_args.Mh*ddy_dmp;
%     Fc = Fgrav_h - u_human + cmd_args.Kr*(y_robot-y_human) + cmd_args.Dr*(dy_robot-dy_human) + cmd_args.Mh*ddy_robot + cmd_args.Dh*dy_human;
    Fc = Fgrav_h - u_human + cmd_args.Kr*(y_dmp-y_human) + cmd_args.Dr*(dy_dmp-dy_human) + cmd_args.Mh*ddy_dmp + cmd_args.Dh*dy_human;

    % External force exerted on the human
    F_human_ext = - Fgrav_h + Fc;
    
    % External force exerted on the robot
    F_robot_ext = - (Fgrav-Fgrav_h) - Fc;
    
    % Robot model dynamics
    ddy_robot = inv(cmd_args.Mr) * ( - cmd_args.Dr*dy_robot + u_robot + 0*F_robot_ext);
    
    % Human model dynamics
    ddy_human = inv(cmd_args.Mh) * ( - cmd_args.Dh*dy_human + u_human + F_human_ext); 
    
    
    %% Force error
    %F_err = 1.0*(F_robot_ext + 2*Fgrav + (1-cmd_args.w_dmp)*Fgrav + 0*cmd_args.w_dmp*Fgrav);
    F_err = 1.0*(F_robot_ext + Fgrav_r_d);
    
    %% DMP model online adaption
    P_lwr = dmp.update_weights(x, F_err, 0.0, 0.0, P_lwr, lambda);
    
    
    %% Stopping criteria
    err_p = max(abs(g-y_robot));
    if (err_p <= cmd_args.tol_stop ...
        && t>=tau && abs(dy_robot)<0.03 && abs(dy_human)<0.03)
        break; 
    end
    
    iters = iters + 1;
    if (t>=1.2*tau || iters>=cmd_args.max_iters)
        warning('Iteration limit reached. Stopping simulation...\n');
        break;
    end

    %% Numerical integration
    t = t + dt;
    
    x = x + dx*dt;
    
    y_dmp = y_dmp + dy_dmp*dt;
    z = z + dz*dt;
    
    y_robot = y_robot + dy_robot*dt;
    dy_robot = dy_robot + ddy_robot*dt;
    
    y_human = y_human + dy_human*dt;
    dy_human = dy_human + ddy_human*dt;
    

end
toc

fontsize = 14;
lineWidth = 1.4;

figure;
subplot(3,1,1);
plot(log_data.Time, log_data.y_dmp_data, 'm-.', log_data.Time, log_data.y_robot_data, 'b-',log_data.Time, log_data.y_human_data, 'g-', log_data.Time, log_data.y_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Position', 'FontSize',fontsize, 'Interpreter','latex');
legend({'DMP','Robot','Human','Ref'}, 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;
subplot(3,1,2);
plot(log_data.Time, log_data.dy_dmp_data, 'm-.', log_data.Time, log_data.dy_robot_data, 'b-',log_data.Time, log_data.dy_human_data, 'g-', log_data.Time, log_data.dy_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Velocity', 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;
subplot(3,1,3);
plot(log_data.Time, log_data.ddy_dmp_data, 'm-.', log_data.Time, log_data.ddy_robot_data, 'b-',log_data.Time, log_data.ddy_human_data, 'g-', log_data.Time, log_data.ddy_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Acceleration', 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m/s^2$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;

figure
t0 = log_data.Time(1);
tend = log_data.Time(end);
plot(log_data.Time,log_data.F_robot_ext_data,'b-' ,log_data.Time,log_data.F_human_ext_data,'g-', log_data.Time,-log_data.Fgrav_r_d_data,'r--', log_data.Time,-log_data.Fgrav_h_d_data,'m--','LineWidth',lineWidth);
title('Forces exterted on robot and human', 'FontSize',14, 'Interpreter','latex');
legend({'$Fext_{robot}$','$Fext_{human}$','$Load_{robot}$','$Load_{human}$'}, 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$N$]', 'FontSize',fontsize, 'Interpreter','latex');
xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;


tend = log_data.Time(end);
n = size(log_data.w_dmp_data,2);
step = floor(n/7);
w_ind = 1:step:n;
if (w_ind(end) ~= n)
    w_ind = [w_ind n];
end
n = length(w_ind);
legend_str = cell(n,1);
min_dy = 1e50;
max_dy = -min_dy;
t_data = log_data.Time(w_ind);
w_data = log_data.w_dmp_data(:,w_ind);

Colors = [153 153   0;
            0 255   0;
            0 204 102;
            0 153 153;
            0  51 255;
            0   0 153;
          102   0 102;
          200   0 200;
          255   0 255;
          255  45 255;
          255 145 255;
         ]/255;

figure;
hold on;
for i=1:n
    t = t_data(i);
    dmp.w = w_data(:,i);
    
    legend_str{i} = ['t = ' num2str(t) ' s'];
    
    [Time, y, dy, ddy] = DMP_sim(dmp, dt, tend, y0, g0);
    
    dy_low = min(dy);
    dy_up = max(dy);
    if (min_dy > dy_low), min_dy=dy_low; end
    if (max_dy < dy_up), max_dy=dy_up; end

    plot(Time, dy, 'Color',Colors(i,:), 'LineWidth',1.0+i*0.15);
end
plot(log_data.Time, log_data.dy_ref_data, 'Color','red', 'LineStyle',':', 'LineWidth',2.0);
legend_str = [legend_str; 'ref'];
title('DMP velocity profile adaption', 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
legend(legend_str, 'FontSize',fontsize, 'Interpreter','latex');

for i=1:n
    t = t_data(i);
%     plot([t t], [min_dy-0.05 max_dy+0.05], 'Color',Colors(i,:), 'LineStyle','--', 'LineWidth',1.0);
    plot([t], [min_dy], 'Color',Colors(i,:), 'MarkerSize',14, 'Marker','*', 'LineStyle','--', 'LineWidth',3.0);
end

axis tight;
hold off;


figure;
plot(log_data.Time, log_data.P_lwr_data, 'LineWidth', 1.5);
title('Increamental LWR - covariance evolution', 'FontSize',fontsize, 'Interpreter','latex');
xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
