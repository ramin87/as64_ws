clc;
% close all;
clear;
format compact;

global a_ grav

grav = 9.81;

%% ========================================================
%% initialize cmd params
a_ = get_cmd_args();

a_.a_z = a_.D_h;

%% ========================================================
%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

%% ========================================================
%% Set up Reference model
refModel = RefModel(a_.y0_ref, a_.g_ref, a_.tau_ref, a_.a6, a_.a7);

%% ========================================================
%% Load training data
load data/data.mat Ts Time_data Y_data dY_data ddY_data

%% ========================================================
%% Set up DMP params
number_of_kernels = a_.N_kernels

%% ========================================================
%% Init canonical clock goal and shape attractor gating functions and dmp
canClockPtr = LinCanonicalClock(1.0);
shapeAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
goalAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
dmp = DMP_VT(a_.N_kernels, a_.a_z, a_.b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, a_.kernelStdScaling);

%% ========================================================
%% Train the DMP
trainParamsName = {'lambda', 'P_cov'};
trainParamsValue = {a_.lambda, a_.P_cov};
disp('DMP training...')
tic
dmp.setTrainingParams(a_.trainMethod, trainParamsName, trainParamsValue);
dmp.trainMulti(Time_data, Y_data, dY_data, ddY_data);      
toc

% dmp.w = zeros(size(dmp.w));

canClockPtr.setTau(1.0);

%% ========================================================
%% ========================================================
%% DMP simulation

%% set initial values
y0 = a_.y0_ref;
g0 = a_.g_ref; 
g = g0; 
x = 0.0;
dx = 0.0;
t = 0.0;

y_dmp = y0;
dy_dmp = 0.0;
ddy_dmp = 0.0;
z = 0;
dz = 0;

w_o = a_.Mo*grav;
w_h = a_.human_load_p*w_o;
w_r = a_.robot_load_p*w_o;

y_r = y0;
dy_r = 0.0;
ddy_r = 0.0;
u_r = 0.0;
F_ext_r = -w_r;

y_h = y0;
dy_h = 0.0;
ddy_h = 0.0;
u_h = 0.0;
F_ext_h = -w_h;

F_err = 0.0;

P_lwr = a_.P_cov*ones(dmp.N_kernels,1);
lambda = a_.lambda;

tau0 = canClockPtr.getTau();
tau = a_.tau_sim_scale*tau0;
canClockPtr.setTau(tau);
dt = a_.dt;

iters = 0;

% dmp.w = zeros(size(dmp.w));

log_data = struct('Time',[], ...,
                  'y_dmp_data',[],'dy_dmp_data',[],'ddy_dmp_data',[],'x_data',[], 'w_dmp_data',[], ...
                  'P_lwr_data',[], ...
                  'y_r_data',[],'dy_r_data',[],'ddy_r_data',[], 'u_r_data',[], 'F_ext_r_data',[], ...
                  'y_ref_data',[],'dy_ref_data',[],'ddy_ref_data',[], ...
                  'y_h_data',[],'dy_h_data',[],'ddy_h_data',[], 'u_h_data',[], 'F_ext_h_data',[],  ...
                  'F_err_data',[], 'w_r_data',[], 'w_h_data',[]);
        
%% Simulation
disp('Simulation...')
tic
while (true)

    %% ===========  Get refernce trajectories =================
    
    %% get the human's model reference
    [y_ref, dy_ref, ddy_ref] = refModel.getRef(t);
    
    %% get the DMP's model reference
    Y_c = 0.0;
    Z_c = 0.0;
    [dy_dmp, dz] = dmp.getStatesDot(x, y_dmp, z, y0, g, Y_c, Z_c);
    dx = canClockPtr.getPhaseDot(x);
    ddy_dmp = dz/dmp.get_v_scale();
     
    %% ===========  data logging ===========  

    log_data.Time = [log_data.Time t];
    
    log_data.x_data = [log_data.x_data x];
    
    log_data.y_dmp_data = [log_data.y_dmp_data y_dmp];
    log_data.dy_dmp_data = [log_data.dy_dmp_data dy_dmp];   
    log_data.ddy_dmp_data = [log_data.ddy_dmp_data ddy_dmp];
    
    log_data.y_r_data = [log_data.y_r_data y_r];
    log_data.dy_r_data = [log_data.dy_r_data dy_r];   
    log_data.ddy_r_data = [log_data.ddy_r_data ddy_r];
    
    log_data.y_h_data = [log_data.y_h_data y_h];
    log_data.dy_h_data = [log_data.dy_h_data dy_h];   
    log_data.ddy_h_data = [log_data.ddy_h_data ddy_h];
        
    log_data.y_ref_data = [log_data.y_ref_data y_ref];
    log_data.dy_ref_data = [log_data.dy_ref_data dy_ref];   
    log_data.ddy_ref_data = [log_data.ddy_ref_data ddy_ref];
    
    log_data.u_h_data = [log_data.u_h_data u_h];
    log_data.F_ext_h_data = [log_data.F_ext_h_data F_ext_h];
    
    log_data.u_r_data = [log_data.u_r_data u_r];
    log_data.F_ext_r_data = [log_data.F_ext_r_data F_ext_r];
    
    log_data.F_err_data = [log_data.F_err_data F_err];
    
    log_data.w_dmp_data = [log_data.w_dmp_data dmp.w];
    
    log_data.P_lwr_data = [log_data.P_lwr_data P_lwr];
    
    log_data.w_r_data = [log_data.w_r_data w_r];
    log_data.w_h_data = [log_data.w_h_data w_h];
    
    
    %% Robot and Human model simulation
    
    % desired load carried by human
    if (a_.const_wh_error)
        a_h = a_.human_load_p + a_.human_load_p_var;
    else
        a_h = a_.human_load_p + (2*rand()-1)*a_.human_load_p_var;
    end
    w_h = w_o*a_h; % simulates the scenario where the human  carries a varying load because he cannot estimate well how much force he applies
    w_h_hat = w_h;
    
    % desired load carried by robot
    w_r = a_.robot_load_p*w_o;
    
    v_r = a_.K_r*(y_dmp-y_r) + a_.D_r*dy_dmp + a_.M_r*ddy_dmp;
    
    % Force exerted by the human
    u_h = a_.K_h*(y_ref-y_h) + a_.D_h*dy_ref + a_.M_h*ddy_ref + w_h;
    
%     Fc = w_h_hat - u_h + a_.K_r*(y_dmp-y_h) + a_.D_r*(dy_dmp-dy_h) + a_.M_h*ddy_dmp + a_.D_h*dy_h;
    Fc = w_h_hat - u_h + a_.M_h*( inv(a_.M_r)*(a_.K_r*(y_dmp-y_r) + a_.D_r*(dy_dmp-dy_r)) + ddy_dmp) + a_.D_h*dy_h;

    % External force exerted on the human
    F_ext_h = - w_h_hat + Fc;
    
    % External force exerted on the robot
    F_ext_r = - (w_o-w_h_hat) - Fc;
    
    % Force exerted by the robot
    u_r = v_r - F_ext_r;
    
    % Robot model dynamics
    ddy_r = inv(a_.M_r) * ( - a_.D_r*dy_r + u_r + F_ext_r);
    
    % Human model dynamics
    ddy_h = inv(a_.M_h) * ( - a_.D_h*dy_h + u_h + F_ext_h); 
    
    
    %% Force error
    F_err = 1.0*(F_ext_r + w_r);
    
    %% DMP model online adaption
    P_lwr = dmp.update_weights(x, F_err, 0.0, 0.0, P_lwr, lambda);
    
    
    %% Stopping criteria
    err_p = max(abs(g-y_r));
    if (err_p <= a_.tol_stop ...
        && t>=tau && abs(dy_r)<0.03 && abs(dy_h)<0.03)
        break; 
    end
    
    iters = iters + 1;
    if (t>=1.2*tau || iters>=a_.max_iters)
        warning('Iteration limit reached. Stopping simulation...\n');
        break;
    end

    %% Numerical integration
    t = t + dt;
    
    x = x + dx*dt;
    
    y_dmp = y_dmp + dy_dmp*dt;
    z = z + dz*dt;
    
    y_r = y_r + dy_r*dt;
    dy_r = dy_r + ddy_r*dt;
    
    y_h = y_h + dy_h*dt;
    dy_h = dy_h + ddy_h*dt;
    

end
toc

fontsize = 14;
lineWidth = 1.4;

figure;
subplot(3,1,1);
plot(log_data.Time, log_data.y_dmp_data, 'm-.', log_data.Time, log_data.y_r_data, 'b-',log_data.Time, log_data.y_h_data, 'g-', log_data.Time, log_data.y_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Position', 'FontSize',fontsize, 'Interpreter','latex');
legend({'DMP','Robot','Human','Ref'}, 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;
subplot(3,1,2);
plot(log_data.Time, log_data.dy_dmp_data, 'm-.', log_data.Time, log_data.dy_r_data, 'b-',log_data.Time, log_data.dy_h_data, 'g-', log_data.Time, log_data.dy_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Velocity', 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;
subplot(3,1,3);
plot(log_data.Time, log_data.ddy_dmp_data, 'm-.', log_data.Time, log_data.ddy_r_data, 'b-',log_data.Time, log_data.ddy_h_data, 'g-', log_data.Time, log_data.ddy_ref_data, 'r-.', 'LineWidth',lineWidth);
title('Acceleration', 'FontSize',fontsize, 'Interpreter','latex');
ylabel('[$m/s^2$]', 'FontSize',fontsize, 'Interpreter','latex');
axis tight;

figure
t0 = log_data.Time(1);
tend = log_data.Time(end);
plot(log_data.Time,log_data.F_ext_r_data,'b-' ,log_data.Time,log_data.F_ext_h_data,'g-', log_data.Time,-log_data.w_r_data,'r--', log_data.Time,-log_data.w_h_data,'m--','LineWidth',lineWidth);
title('Forces exterted on robot and human', 'FontSize',14, 'Interpreter','latex');
legend({'$F_{ext,r}$','$F_{ext,h}$','$Load_{robot}$','$Load_{human}$'}, 'FontSize',fontsize, 'Interpreter','latex');
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
