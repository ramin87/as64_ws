clc;
close all;
clear;

global a_z b_z a_x a_g D N_kernels x0 tau std_K x_end ...
       USE_GOAL_FILT ...
       USE_PHASE_STOP a_px ...
       USE_2nd_order_can_sys ...
       CAN_SYS_TYPE

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

USE_GOAL_FILT = false;

USE_PHASE_STOP = false;
a_px = 100;

USE_2nd_order_can_sys = false;
CAN_SYS_TYPE = 'lin'; % 'lin', exp', 'spring-damper'

%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data, Ts, 0.01, 0.03);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Plot the training data
% plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data);


%% Set up DMP params
tau = (n_data-1)*Ts;
a_x = -log(0.05);
x_end = 0.05;
x0 = 1;

D = size(yd_data,1); % dimensionality of training data
N_kernels = 100;
a_z = 80;
b_z = a_z/4;
std_K = 1.11;
a_g = 60;

N_kernels

if (strcmpi(CAN_SYS_TYPE,'lin'))
    can_sys_ptr = DMP_lin_canonical_system(x_end, tau, USE_PHASE_STOP, a_px);
elseif (strcmpi(CAN_SYS_TYPE,'exp'))
    can_sys_ptr = DMP_exp_canonical_system(x_end, tau, USE_PHASE_STOP, a_px);
elseif (strcmpi(CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = DMP_spring_damper_canonical_system(x_end, tau, USE_PHASE_STOP, a_px);
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',CAN_SYS_TYPE);
end


dmp = cell(D,1);
for i=1:D
    dmp{i} = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K);
    
    %dmp{i}.set_centers('lin');
    %dmp{i}.set_centers('exp',a_x);
    %dmp{i}.set_stds(std_K);
    %dmp{i}.set_kernels_with_EM(Time_demo, yd_data(i,:));
end


%% Train the DMP
disp('DMP training...')
tic
train_method = 'LWR'; % train the DMP using LWR (Locally Weighted Regression)
%train_method = 'LS'; % train the DMP using LS (Least Squares)
train_mse = zeros(D,1);
for i=1:D
    [train_mse(i), F, Fd] = dmp{i}.train(yd_data(i,:), dyd_data(i,:), ddyd_data(i,:), Ts, train_method);

    t = (0:(length(F)-1))*Ts;
    figure;
    subplot(2,1,1);
    plot(t,F,t,Fd);
    legend('F','F_d');
    subplot(2,1,2);
    plot(t,F-Fd);
    legend('F-F_d');
          
end
toc

%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g = yd_data(:,end); 
g0 = g;
if (USE_GOAL_FILT), g = y0; end
x0 = 1;
x = x0;
dx = 0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;

dt = 0.002; %simulation time_step;

% Variables to store the simulation data
Time = 0;
ddy_data = [];
dy_data = [];
y_data = [];
x_data = [];
dx_data = [];
g_data = cell(D,1);
Psi_data = cell(D,1);
f_data = cell(D,1);
shape_attr_data = cell(D,1);
goal_attr_data = cell(D,1);

% minimum difference between current and goal position
tol_stop = 1e-3;
tau = 1*tau;
can_sys_ptr.tau = tau;
max_iters = 8000;

disp('DMP simulation...')
tic
while (true)

    for i=1:D
        
        Psi = dmp{i}.activation_function(x);
        Psi_data{i} = [Psi_data{i} Psi(:)];
        
        f = dmp{i}.forcing_term(x);
        shape_attr = dmp{i}.shape_attractor(x(i),g(i),y0(i))*can_sys_ptr.tau^2;
        goal_attr = dmp{i}.goal_attractor(y(i),dy(i),g(i))*can_sys_ptr.tau^2;
        
        f_data{i} = [f_data{i} f];
        shape_attr_data{i} = [shape_attr_data{i} shape_attr];
        goal_attr_data{i} = [goal_attr_data{i} goal_attr];
        g_data{i} = [g_data{i} g(i)];
        
        % Get DMP output, i.e. the accelaration
        %ddy(i) = dmp.ts^2 * ( dmp.a{i}*(dmp.b{i}*(g(i)-y(i))-dy(i)/dmp.ts) + f(i)*x*(g(i)-y0(i)) ); 
        ddy(i) = dmp{i}.get_output(y(i),dy(i),g(i),y0(i),x); 
      
    end
    
    y = y + dy*dt;
    dy = dy + ddy*dt;
    
    X_in = x;
    if (USE_2nd_order_can_sys)
        X_in = [X_in; dx];
    end
        
    X_out = can_sys_ptr.get_single_step_output(dt, X_in);
    
    x = X_out(1);
    if (length(X_out) > 1)
        dx = X_out(2);
    end
    
    %dx = -a_x*x/tau;
    %x = x + dx*dt;
    
    if (USE_GOAL_FILT)
        dg = -a_g*(g-g0)/tau;
        g = g + dg*dt;
    end

    %% data logging
    ddy_data = [ddy_data ddy];
    dy_data = [dy_data dy];
    y_data = [y_data y];
    x_data = [x_data x];
    
    if (max(abs(g0-y))<=tol_stop), break; end
    
    max_iters = max_iters -1;
    if (max_iters <= 0), break; end
    
    %err = max(abs(norm(g-y)))
    
    Time = [Time Time(end)+dt];
    
end
toc


%% Find mean square error between the signals

dist_f = @(s1,s2) norm(s1-s2);
dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data', yd_data', dtw_win, dist_f);
sim_mse = sum(C)/length(C);



%% Plot results
disp('Ploting results...')
tic

% figure;
% plot(Time,x_data)
% title('phase variable evolution');
% axis tight;

if (USE_GOAL_FILT)
    figure;
    for i=1:D
        subplot(D,1,i);
        plot(Time,g_data{i});
        if (i==1), title('Goal evolution'); end
        axis tight;
    end
end
    
% lineWidth = 1.2;
% for i=1:D
%    figure;
%    hold on;
%    plot(Time,shape_attr_data{i},'LineWidth',lineWidth);
%    plot(Time,goal_attr_data{i},'LineWidth',lineWidth);
%    plot(Time, shape_attr_data{i}+goal_attr_data{i}, 'LineWidth',lineWidth);
%    plot(Time, ddy_data(i,:),'LineWidth',lineWidth);
%    legend('shape-attr','goal-attr','goal+shape','ddy');
%    hold off;
% end

plot_signals_and_errorSignal(Time,y_data, Time_demo,yd_data, 'DMP', 'demo', 1.1);

for i=1:D
    plot_psi_activations_and_psiWeightedSum(x_data,Psi_data{i}, f_data{i}, dmp{i}.c, dmp{i}.w);
end

for i=1:D
    figure
    plot(Time, Psi_data{i});
    xlabel('time [s]');
end

if (D==2 || D==3)
    plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
end

toc

train_mse
sim_mse
dtw_dist/length(C)