clc;
close all;
clear;

global a b ax D N_kernels x0 ts

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();


%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data,Ts);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Plot the training data
plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data);


%% Set up DMP params
ts = (n_data-1)*Ts;
%ax = 1.5;
ax = -log(0.08);
x0 = 1;

D = size(yd_data,1); % dimensionality of training data
N_kernels = 200;
a = 100;
b = a/4;

dmp = cell(D,1);
for i=1:D
    dmp{i} = DMP(N_kernels, a, b, ts, ax, 'exp');
end

%% Train the DMP
disp('DMP training...')
tic
train_method = 'LWR'; % train the DMP using LWR (Locally Weighted Regression)
%train_method = 'LS'; % train the DMP using LS (Least Squares)
train_mse = zeros(D,1);
for i=1:D
    train_mse(i) = dmp{i}.train(yd_data(i,:), dyd_data(i,:), ddyd_data(i,:), Ts, train_method);
end
toc


%% DMP simulation
% set initial values
g = yd_data(:,end);
y0 = yd_data(:,1);
x0 = 1;
x = x0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;

dt = 0.005; %simulation time_step;

% Variables to store the simulation data
Time = 0;
ddy_data = [];
dy_data = [];
y_data = [];
x_data = [];
Psi_data = cell(D,1);
f_data = cell(D,1);

% minimum difference between current and goal position
tol_stop = 1e-3;
ts = 1*ts;
for i=1:D
    dmp{i}.ts = ts;
end
max_iters = 1000;

disp('DMP simulation...')
tic
while (true)
    ddy_data = [ddy_data ddy];
    dy_data = [dy_data dy];
    y_data = [y_data y];
    x_data = [x_data x];

    f = zeros(D,1);
    for i=1:D
        Psi = dmp{i}.activation_function(x);
        Psi_data{i} = [Psi_data{i} Psi(:)];
        
        f(i) = dmp{i}.forcing_term(x);
        f_data{i} = [f_data{i} f(i)];
        
        %ddy(i) = dmp.ts^2 * ( dmp.a{i}*(dmp.b{i}*(g(i)-y(i))-dy(i)/dmp.ts) + f(i)*x*(g(i)-y0(i)) ); 
        %ddy(i) = dmp.ts^2 * ( dmp.goal_attractor(y(i),dy(i),g(i),i) + dmp.shape_attractor(x,g(i),y0(i),i) ); 
        ddy(i) = dmp{i}.get_output(y(i),dy(i),g(i),y0(i),x);
    end
    
    y = y + dy*dt;
    dy = dy + ddy*dt;
    
    dx = -ax*x/ts;
    x = x + dx*dt;
    
    if (max(abs(norm(g-y)))<=tol_stop), break; end
    
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

figure;
plot(Time,x_data)
title('phase variable evolution');
axis tight;

plot_signals_and_errorSignal(Time,y_data, Time_demo,yd_data, 'DMP', 'demo', 1.1);

%plot_psi_activations_and_psiWeightedSum(x_data,Psi_data, f_data, dmp.c, dmp.w);

if (D==2 || D==3)
    plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
end

toc

train_mse
sim_mse
dtw_dist/length(C)