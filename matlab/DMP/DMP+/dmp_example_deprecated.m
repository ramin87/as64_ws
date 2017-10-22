clc;
close all;
clear;

global a b ax N_kernels x0 ts

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();


%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data,Ts);

D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;

%% Plot the training data
plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data);

%% Set up DMP params
ax = 1.6;
x0 = 1;

N_kernels = cell(D,1);
a = cell(D,1);
b = cell(D,1);

for i=1:D
    N_kernels{i} = 60;
    a{i} = 80;
    b{i} = a{i}/4;
end

g = yd_data(:,end);
y0 = yd_data(:,1);

%% Initialize the DMP weights, centers and stds
w = cell(D,1);
c = cell(D,1);
h = cell(D,1);

for i=1:D
    w{i} = rand(N_kernels{i},1);
    c{i} = set_centers('exp', N_kernels{i}, ax);
    h{i} = set_stds(c{i});
end


t = (0:n_data-1)*Ts;
ts = t(end);

%% Train the DMP using LWR
disp('DMP training...')
tic

for i=1:D
    Fd = ddyd_data(i,:)/ts^2 - a{i}*(b{i}*(g(i)-yd_data(i,:))-dyd_data(i,:)/ts);
    
    x = x0*exp(-ax*t/ts);
    
    s = x*(g(i)-y0(i));
    s = s(:);
    
    for k=1:length(w{i})
        Psi = diag( exp(-h{i}(k)*(x-c{i}(k)).^2) );
        w{i}(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + realmin);
    end
end
toc

% set initial values
x = x0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;

dt = 0.01;%time_step;

% Variables to store the simulation data
Time = 0;
ddy_data = [];
dy_data = [];
y_data = [];
x_data = [];
Psi_data = cell(D,1);

% minimum difference between current and goal position
tol_stop = 1e-4;
ts = 1*ts;
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
        Psi = gaussian_kernel(x,c{i},h{i});
        Psi_data{i} = [Psi_data{i} Psi(:)];
        
        f(i) = shape_attractor(w{i},c{i},h{i},x);
        
        ddy(i) = ts^2 * ( a{i}*(b{i}*(g(i)-y(i))-dy(i)/ts) + f(i)*x*(g(i)-y0(i)) ); 
    end
    
    y = y + dy*dt;
    dy = dy + ddy*dt;
    
    dx = -ax*x/ts;
    x = x + dx*dt;
    
    if (max(abs(norm(g-y)))<=tol_stop), break; end
    
    max_iters = max_iters -1;
    if (max_iters <= 0), break; end
    
    err = max(abs(norm(g-y)))
    
    Time = [Time Time(end)+dt];
    
    
end
toc


dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data', yd_data', dtw_win);
y_data_dtw = y_data(:,ind_y);
yd_data_dtw = yd_data(:,ind_yd);
Time_dtw = Time(ind_y);
% sum_C = sum(C)
% dtw_dist
mean_error = sum(C)/length(C)

disp('Ploting results...')
tic

figure;
plot(Time,x_data)
title('phase variable evolution');
axis tight;


figure;
for i=1:D
    subplot(D,2,1+(i-1)*2);
    plot(Time,y_data(i,:),Time_demo,yd_data(i,:));
    legend('DMP','demo');
    axis tight;
    subplot(D,2,2+(i-1)*2);
%     yd = [yd_data(:,i); repmat(yd_data(end,i), length(y_data(:,i))- length(yd_data(:,i)),1)];
%     plot(Time,y_data(:,i)-yd,'r');
    plot(Time_dtw,y_data_dtw(i,:)-yd_data_dtw(i,:),'r');
    legend('error=DMP-demo');
    axis tight;
end

Psi_data_sum = cell(D,1);
for i=1:D
    Psi_data_sum{i} = sum(Psi_data{i},1);
end

for d=1:D
    figure;
    subplot(2,1,1);
    hold on;
    for i=1:N_kernels{d}
        plot((x_data),(Psi_data{d}(i,:)));
        axis tight;
    end
    title('Psi activations');
    set(gca,'Xdir','reverse');
    hold off;
    subplot(2,1,2);
    hold on;
    plot((x_data),(Psi_data_sum{d}));
    bar(c{d},w{d});
    title('Weighted summation');
    set(gca,'Xdir','reverse');
    xlim([0 1]);
    axis tight;
    hold off;
end

markerSize = 10;
lineWidth = 2;
if (D == 2)
    figure;
    hold on;
    plot(y_data(1,:),y_data(2,:),'b',yd_data(1,:),yd_data(2,:),'g');
    plot(yd_data(1,1), yd_data(2,1),'mo','Markersize',markerSize, 'LineWidth',lineWidth);
    plot(yd_data(1,end), yd_data(2,end),'rx','Markersize',markerSize, 'LineWidth',lineWidth);
    legend('DMP','demo','start','end');
    hold off;
elseif (D == 3)
    figure;
    hold on;
    plot3(y_data(1,:),y_data(2,:),y_data(3,:),'b',yd_data(1,:),yd_data(2,:),yd_data(3,:),'g');
    plot3(yd_data(1,1), yd_data(2,1),yd_data(3,1),'mo','Markersize',markerSize, 'LineWidth',lineWidth);
    plot3(yd_data(1,end), yd_data(2,end),yd_data(3,end),'rx','Markersize',markerSize, 'LineWidth',lineWidth);
    legend('DMP','demo','start','end');
    hold off;
end

toc


function psi = gaussian_kernel(x,c,h)

psi = exp(-h.*((x-c).^2));


end

function c = set_centers(part_type, n_centers, ax)

if (strcmpi(part_type,'linear'))
    c = ((1:n_centers)'-1)/(n_centers-1);
elseif (strcmpi(part_type,'exp'))
    if (nargin < 3), error('Not enough input arguments for exponential partition'); end
    c = exp(-ax*((1:n_centers)'-1)/(n_centers-1)); 
else
    error('Unsupported partition type %s',part_type);
end

end

function h = set_stds(c)

h = 1./(c(2:end)-c(1:end-1)).^2;
h = [h; h(end)];

end

function f = shape_attractor(w,c,h,x)

Psi = gaussian_kernel(x,c,h);

f = dot(Psi,w) / (sum(Psi)+realmin);

end






