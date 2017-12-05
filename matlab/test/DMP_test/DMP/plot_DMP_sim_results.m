function plot_DMP_sim_results()

% clc; 
close all;
clear;

load data/dmp_results.mat log_data log_data_o

plot_DMP_sim_results_helper(log_data);

% plot_DMP_sim_results_helper(log_data_o);

end

function plot_DMP_sim_results_helper(log_data)

fontsize = 14;

Time_demo = log_data.Time_demo;
yd_data = log_data.yd_data;
dyd_data = log_data.dyd_data;
ddyd_data = log_data.ddyd_data;

D = log_data.D;
Ts = log_data.Ts;
g0 = log_data.g0;

Time_offline_train = log_data.Time_offline_train;
F_offline_train_data = log_data.F_offline_train_data;
Fd_offline_train_data = log_data.Fd_offline_train_data;
Time_online_train = log_data.Time_online_train;
F_online_train_data = log_data.F_online_train_data;
Fd_online_train_data = log_data.Fd_online_train_data;

Time = log_data.Time;
y_data = log_data.y_data;
dy_data = log_data.dy_data;
z_data = log_data.z_data;
dz_data = log_data.dz_data;
x_data = log_data.x_data;
u_data = log_data.u_data;

y_robot_data = log_data.y_robot_data;
dy_robot_data = log_data.dy_robot_data;

Fdist_data = log_data.Fdist_data;

Force_term_data = log_data.Force_term_data;
g_data = log_data.g_data;

Psi_data = log_data.Psi_data;
shape_attr_data = log_data.shape_attr_data;
goal_attr_data = log_data.goal_attr_data;

dmp = log_data.dmp;
P_lwr = log_data.P_lwr;
DMP_w = log_data.DMP_w;

e_track_data = y_data - y_robot_data;


OFFLINE_DMP_TRAINING_enable = ~isempty(F_offline_train_data);
ONLINE_DMP_UPDATE_enable = ~isempty(F_online_train_data);
USE_GOAL_FILT = g_data(1,1)~=g_data(1,end);

%% ========   Plot results  ========
disp('Ploting results...')
tic

% for i=1:D
%     X = dmp{i}.can_sys_ptr.get_continuous_output(Time_offline_train, 1);
%     x = X(1,:);
%     
%     Psi_train_data = dmp{i}.activation_function(x);
% 
%     N_kernels = size(Psi_train_data,1);
%     figure;
%     hold on;
%     for k=1:N_kernels
%         plot((Time_offline_train),(Psi_train_data(k,:)));
%         axis tight;
%     end
%     plot((Time_offline_train),(F_offline_train_data(i,:)/max(abs(F_offline_train_data(i,:)))),'LineWidth',1.5);
%     plot((Time_offline_train),(Fd_offline_train_data(i,:)/max(abs(Fd_offline_train_data(i,:)))),'LineWidth',3);
%     title('Psi activations','Interpreter','latex','fontsize',fontsize);
%     hold off;
% end


%% Plot the training data
plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data);


for i=1:D
    figure;
    subplot(2,1,1);
    plot(Time,y_data(i,:)-g0(i));
    legend({'$e_{DMP}=p_{DMP}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
    subplot(2,1,2);
    plot(Time,y_robot_data(i,:)-g0(i));
    legend({'$e_{robot}=p_{robot}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);


    n_splots = 3;
    figure;
    subplot(n_splots,1,1);
    plot(Time,x_data, Time,u_data);
    legend({'$x$','$u$'},'Interpreter','latex','fontsize',fontsize);
    subplot(n_splots,1,2);
    plot(Time,y_robot_data(i,:), Time,y_data(i,:), Time_demo,yd_data(i,:), Time,g_data(i,:), '--', Time(end),g0(i),'r*','Markersize',10);
    legend({'$p_{robot}$','$p_{DMP}$','$p_{train}$','goal evolution','$p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
    subplot(n_splots,1,3);
    plot(Time,Fdist_data);
    legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);

    n_splots = 4;
    figure;
    subplot(n_splots,1,1);
    plot(Time,Fdist_data);
    legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
    subplot(n_splots,1,2);
    plot(Time,Force_term_data);
    legend({'Forcing term'},'Interpreter','latex','fontsize',fontsize);
    subplot(n_splots,1,3);
    plot(Time,e_track_data(i,:));
    legend({'$e_{track}$'},'Interpreter','latex','fontsize',fontsize);
    subplot(n_splots,1,4);
    plot(Time,dy_robot_data(i,:), Time, dy_data(i,:), Time, z_data(i,:));
    legend({'$\dot{y}_{robot}$','$\dot{y}_{DMP}$','$z_{DMP}$'},'Interpreter','latex','fontsize',fontsize);

end


%% Plot 'F' training
if (OFFLINE_DMP_TRAINING_enable)
    for i=1:D
        F = F_offline_train_data(i,:);
        Fd = Fd_offline_train_data(i,:);
        scale = 1/max(abs([F Fd]));
        x_data_train = dmp{i}.can_sys_ptr.get_continuous_output(Time_offline_train);
        x_data_train = x_data_train(1,:);
        Psi = [];
        for j=1:length(x_data_train)
            Psi = [Psi dmp{i}.activation_function(x_data_train(j))];
        end

        figure;
        subplot(2,2,1);
        plot(Time_offline_train,F,Time_offline_train,Fd, 'LineWidth',1.2);
        legend({'$F$','$F_d$'},'Interpreter','latex','fontsize',fontsize);
        title('Off-line training','Interpreter','latex','fontsize',fontsize);
        axis tight;
        subplot(2,2,2);
        plot(Time_offline_train,F_offline_train_data(i,:)-Fd_offline_train_data(i,:), 'r', 'LineWidth',1.2);
        legend({'$F-F_d$'},'Interpreter','latex','fontsize',fontsize);
        axis tight;
        subplot(2,2,[3 4]);
        hold on;
        plot(Time_offline_train,F*scale, Time_offline_train,Fd*scale, 'LineStyle', '--', 'LineWidth',1.2);
        for k=1:size(Psi,1)
            plot(Time_offline_train,Psi(k,:));
        end
        axis tight;
        hold off;

    end
end

%% Plot 'F' online training
if (ONLINE_DMP_UPDATE_enable)
    for i=1:D
        F = F_online_train_data(i,:);
        Fd = Fd_online_train_data(i,:);
        scale = 1/max(abs([F Fd]));
        x_data_train = dmp{i}.can_sys_ptr.get_continuous_output(Time_online_train, 1);
        x_data_train = x_data_train(1,:);
        Psi = [];
        for j=1:length(x_data_train)
            Psi = [Psi dmp{i}.activation_function(x_data_train(j))];
        end

        figure;
        subplot(2,2,1);
        plot(Time_online_train,F,Time_online_train,Fd, 'LineWidth',1.2);
        legend({'$F$','$F_d$'},'Interpreter','latex','fontsize',fontsize);
        title('On-line training','Interpreter','latex','fontsize',fontsize);
        axis tight;
        subplot(2,2,2);
        plot(Time_online_train,F_online_train_data(i,:)-Fd_online_train_data(i,:), 'r', 'LineWidth',1.2);
        legend({'$F-F_d$'},'Interpreter','latex','fontsize',fontsize);
        axis tight;
        subplot(2,2,[3 4]);
        hold on;
        plot(Time_online_train,F*scale, Time_online_train,Fd*scale, 'LineStyle', '--', 'LineWidth',1.2);
        for k=1:size(Psi,1)
            plot(Time_online_train,Psi(k,:));
        end
        axis tight;
        hold off;

    end
end

%% Plot DMP RLWR cov 'P' evoultion
if (ONLINE_DMP_UPDATE_enable)
   for i=1:D
       figure;
       plot(Time_online_train,P_lwr{i}');
       title('DMP RLWR cov $P$ evoultion during on-line training','Interpreter','latex','fontsize',fontsize);
   end
end
    
%% Plot DMP weights evoultion
if (ONLINE_DMP_UPDATE_enable)
   for i=1:D
       figure;
       plot(Time_online_train,DMP_w{i}');
       title('DMP weights evolution during on-line training','Interpreter','latex','fontsize',fontsize);
   end
end


%% Plot phase variable evolution
% figure;
% plot(Time,x_data, Time, u_data);
% legend({'x','u'},'Interpreter','latex','fontsize',fontsize);
% title('phase variable evolution','Interpreter','latex','fontsize',fontsize);
% axis tight;


%% Plot goal evolution
if (USE_GOAL_FILT)
    figure;
    for i=1:D
        subplot(D,1,i);
        plot(Time,g_data(i,:));
        if (i==1), title('Goal evolution','Interpreter','latex','fontsize',fontsize); end
        axis tight;
    end
end
    
lineWidth = 1.2;
% for i=1:D
%    figure;
%    hold on;
%    plot(Time,shape_attr_data(i,:),'LineWidth',lineWidth);
%    plot(Time,goal_attr_data(i,:),'LineWidth',lineWidth);
%    plot(Time, shape_attr_data(i,:)+goal_attr_data(i,:), 'LineWidth',lineWidth);
%    %plot(Time, ddy_data(i,:),'LineWidth',lineWidth);
%    legend({'shape-attr','goal-attr','goal+shape'},'Interpreter','latex','fontsize',fontsize);
%    hold off;
% end

%% Plot DMP simulation and demo pos, vel, accel
lineWidth = 1.1;
plot_signals_and_errorSignal(Time,y_data, Time_demo,yd_data, 'DMP', 'demo', 'Position', lineWidth);
% plot_signals_and_errorSignal(Time,dy_data, Time_demo,dyd_data, 'DMP', 'demo', 'Velocity', lineWidth);
% plot_signals_and_errorSignal(Time,ddy_data, Time_demo,ddyd_data, 'DMP', 'demo', 'Accelaration', lineWidth);


%% Plot shape attractor
% figure;
% for i=1:D
%     subplot(D,1,i);
%     plot(Time, shape_attr_data{i});
%     if (i==1), title('Shape attractor','Interpreter','latex','fontsize',fontsize); end
% end


%% Plot psi activations with respect to phase variable
for i=1:D
    f_data = Force_term_data(i,:);
    plot_psi_activations_and_psiWeightedSum(x_data,Psi_data{i}, f_data, dmp{i}.c, dmp{i}.w);
end
% 
%% Plot psi activations with respect to time
% for i=1:D
%     figure
%     plot(Time, Psi_data{i});
%     xlabel('time [s]','Interpreter','latex','fontsize',fontsize);
% end

%% Plot 2D or 3D line path of DMP simulation and demo
if (D==2 || D==3)
    plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
end

for i=1:D
    figure;
    hold on;
    plot(Time_offline_train, Fd_offline_train_data(i,:));
    plot(Time_offline_train, F_offline_train_data(i,:));
    plot(Time, Force_term_data(i,:));
    legend({'$F_{d_{train}}$','$F_{train}$','$F_{sim}$'},'Interpreter','latex','fontsize',fontsize);
    hold off;
end

toc

end
