%clc; 
%close all;
clear;

load dmp_results.mat

fontsize = cmd_args.fontsize;

dmp = log_data.dmp;
Time_demo = log_data.Time_demo;
yd_data = log_data.yd_data;
dyd_data = log_data.dyd_data;
ddyd_data = log_data.ddyd_data;
D = log_data.D;
Ts = log_data.Ts;
g0 = log_data.g0;
Time_train = log_data.Time_train;
F_train_data = log_data.F_train_data;
Fd_train_data = log_data.Fd_train_data;
Time = log_data.Time;
y_data = log_data.y_data;
dy_data = log_data.dy_data;
y_robot_data = log_data.y_robot_data;
dy_robot_data = log_data.dy_robot_data;
z_data = log_data.z_data;
dz_data = log_data.dz_data;
x_data = log_data.x_data;
u_data = log_data.u_data;
Fdist_data = log_data.Fdist_data;
Force_term_data = log_data.Force_term_data;
g_data = log_data.g_data;
Psi_data = log_data.Psi_data;
shape_attr_data = log_data.shape_attr_data;
goal_attr_data = log_data.goal_attr_data;


e_track_data = y_data - y_robot_data;

%% ========   Plot results  ========
disp('Ploting results...')
tic


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
    legend({'$\dot{y_{robot}}$','$\dot{y_{DMP}}$','$z_{DMP}$'},'Interpreter','latex','fontsize',fontsize);

end


%% Plot 'F' training
for i=1:D
    figure;
    subplot(2,1,1);
    plot(Time_train,F_train_data(i,:),Time_train,Fd_train_data(i,:));
    legend({'$F$','$F_d$'},'Interpreter','latex','fontsize',fontsize);
    subplot(2,1,2);
    plot(Time_train,F_train_data(i,:)-Fd_train_data(i,:));
    legend({'$F-F_d$'},'Interpreter','latex','fontsize',fontsize);
end


%% Plot phase variable evolution
figure;
plot(Time,x_data, Time, u_data);
legend({'x','u'},'Interpreter','latex','fontsize',fontsize);
title('phase variable evolution','Interpreter','latex','fontsize',fontsize);
axis tight;


%% Plot goal evolution
if (cmd_args.USE_GOAL_FILT)
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
%    plot(Time,shape_attr_data{i},'LineWidth',lineWidth);
%    plot(Time,goal_attr_data{i},'LineWidth',lineWidth);
%    plot(Time, shape_attr_data{i}+goal_attr_data{i}, 'LineWidth',lineWidth);
%    plot(Time, ddy_data(i,:),'LineWidth',lineWidth);
%    legend({'shape-attr','goal-attr','goal+shape','$\ddot{y}$'},'Interpreter','latex','fontsize',fontsize);
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
    plot_psi_activations_and_psiWeightedSum(x_data,Psi_data{i}, Force_term_data(i,:), dmp{i}.c, dmp{i}.w);
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

toc