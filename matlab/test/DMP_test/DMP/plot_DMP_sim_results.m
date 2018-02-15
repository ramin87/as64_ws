function plot_DMP_sim_results()

% clc; 
% close all;
clear;

set_matlab_utils_path();

load data/dmp_results.mat log_data

plot_DMP_sim_results_helper(log_data);

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
x_data = log_data.x_data;

g_data = log_data.g_data;

y_data = log_data.y_data;
dy_data = log_data.dy_data;
ddy_data = log_data.ddy_data;
z_data = log_data.z_data;
dz_data = log_data.dz_data;

y_robot_data = log_data.y_robot_data;
dy_robot_data = log_data.dy_robot_data;
ddy_robot_data = log_data.ddy_robot_data;

Fdist_data = log_data.Fdist_data;

dmp = log_data.dmp;
P_lwr = log_data.P_lwr;
DMP_w = log_data.DMP_w;
DMP_c = log_data.DMP_c;
DMP_h = log_data.DMP_h;


shapeAttrGating_data = dmp{1}.shapeAttrGating(x_data);
goalAttrGating_data = dmp{1}.goalAttrGating(x_data);

Force_term_data = zeros(size(y_data));
y0 = y_data(:,1);
for j=1:length(x_data)
    for i=1:D
        Force_term_data(i,j) = dmp{i}.learnedForcingTerm(x_data(j), y0(i), g_data(i,j));
    end
end

% Psi_data_train = cell(D,1);
% for i=1:D
%     x_data_train = dmp{i}.phase(Time_offline_train);
%     for j=1:length(x_data_train)
%         Psi_data_train{i} = [Psi_data_train{i} dmp{i}.kernelFunction(x_data_train(j))];
%     end
% end
% 
% Psi_data = cell(D,1);
% for i=1:D
%     x_data_train = dmp{i}.phase(Time);
%     for j=1:length(x_data_train)
%         Psi_data{i} = [Psi_data{i} dmp{i}.kernelFunction(x_data(j))];
%     end
% end

e_track_data = y_data - y_robot_data;



OFFLINE_DMP_TRAINING_enable = ~isempty(F_offline_train_data);
ONLINE_DMP_UPDATE_enable = ~isempty(F_online_train_data);
USE_GOAL_FILT = g_data(1,1)~=g_data(1,end);

%% ========   Plot results  ========
disp('Ploting results...')
tic

if (log_data.poseDataFlag)
    
   Time_d = Time_demo;
   Pos_d = yd_data(1:3,:);
   Quat_d = qpos2quat(yd_data(4:6,:));
   
   Time = Time;
   Pos = y_robot_data(1:3,:);
   Quat = qpos2quat(y_robot_data(4:6,:));
   
   figure;
   ax = axes();
   
   plot_3Dpath_with_orientFrames(Pos_d, Quat_d, ax, ...
    'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',1.8, ...
    'frameXAxisColor', [0.64 0.08 0.18], 'frameYAxisColor', [0.75 0.75 0], 'frameZAxisColor', [0.3 0.75 0.93], ...
    'LineWidth',1.4, 'LineColor',[0.5 0.5 0.5], 'LineStyle','--', 'LineLegend', 'Demo path', ...
    'title','3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14);

   
   plot_3Dpath_with_orientFrames(Pos, Quat, ax, ...
    'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',1.8, ...
    'frameXAxisColor', [1 0 0], 'frameYAxisColor', [0 1 0], 'frameZAxisColor', [0 0 1], ...
    'frameXAxisLegend', 'Robot x-axis', 'frameYAxisLegend', 'Robot y-axis', 'frameZAxisLegend', 'Robot z-axis', ...
    'LineWidth',1.4, 'LineColor',[0.6 0.2 0.0], 'LineLegend', 'Robot path', ...
    'title','3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14, ...
    'animated',false, 'Time',0.001*Time, 'VideoCapture',false, 'ScreenScale', [0.7 0.6]);
    
end


%% Plot the training data
plotPosVelAccel(Time_demo, yd_data, dyd_data, ddyd_data, 'LineWidth',2, 'FontSize',14, 'Interpreter','latex');

for i=1:D
%     figure;
%     subplot(2,1,1);
%     plot(Time,y_data(i,:)-g0(i));
%     legend({'$e_{DMP}=p_{DMP}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
%     subplot(2,1,2);
%     plot(Time,y_robot_data(i,:)-g0(i));
%     legend({'$e_{robot}=p_{robot}-p_{goal}$'},'Interpreter','latex','fontsize',fontsize);

%     n_splots = 3;
%     figure;
%     subplot(n_splots,1,1);
%     plot(Time,x_data, Time,shapeAttrGating_data, Time, goalAttrGating_data);
%     legend({'$x$','$shapeAttrGating$', '$goalAttrGating$'},'Interpreter','latex','fontsize',fontsize);
%     subplot(n_splots,1,2);
%     plot(Time,y_robot_data(i,:), Time,y_data(i,:), Time_demo,yd_data(i,:), Time,g_data(i,:), '--', Time(end),g0(i),'r*','Markersize',10);
%     legend({'$p_{robot}$','$p_{DMP}$','$p_{train}$','goal evolution','$p_{goal}$'},'Interpreter','latex','fontsize',fontsize);
%     subplot(n_splots,1,3);
%     plot(Time,Fdist_data(i,:));
%     legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);

%     n_splots = 4;
%     figure;
%     subplot(n_splots,1,1);
%     plot(Time,Fdist_data);
%     legend({'Disturbance force'},'Interpreter','latex','fontsize',fontsize);
%     xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
%     subplot(n_splots,1,2);
%     plot(Time,Force_term_data);
%     legend({'Forcing term'},'Interpreter','latex','fontsize',fontsize);
%     subplot(n_splots,1,3);
%     plot(Time,e_track_data(i,:));
%     legend({'$e_{track}$'},'Interpreter','latex','fontsize',fontsize);
%     subplot(n_splots,1,4);
%     plot(Time,dy_robot_data(i,:), Time, dy_data(i,:), Time, z_data(i,:));
%     legend({'$\dot{y}_{robot}$','$\dot{y}_{DMP}$','$z_{DMP}$'},'Interpreter','latex','fontsize',fontsize);

end

% %% Plot 'F' training
% if (OFFLINE_DMP_TRAINING_enable)
%     for i=1:D 
%         lineWidth = 1.2;
%         F = F_offline_train_data(i,:);
%         Fd = Fd_offline_train_data(i,:);
%         Psi = Psi_data_train{i};
%         plot_F1_F2_Psi(Time_offline_train, F, Time_offline_train, Fd, Time_offline_train, Psi, lineWidth, fontsize, 'Forcing term - Offline training', '$F$', '$F_d$');
%     end
% end
% 
% 
% %% Plot 'F' online training
% if (ONLINE_DMP_UPDATE_enable)
%     for i=1:D
%         F = F_online_train_data(i,:);
%         Fd = Fd_online_train_data(i,:);
%         Psi = Psi_data{i};
%         plot_F1_F2_Psi(Time_offline_train, F, Time_offline_train, Fd, Time_offline_train, Psi, lineWidth, fontsize, 'Forcing term - Online training', '$F$', '$F_d$');
%     end
% end

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

% %% Plot goal evolution
% if (USE_GOAL_FILT)
%     figure;
%     for i=1:D
%         subplot(D,1,i);
%         plot(Time,g_data(i,:));
%         if (i==1), title('Goal evolution','Interpreter','latex','fontsize',fontsize); end
%         axis tight;
%     end
% end


%% Plot DMP simulation and demo pos, vel, accel
lineWidth = 1.2;
% plot_signals_and_errorSignal(Time,y_robot_data, Time_demo,y_data, 'robot', 'DMP', 'Position', lineWidth);
% plot_signals_and_errorSignal(Time,dy_robot_data, Time_demo,dy_data, 'robot', 'DMP', 'Velocity', lineWidth);
% plot_signals_and_errorSignal(Time,ddy_robot_data, Time_demo,ddy_data, 'robot', 'DMP', 'Acceleration', lineWidth);

plot_signals_and_errorSignal(Time,y_robot_data, Time_demo,yd_data, 'robot', 'demo', 'Position', lineWidth);
plot_signals_and_errorSignal(Time,dy_robot_data, Time_demo,dyd_data, 'robot', 'demo', 'Velocity', lineWidth);
plot_signals_and_errorSignal(Time,ddy_robot_data, Time_demo,ddyd_data, 'robot', 'demo', 'Acceleration', lineWidth);

% plot_signals_and_errorSignal(Time,y_data, Time_demo,yd_data, 'DMP', 'demo', 'Position', lineWidth);
% plot_signals_and_errorSignal(Time,dy_data, Time_demo,dyd_data, 'DMP', 'demo', 'Velocity', lineWidth);
% plot_signals_and_errorSignal(Time,ddy_data, Time_demo,ddyd_data, 'DMP', 'demo', 'Acceleration', lineWidth);


%% Plot psi kernels with respect to phase variable
% for i=1:D
%     f_data = Force_term_data(i,:);
%     plot_psi_kernels_and_psiWeightedSum(x_data,Psi_data{i}, f_data, dmp{i}.c, dmp{i}.w);
% end


%% Plot 2D or 3D line path of DMP simulation and demo
if (D==2 || D==3)
    plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
end

% for i=1:D
%     lineWidth = 1.2;
%     plot_F1_F2_Psi(Time, Force_term_data(i,:), Time_offline_train, Fd_offline_train_data(i,:), Time, Psi_data{i}, lineWidth, fontsize, 'Forcing term - Simulation', '$F_{sim}$', '$F_d$');
% end

% figure;
% for i=1:D
% subplot(D,1,i);
% bar(DMP_c{i}, DMP_w{i});%, 3*sqrt(1/DMP_h{i}(1)));
% if (i==1)
%     title('DMP weights');
% end
% ylabel(['DMP ' num2str(i)]);
% xlabel('DMP centers');
% axis tight;
% end

toc

end


