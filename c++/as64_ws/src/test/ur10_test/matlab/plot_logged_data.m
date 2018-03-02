clc;
close all
clear;

addpath('utils/');

global N_JOINTS fontsize interpreter lineWidth

load('logged_data.mat','Time','q_data','dq_data','pos_data','Q_data','V_data','wrench_data','jTorques_data');

N_JOINTS = 6;
fontsize = 14;
interpreter = 'latex';
lineWidth = 1.2;

plot_joint_data(Time, q_data, dq_data, jTorques_data);

plot_cartesian_data(Time, pos_data, Q_data, V_data, wrench_data);



function plot_joint_data(Time, q_data, dq_data, jTorques_data)

    global N_JOINTS fontsize interpreter lineWidth

    figure;
    for i=1:N_JOINTS
        subplot(N_JOINTS,1,i);
        plot(Time, q_data(i,:)*180/pi, 'LineWidth',lineWidth);
        ylabel('degrees', 'Interpreter',interpreter, 'FontSize',fontsize);
        if (i==1), title('Joint Positions', 'Interpreter',interpreter, 'FontSize',fontsize); end
        if (i==N_JOINTS), xlabel('time [$s$]', 'Interpreter',interpreter, 'FontSize',fontsize); end
    end

    figure;
    for i=1:N_JOINTS
        subplot(N_JOINTS,1,i);
        plot(Time, dq_data(i,:)*180/pi, 'LineWidth',lineWidth);
        ylabel('degrees/s', 'Interpreter',interpreter, 'FontSize',fontsize);
        if (i==1), title('Joint Velocities', 'Interpreter',interpreter, 'FontSize',fontsize); end
        if (i==N_JOINTS), xlabel('time [$s$]', 'Interpreter',interpreter, 'FontSize',fontsize); end
    end

    figure;
    for i=1:N_JOINTS
        subplot(N_JOINTS,1,i);
        plot(Time, jTorques_data(i,:), 'LineWidth',lineWidth);
        ylabel('Nm', 'Interpreter',interpreter, 'FontSize',fontsize);
        if (i==1), title('Joint Torques', 'Interpreter',interpreter, 'FontSize',fontsize); end
        if (i==N_JOINTS), xlabel('time [$s$]', 'Interpreter',interpreter, 'FontSize',fontsize); end
    end

end


function plot_cartesian_data(Time, pos_data, Q_data, V_data, wrench_data)

global fontsize interpreter lineWidth

    %% Plot Cart pos
    figure;
    subplot(3,1,1);
    plot(Time, pos_data(1,:), 'LineWidth',lineWidth);
    ylabel('x [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('End-effector Cartesian position');
    subplot(3,1,2);
    plot(Time, pos_data(2,:), 'LineWidth',lineWidth);
    ylabel('y [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,1,3);
    plot(Time, pos_data(3,:), 'LineWidth',lineWidth);
    ylabel('z [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    xlabel('time [$s$]', 'Interpreter',interpreter, 'FontSize',fontsize);

    %% Plot Cart pos 3D path
    figure;
    plot3(pos_data(1,:),pos_data(2,:),pos_data(3,:), 'LineWidth',lineWidth);
    xlabel('x [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    ylabel('y [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    zlabel('z [$m$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('End-effector Cartesian position path')
    axis equal;
    
    %% Plot oriented 3D path
    figure;
    ax = axes();
    plot_3Dpath_with_orientFrames(pos_data, Q_data, ax, ...
    'numberOfFrames',12, 'frameScale',0.4, 'frameLineWidth',1.8, ...
    'frameXAxisColor', [0.64 0.08 0.18], 'frameYAxisColor', [0.75 0.75 0], 'frameZAxisColor', [0.3 0.75 0.93], ...
    'LineWidth',1.8, 'LineColor',[0.5 0.5 0.5], 'LineStyle','-', 'LineLegend', 'Demo path', ...
    'title','3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14, 'animated',false);
    axis(ax,'equal','tight');
    
    %% Plot Cartesian Velocities
    figure;
    subplot(3,2,1);
    plot(Time, V_data(1,:), 'LineWidth',lineWidth);
    ylabel('x [$m/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('Linear velocity', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,2);
    plot(Time, V_data(4,:), 'LineWidth',lineWidth);
    ylabel('x [$rad/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('Angular velocity', 'Interpreter',interpreter, 'FontSize',fontsize);
    
    subplot(3,2,3);
    plot(Time, V_data(2,:), 'LineWidth',lineWidth);
    ylabel('y [$m/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,4);
    plot(Time, V_data(5,:), 'LineWidth',lineWidth);
    ylabel('y [$rad/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    
    subplot(3,2,5);
    plot(Time, V_data(3,:), 'LineWidth',lineWidth);
    ylabel('z [$m/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    xlabel('time [s]', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,6);
    plot(Time, V_data(6,:), 'LineWidth',lineWidth);
    ylabel('z [$rad/s$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    xlabel('time [s]', 'Interpreter',interpreter, 'FontSize',fontsize);
    
    
    %% Plot Cartesian Forces
    figure;
    subplot(3,2,1);
    plot(Time, wrench_data(1,:), 'LineWidth',lineWidth);
    ylabel('x [$N$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('Cartesian forces', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,2);
    plot(Time, wrench_data(4,:), 'LineWidth',lineWidth);
    ylabel('x [$Nm$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    title('Cartesian torques', 'Interpreter',interpreter, 'FontSize',fontsize);
    
    subplot(3,2,3);
    plot(Time, wrench_data(2,:), 'LineWidth',lineWidth);
    ylabel('y [$N$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,4);
    plot(Time, wrench_data(5,:), 'LineWidth',lineWidth);
    ylabel('y [$Nm$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    
    subplot(3,2,5);
    plot(Time, wrench_data(3,:), 'LineWidth',lineWidth);
    ylabel('z [$N$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    xlabel('time [s]', 'Interpreter',interpreter, 'FontSize',fontsize);
    subplot(3,2,6);
    plot(Time, wrench_data(6,:), 'LineWidth',lineWidth);
    ylabel('z [$Nm$]', 'Interpreter',interpreter, 'FontSize',fontsize);
    xlabel('time [s]', 'Interpreter',interpreter, 'FontSize',fontsize);

end