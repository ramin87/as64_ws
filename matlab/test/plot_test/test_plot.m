clc;
close all;
clear;

% figure;
% t_frame = get_frame_handle(0.2);

dt = 0.01;
Tend = 2;
Time = 0:dt:Tend;

x = Time.^2 - Time.^3;
y = Time.^1.5 - 0.2*Time.^4;
z = 5*exp(-Time);
Pos = [x; y; z];

q1 = 0.5*Time - exp(-0.5*Time);
q2 = 0.2*Time.^2 + exp(-0.5*Time);
q3 = 0.25*Time.^3;
q4 = sqrt(Time);
Quat = [q1; q2; q3; q4];

for i=1:size(Quat,2)
   Quat(:,i) =  Quat(:,i)/norm(Quat(:,i));
end

% figure;
% plot(Pos');
% 
% figure
% plot(Q');

% animate_3d_trajectory_of_frames(Time, Pos, Quat);


figH = figure('Name','3d path with orientation frames','NumberTitle','off');
set(groot,'CurrentFigure',figH); % set this figure as the current figure
% Create two axes
subplot(2,1,1);
subplot(2,1,2);
% ax = gca; % get the axis of the current figure

ax = figH.Children(2);
% figH.CurrentAxes = figH.Children(2);
% ax = figH.Children(2);
tic
plot_3Dpath_with_orientFrames(Pos, Quat, ax, ...
    'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',1.8, ...
    'frameXAxisColor', [1 0 0], 'frameYAxisColor', [0 1 0], 'frameZAxisColor', [0 0 1], ...
    'LineWidth',1.4, 'LineColor',[0.5 0.2 0.33], ...
    'title','3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14);

toc
ax = figH.Children(1);
plot_3Dpath_with_orientFrames(Pos, Quat, ax, ...
    'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',1.8, ...
    'LineWidth',1.4, 'LineColor',[0.5 0.2 0.33], ...
    'title','Animated 3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14, ...
    'animated',true, 'Time',0.2*Time, 'DADs', 6, 'WOWOF', 'faf');


