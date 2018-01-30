function create_data()

clc;
% close all;
clear;

data_filename = 'data/data';
CartPos_filename = 'data/CartPos_data';
orient_filename = 'data/orient_data';

Tend = 2.0;

Ts = 0.001;
t = 0:Ts:Tend;


%% Create data
z(3,:) = 0.8*(t - 2*exp(t)) + sin(2*pi*1.5*t) + t.^2.*exp(-t).*sin(2*pi*1.5*t); % + sin(2*pi*5*t);% + 0.5*sin(2*pi*150*t);
z(1,:) = cos(12*t) + 2*t.^2 - 3*exp(t);
z(2,:) = 2*t.^2 - 2*t.^3 - 3*exp(-2*t);

D = 1; % change this to define the data dimensionality
y = zeros(D,length(t));
for i=1:D
    y(i,:) = z(i,:);
end


%% Create quaternion data
Q(1,:) = z(1,:).^2 - z(2,:);
Q(2,:) = 0.6*z(2,:) + z(3,:);
Q(3,:) = z(3,:);
Q(4,:) = z(1,:) - z(2,:);

% normalize the quaternions
for k=1:12
    for i=1:size(Q,2)
        Q(:,i) = Q(:,i)/norm(Q(:,i));
    end
end

Qd_pos_data = Q;


%% Create Cartesian position data
data_pos = y;
Yd_pos_data = z(1:3,:);


%% Smooth all generated data to ensure smooth signals with zero intial and final velocities and accelerations
add_points = 50;
smooth_points = add_points/2;
smooth_times = 3.0;
smooth_method = 'moving';
useSmoothing = true;
recomputeDerivatives = true;

data_pos = [repmat(data_pos(:,1),1,add_points) data_pos repmat(data_pos(:,end),1,add_points)];
Yd_pos_data = [repmat(Yd_pos_data(:,1),1,add_points) Yd_pos_data repmat(Yd_pos_data(:,end),1,add_points)];
Qd_pos_data = [repmat(Qd_pos_data(:,1),1,add_points) Qd_pos_data repmat(Qd_pos_data(:,end),1,add_points)];

% calculate numerically the 1st and 2nd derivatives
data_dim_title = cell(0);
for i=1:D
    data_dim_title{i} = ['dim ' num2str(i)];
end
[data_pos, data_vel, data_accel] = calcLinVelAccel(data_pos, Ts, ...
                             'useSmoothing',useSmoothing, 'smoothTimes',smooth_times, 'smoothMethod',smooth_method, ....
                             'smoothMethodDegree',2, 'recomputeLowerDerivatives',recomputeDerivatives, ...
                             'returnAllDerivatives',true, 'smoothSpan',smooth_points);
                             


[Yd_pos_data, Yd_vel_data, Yd_accel_data] = calcLinVelAccel(Yd_pos_data, Ts, ...
                             'useSmoothing',useSmoothing, 'smoothTimes',smooth_times, 'smoothMethod',smooth_method, ....
                             'smoothMethodDegree',2, 'recomputeLowerDerivatives',recomputeDerivatives, ...
                             'returnAllDerivatives',true, 'smoothSpan',smooth_points);
                             
                         

[Qd_pos_data, Qd_vel_data, Qd_accel_data] = calcRotVelAccel(Qd_pos_data, Ts, ...
                             'useSmoothing',useSmoothing, 'smoothTimes',smooth_times, 'smoothMethod',smooth_method, ....
                             'smoothMethodDegree',2, 'recomputeLowerDerivatives',recomputeDerivatives, ...
                             'returnAllDerivatives',true, 'smoothSpan',smooth_points);
                             

                        
%% Group the generated data in cell arrays
%  Each cell array is 3x1 containing the position, velocity and acceleration.
Time = (0:(size(data_pos,2)-1))*Ts;
data = {data_pos; data_vel; data_accel};
CartPos_data = {Yd_pos_data; Yd_vel_data; Yd_accel_data};
Orient_data = {Qd_pos_data; Qd_vel_data; Qd_accel_data};


%% Save the data
% save in 'mat' format
save([data_filename '.mat'],'data','Time');
save([CartPos_filename '.mat'],'CartPos_data','Time');
save([orient_filename '.mat'],'Orient_data','Time');

% save in 'binary' format
save_binary(data,Time,data_filename);
save_binary(CartPos_data,Time,CartPos_filename);
save_binary(Orient_data,Time,orient_filename);

% save in 'txt' format
save_ascii(data,Time,data_filename);
save_ascii(CartPos_data,Time,CartPos_filename);
save_ascii(Orient_data,Time,orient_filename);


%% Plot the data
lineWidth = 2;
fontSize = 14;
interpreter = 'latex';

plotPosVelAccel(Time, data{1}, data{2}, data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',data_dim_title);
plotPosVelAccel(Time, CartPos_data{1}, CartPos_data{2}, CartPos_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'CartPos x', 'CartPos y', 'CartPos z'});
plotPosVelAccel(Time, quat2qpos(Orient_data{1}), Orient_data{2}, Orient_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'qpos 1', 'qpos 2', 'qpos 3'});


% %% Load the data from 'txt' file and plot them (comparison with initial data)
% [Time, data] = load_ascii(data_filename);
% [Time, CartPos_data] = load_ascii(CartPos_filename);
% [Time, Orient_data] = load_ascii(orient_filename);
% 
% plotPosVelAccel(Time, data{1}, data{2}, data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',data_dim_title);
% plotPosVelAccel(Time, CartPos_data{1}, CartPos_data{2}, CartPos_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'CartPos x', 'CartPos y', 'CartPos z'});
% plotPosVelAccel(Time, quat2qpos(Orient_data{1}), Orient_data{2}, Orient_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'qpos 1', 'qpos 2', 'qpos 3'});
% 
% 
% %% Load the data from 'bin' file and plot them (comparison with initial data)
% [Time, data] = load_binary(data_filename);
% [Time, CartPos_data] = load_binary(CartPos_filename);
% [Time, Orient_data] = load_binary(orient_filename);
% 
% plotPosVelAccel(Time, data{1}, data{2}, data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',data_dim_title);
% plotPosVelAccel(Time, CartPos_data{1}, CartPos_data{2}, CartPos_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'CartPos x', 'CartPos y', 'CartPos z'});
% plotPosVelAccel(Time, quat2qpos(Orient_data{1}), Orient_data{2}, Orient_data{3}, 'LineWidth',lineWidth, 'FontSize',fontSize, 'Interpreter',interpreter, 'dimTitle',{'qpos 1', 'qpos 2', 'qpos 3'});


end

function save_ascii(data,Time,data_filename)

    fid = fopen([data_filename '.txt'],'w');
    
    write_mat(Time, fid, false);
    
    D = length(data);
    write_scalar(D, fid, false);
    fprintf(fid, '\n');
    
    for i=1:length(data)
        write_mat(data{i}, fid, false);
    end

    fclose(fid);

end


function save_binary(data,Time,data_filename)

    fid = fopen([data_filename '.bin'],'w');
    
    write_mat(Time, fid, true);
    
    D = int32(length(data));
    write_scalar(D, fid, true);
    
    for i=1:length(data)
        write_mat(data{i}, fid, true);
    end

    fclose(fid);

end


function [Time, data] = load_ascii(data_filename)

    fid = fopen([data_filename '.txt'],'r');
    
    Time = read_mat(fid, false);
    
    D = read_scalar(fid, false);
    data = cell(D,1);
    
    for i=1:length(data)
        data{i} = read_mat(fid, false);
    end

    fclose(fid);

end


function [Time, data] = load_binary(data_filename)

    fid = fopen([data_filename '.bin'],'r');
    
    Time = read_mat(fid, true);
    
    D = read_scalar(fid, true, 'int32');
    data = cell(D,1);
    
    for i=1:length(data)
        data{i} = read_mat(fid, true);
    end

    fclose(fid);

end
