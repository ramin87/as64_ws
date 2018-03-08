clc;
close all;
clear;

set_matlab_utils_path();

CartPos_filename = 'data/CartPos_data';
orient_filename = 'data/orient_data';


filename = 'data/ur10_demo_data.bin';
fid = fopen(filename,'r');
if (fid < 0), error(['Failed to open file ' filename]); end

if (~isempty(regexp(filename, '.*\.txt','once')))
    binary = false;
else
    binary = true;
end

Time = read_mat(fid, binary);

Yd_pos_data = read_mat(fid, binary);
Yd_vel_data = read_mat(fid, binary);
Yd_accel_data = read_mat(fid, binary);

Qd_pos_data = read_mat(fid, binary);
Qd_vel_data = read_mat(fid, binary);
Qd_accel_data = read_mat(fid, binary);


% for i=2:size(Qd_pos_data,2)
%     if (Qd_pos_data(:,i)'*Qd_pos_data(:,i-1)<0)
%         Qd_pos_data(:,i) = -Qd_pos_data(:,i);
%     end
% end


CartPos_data = {Yd_pos_data; Yd_vel_data; Yd_accel_data};
Orient_data = {Qd_pos_data; Qd_vel_data; Qd_accel_data};


ax = axes();
plot_3Dpath_with_orientFrames(Yd_pos_data, Qd_pos_data, ax, ...
    'numberOfFrames',2, 'frameScale',0.1, 'frameLineWidth',1.8, ...
    'frameXAxisColor', [0.64 0.08 0.18], 'frameYAxisColor', [0.75 0.75 0], 'frameZAxisColor', [0.3 0.75 0.93], ...
    'LineWidth',1.4, 'LineColor',[0.5 0.5 0.5], 'LineStyle','-', 'LineLegend', 'Demo path', ...
    'title','3D path with frames', 'xlabel','x-axis[$m$]', 'ylabel','y-axis[$m$]', 'zlabel','z-axis[$m$]', ...
    'Interpreter', 'latex', 'fontSize', 14);
% axis(ax,'equal');

lineWidth = 1.2;
fontsize = 14;


figure
hold on;
plot(Time, Qd_pos_data(1,:), 'LineWidth',lineWidth);
plot(Time, Qd_pos_data(2,:), 'LineWidth',lineWidth);
plot(Time, Qd_pos_data(3,:), 'LineWidth',lineWidth);
plot(Time, Qd_pos_data(4,:), 'LineWidth',lineWidth);
title('Orientation','Interpreter','latex', 'FontSize', fontsize);
legend({'$\eta$','$\epsilon_1$','$\epsilon_2$','$\epsilon_3$'},'Interpreter','latex', 'FontSize', fontsize);
hold off;

figure
hold on;
plot(Time, Yd_pos_data(1,:), 'LineWidth',lineWidth);
plot(Time, Yd_pos_data(2,:), 'LineWidth',lineWidth);
plot(Time, Yd_pos_data(3,:), 'LineWidth',lineWidth);
title('Cartesian Position','Interpreter','latex', 'FontSize', fontsize);
legend({'$x$','$y$','$z$'},'Interpreter','latex', 'FontSize', fontsize);
hold off;

%% Save the data
save([CartPos_filename '.mat'],'CartPos_data','Time');
save([orient_filename '.mat'],'Orient_data','Time');

% save in 'binary' format
save_binary(CartPos_data,Time,CartPos_filename);
save_binary(Orient_data,Time,orient_filename);

% save in 'txt' format
save_ascii(CartPos_data,Time,CartPos_filename);
save_ascii(Orient_data,Time,orient_filename);



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