function create_data()

clc;
clear;
close all;
clear;

set_matlab_utils_path();

data_filename = 'data/data';

y0 = 0;
g = 1;
Tau = [0.84 0.95 1.05 1.2];
% Tau = [1.0 1.0 1.0 1.0];

Ts = 0.008;

A6 = [0 4 8 -5];
A7 = [0 0 0 0];

N = length(A6);
Time_data = cell(N,1);
Y_data = cell(N,1);
dY_data = cell(N,1);
ddY_data = cell(N,1);

for i=1:N
    tau = Tau(i);
    Time = 0:Ts:tau;
    t = Time/tau;
    [y, dy, ddy] = getDemoData(y0, g, t, A6(i), A7(i));
    Time_data{i} = Time;
    Y_data{i} = y;
    dY_data{i} = dy;
    ddY_data{i} = ddy;
end



% %% Plot the data
% for i=1:length(Time_data)
%     plotPosVelAccel(Time_data{i}, Y_data{i}, dY_data{i}, ddY_data{i}, 'LineWidth',2, 'FontSize',14, 'Interpreter','latex');
% end

N = length(Time_data);
legend_str = cell(N,1);
lineWidth = 1.3;
fontsize = 14;
figure;
subplot(3,1,1);
hold on;
for i=1:N
    legend_str{i} = ['demo ' num2str(i)];
    plot(Time_data{i},Y_data{i}, 'LineWidth',lineWidth);
    title('Object lifting demonstrations', 'Interpreter','latex', 'fontsize',fontsize);
    if (i==1), ylabel('Position $[m]$', 'Interpreter','latex', 'fontsize',fontsize); end 
end
legend(legend_str, 'Interpreter','latex', 'fontsize',fontsize);
axis tight;
hold off;

subplot(3,1,2);
hold on;
for i=1:N
    plot(Time_data{i},dY_data{i}, 'LineWidth',lineWidth);
    if (i==1), ylabel('Velocity $[m/s]$', 'Interpreter','latex', 'fontsize',fontsize); end
end
axis tight;
hold off;

subplot(3,1,3);
hold on;
for i=1:N
    plot(Time_data{i},ddY_data{i}, 'LineWidth',lineWidth);
    if (i==1), ylabel('Acceleration $[m/s^2]$', 'Interpreter','latex', 'fontsize',fontsize); end
    xlabel('$time [s]$', 'Interpreter','latex', 'fontsize',fontsize);
end
axis tight;
hold off;

                        
%% Save the data
% save in 'mat' format
save([data_filename '.mat'],'Ts','Time_data','Y_data','dY_data','ddY_data');

% save in 'binary' format
% save_binary(data,Time,data_filename);


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
