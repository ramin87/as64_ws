function create_data()

clc;
clear;
close all;
clear;

setMatlabPath();

data_filename = 'data/data';

y0 = 0;
g = 1;
Tau = [0.7 0.75 0.85 1.0];
% Tau = [1.0 1.0 1.0 1.0];

Ts = 0.008;

A6 = [-20 -15 0 -30];
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

function [y, dy, ddy] = getDemoData(y0, g, t, a6, a7)

    dt = diff(t);
    tau = t(end);
    t = t/tau;
    
    a0 = y0;
    a1 = 0;
    a2 = 0;

    A = [1 1 1; 3 4 5; 6 12 20];
    b = [g-a0-a1-a2-a6-a7; 0-a1-2*a2-6*a6-7*a7; 0-2*a2-30*a6-42*a7];
    a345 = A\b;
    a3 = a345(1);
    a4 = a345(2);
    a5 = a345(3);

    %% Create data
    y = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5 + a6*t.^6 + a7*t.^7;

    dy = [0 diff(y)./dt];
    ddy = [0 diff(dy)./dt];
    
end

