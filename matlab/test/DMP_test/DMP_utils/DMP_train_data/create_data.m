function create_data()

clc;
% close all;
clear;

data_filename = 'data/data';

Tend = 2;

Ts = 0.001;
t = 0:Ts:Tend;

z(1,:) = 0.8*(t - 2*exp(t)) + sin(2*pi*1.5*t) + t.^2.*exp(-t).*sin(2*pi*1.5*t); % + sin(2*pi*5*t);% + 0.5*sin(2*pi*150*t);
z(2,:) = cos(12*t) + 2*t.^2 - 3*exp(t);
z(3,:) = 2*t.^2 - 2*t.^3 - 3*exp(-2*t);

% Q(1,:) = 1 + 0.5*t.^2 - 0.1*t.^3;
% Q(2,:) = -1 + 0.1*t.^3 - 0.2*t.^2;
% Q(3,:) = 2 + 0.2*t.^2 - 0.1*t.^3;
% Q(4,:) = 0.3*t.^4 - t;


Q(1,:) = z(1,:).^2 - z(2,:);
Q(2,:) = 0.6*z(2,:) + z(3,:);
Q(3,:) = z(3,:);
Q(4,:) = z(1,:) - z(2,:);



for k=1:12
    for i=1:size(Q,2)
        Q(:,i) = Q(:,i)/norm(Q(:,i));
    end
end


D = 1;
y = zeros(D,length(t));
for i=1:D
    y(i,:) = z(i,:);
end


data = y;
Qd_data = Q;
Yd_data = z(1:3,:);
save([data_filename '.mat'],'data','Yd_data','Qd_data','Ts');


save_ascii(data,Ts,data_filename);
save_binary(data,Ts,data_filename);

[Ts2, data2] = load_binary(data_filename);

if (abs(Ts-Ts2) > 1e-50)
    error('Error reading Ts');
end

if (norm(data(:)-data2(:)) > 1e-50)
    error('Error reading Ts');
end

lineWidth = 1.2;
fontsize = 14;

figure;
for i=1:D
    subplot(D,1,i);
    plot(t,y(i,:), 'LineWidth',lineWidth);
    if (i==D)
        xlabel({'time [$s$]'}, 'Interpreter','latex', 'fontsize',fontsize);
    end
end

figure;
hold on;
plot(t,Qd_data(1,:), 'LineWidth',lineWidth);
plot(t,Qd_data(2,:), 'LineWidth',lineWidth);
plot(t,Qd_data(3,:), 'LineWidth',lineWidth);
plot(t,Qd_data(4,:), 'LineWidth',lineWidth);
legend({'$\eta$', '$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'}, 'Interpreter','latex', 'fontsize',fontsize);
title({'Orientation as Unit Quaternion'}, 'Interpreter','latex', 'fontsize',fontsize);
xlabel({'time [$s$]'}, 'Interpreter','latex', 'fontsize',fontsize);
hold off;


figure;
hold on;
plot(t,Yd_data(1,:), 'LineWidth',lineWidth);
plot(t,Yd_data(2,:), 'LineWidth',lineWidth);
plot(t,Yd_data(3,:), 'LineWidth',lineWidth);
legend({'$x$', '$y$', '$z$'}, 'Interpreter','latex', 'fontsize',fontsize);
title({'Cartesian Position'}, 'Interpreter','latex', 'fontsize',fontsize);
xlabel({'time [$s$]'}, 'Interpreter','latex', 'fontsize',fontsize);
ylabel({'Position [$m$]'}, 'Interpreter','latex', 'fontsize',fontsize);
hold off;


end

function save_ascii(data,Ts,data_filename)

    fid = fopen([data_filename '.txt'],'w');

    fprintf(fid, '%.6f', Ts);
    fprintf(fid, '\n');

    write_mat(data, fid, false);

    fclose(fid);

end

function save_binary(data,Ts,data_filename)

    fid = fopen([data_filename '.bin'],'w');

    write_scalar(Ts, fid, true);

    write_mat(data, fid, true);

    fclose(fid);

end

function [Ts, data] = load_binary(data_filename)

    fid = fopen([data_filename '.bin']);

    Ts = read_scalar(fid, true, 'double');
    
    data = read_mat(fid, true);

    fclose(fid);

end
