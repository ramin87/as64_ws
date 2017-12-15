function create_data()

clc;
close all;
clear;

data_filename = 'data/data';

Tend = 2;

Ts = 0.001;
t = 0:Ts:Tend;

z(1,:) = 0.8*(t - 2*exp(t)) + sin(2*pi*1.5*t) + t.^2.*exp(-t).*sin(2*pi*1.5*t); % + sin(2*pi*5*t);% + 0.5*sin(2*pi*150*t);
z(2,:) = cos(12*t) + 2*t.^2 - 3*exp(t);
z(3,:) = 2*t.^2 - 2*t.^3 - 3*exp(-2*t);

Q(1,:) = 1 + t.^2 - 0.3*t.^3;
Q(2,:) = -1 + t.^3 - 2*t.^2;
Q(3,:) = 2 + t.^2 - t.^3;
Q(4,:) = 0.3*t.^4 - t;

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
Q_data = Q;
save([data_filename '.mat'],'data','Q_data','Ts');


save_ascii(data,Ts,data_filename);
save_binary(data,Ts,data_filename);

[Ts2, data2] = load_binary(data_filename);

if (abs(Ts-Ts2) > 1e-50)
    error('Error reading Ts');
end

if (norm(data(:)-data2(:)) > 1e-50)
    error('Error reading Ts');
end

figure;
for i=1:D
    subplot(D,1,i);
    plot(t,y(i,:));
end

figure;
hold on;
plot(t,Q(1,:));
plot(t,Q(2,:));
plot(t,Q(3,:));
plot(t,Q(4,:));
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
