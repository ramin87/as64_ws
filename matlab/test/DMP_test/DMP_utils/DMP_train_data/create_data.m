function create_data()

clc;
close all;
clear;

Tend = 2;

Ts = 0.002;
t = 0:Ts:Tend;

z(1,:) = sin(8*t) + t - 2*exp(t);
z(2,:) = cos(12*t) + 2*t.^2 - 3*exp(t);
z(3,:) = 2*t.^2 - 2*t.^3 - 3*exp(-2*t);

D =1;
y = zeros(D,length(t));
for i=1:D
    y(i,:) = z(i,:);
end

data = y;
save('data/data.mat','data','Ts');

save_ascii(data,Ts);
save_binary(data,Ts);

figure;
for i=1:D
    subplot(D,1,i);
    plot(t,y(i,:));
end

end

function save_ascii(data,Ts)

    fid = fopen('data/data.txt','w');

    fprintf(fid, '%.6f', Ts);
    fprintf(fid, '\n');

    write_mat(data, fid, false);

    fclose(fid);

end

function save_binary(data,Ts)

    fid = fopen('data/data.bin','w');

    fwrite(fid, Ts, 'int64');

    write_mat(data, fid, true);

    fclose(fid);

end
