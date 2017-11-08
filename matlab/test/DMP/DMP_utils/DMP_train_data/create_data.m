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
save('data.mat','data','Ts');

save_ascii(data,Ts);

figure;
for i=1:D
    subplot(D,1,i);
    plot(t,y(i,:));
end

end

function save_ascii(data,Ts)

n_data = size(data,2);
D = size(data,1);

fid = fopen('data.txt','w');

fprintf(fid, '%i', D);
fprintf(fid, '\n');

fprintf(fid, '%i', n_data);
fprintf(fid, '\n');

fprintf(fid, '%.6f', Ts);
fprintf(fid, '\n');


for d=1:D
   for i=1:n_data
       fprintf(fid, '%.6f ', data(d,i));
   end
   fprintf(fid, '\n');
end

fclose(fid);

end

