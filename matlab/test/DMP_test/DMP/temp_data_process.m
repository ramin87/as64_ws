clc;
close all;
clear;

fid = fopen('temp_data_1.bin');

if (fid < 0)
    error('Could not open data file'); 
end

binary = true;

Time2 = read_mat(fid, binary);
Psi2 = read_mat(fid, binary);
s2 = read_mat(fid, binary);
Fd2 = read_mat(fid, binary);
zero_tol = read_scalar(fid, binary, 'float64');

fclose(fid);

load('temp_data.mat', 'Time', 'Fd', 's', 'Psi');



% figure;
% plot(Time, Fd);
% legend('F_d');
% 
% figure;
% plot(Time, s);
% legend('s');
% 
% figure;
% hold on;
% for i=1:size(Psi,1)
%     plot(Time, Psi(i,:));
% end
% hold off;
% title('Psi activations');
% 
% zero_tol


w = LWR(Psi, s, Fd, zero_tol)

