clc;
clear;


cpp_results_filename = 'Shan_dat.bin';

fid = fopen(cpp_results_filename);

if (fid < 0)
    error(['Could not open file: ' cpp_results_filename]);
end

binary = true;

Time_cpp = read_mat(fid, binary);
x_cpp = read_mat(fid, binary);
s_cpp = read_mat(fid, binary);
Fd_cpp = read_mat(fid, binary);
F_cpp = read_mat(fid, binary);
P1_cpp = read_mat(fid, binary);
c_cpp = read_mat(fid, binary);
h_cpp = read_mat(fid, binary);
w_cpp = read_mat(fid, binary);

    

% figure;
% plot(Time_cpp, x_cpp, Time_cpp, s_cpp);
% legend('x','s');
% 
% figure;
% plot(Time_cpp, Fd_cpp);
% 
% figure;
% plot(P1_cpp);



load('Shan_dat.mat','Time','x','s','Fd','F','P1','c','h','w');

% figure;
% plot(Time, x, Time, s);
% legend('x','s');
% 
% figure;
% plot(Time, Fd);
% 
% figure;
% plot(P1);


err_Time = norm(Time-Time_cpp)
err_x = norm(x-x_cpp)
err_s = norm(s-s_cpp)
err_Fd = norm(Fd-Fd_cpp)
err_F = norm(F-F_cpp)
err_P1 = norm(P1-P1_cpp)
err_c = norm(c-c_cpp)
err_h = norm(h-h_cpp)
err_w = norm(w-w_cpp)

