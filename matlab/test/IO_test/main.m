clc;
close all;
clear;

clc;
close all;
clear;

%% include path
MATLAB_PATH = 'C:/Users/Slifer/Dropbox/64631466/lib/as64_ws/matlab/utils/';
addpath([MATLAB_PATH 'IO_lib/']);


% choose 'binary' or 'text' format
binary = false;
precision = 10;

%% create data
% A = [1 2 3; 4 5 6; 7 8 9; 10 11 12]*0.1;
% B = A(1:2,2:end);
% m = cell(2,1);
% m{1} = A;
% m{2} = B;
% 
% v = [0.1 0.5 0.6]';
% 
% rowV = [0.2 0.3 0.4];


A = rand(24,25);
B = rand(36,28);
m = cell(2,1);
m{1} = A;
m{2} = B;

v = rand(32,1);

rowV = rand(1,61);


%% write data
filename = 'out.txt';
if (binary), filename = 'out.bin'; end
fid = fopen(filename, 'w');

fprintf('Writing to %s...\n',filename);

write_mat(A, fid, binary, precision);

write_rowVec(rowV, fid, binary, precision);

write_mat(B, fid, binary, precision);

write_vec(v, fid, binary, precision);

write_vec_mat(m, fid, binary, precision);

fclose(fid);

%% read data

fprintf('Reading from %s...\n',filename);

fid = fopen(filename, 'r');

A2 = read_mat(fid, binary);

rowV2 = read_rowVec(fid, binary);

B2 = read_mat(fid, binary);

v2 = read_vec(fid, binary);

m2 = read_vec_mat(fid, binary);

fclose(fid);


%% test

test_type = 'text format';
if (binary), test_type='binary format'; end

fprintf('Testing %s:\n',test_type);

A_err  = norm(A(:)-A2(:))
B_err  = norm(B(:)-B2(:))
v_err  = norm(v(:)-v2(:))
rowV_err  = norm(rowV(:)-rowV2(:))

m_err = 0;
for k=1:length(m)
    m_err = m_err + norm(m{k}(:)-m2{k}(:));
end
m_err

%delete(filename);
    

