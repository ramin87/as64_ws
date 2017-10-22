clc;
close all;
clear;

addpath('../../IO_lib');


binary = true;

%% create data
A = [1 2 3; 4 5 6; 7 8 9; 10 11 12]*0.1;
B = A(1:2,2:end);
m = cell(2,1);
m{1} = A;
m{2} = B;

v = [0.1 0.5 0.6]';

rowV = [0.2 0.3 0.4];


% A = rand(24,25);
% B = rand(36,28);
% m = cell(2,1);
% m{1} = A;
% m{2} = B;
% 
% v = rand(32,1);
% 
% rowV = rand(1,61);


%% write data
filename = 'out.txt';
if (binary), filename = 'out.bin'; end
fid = fopen(filename, 'w');

write_mat(A, fid, binary);

write_rowVec(rowV, fid, binary);

write_mat(B, fid, binary);

write_vec(v, fid, binary);

write_vec_mat(m, fid, binary);

fclose(fid);

%% read data
fid = fopen(filename, 'r');

A2 = read_mat(fid, binary);

rowV2 = read_rowVec(fid, binary);

B2 = read_mat(fid, binary);

v2 = read_vec(fid, binary);

m2 = read_vec_mat(fid, binary);

fclose(fid);


%% test

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
    
