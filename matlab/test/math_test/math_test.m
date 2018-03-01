clc;
close all;
clear;

R = [  
   0.9154   0.6760   0.5360
   0.7071   0.4724   0.3415
   0.8702   0.7110   0.3712
      ];
  
q = rotm2quat(R);

R2 = quat2rotm(q);

R
R2