clc;
% close all;
clear;

% m = OL_1D_DMP_RLWR_nod();
% m = OL_1D_DMP_KF_nod();
m = OL_1D_DMP_rup();

m.trainModel();

m.simulation();

m.plotResults();

