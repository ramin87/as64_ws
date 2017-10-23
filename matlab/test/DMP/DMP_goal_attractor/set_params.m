function set_params()

global a_z b_z D N_kernels x0 x_end tau std_K ...
       USE_GOAL_FILT a_g USE_PHASE_STOP a_px a_py ...
       CAN_SYS_TYPE ...
       tol_stop max_iters dt ...
       Kd Dd ...
       fontsize

%% Robot controller params
Kd = 150;
Dd = 2;

%% Simulation params
tol_stop = 1e-3;
max_iters = 8000;
dt = 0.002; %simulation time_step;

%% Set up DMP params

x_end = 0.005;
x0 = 1;

CAN_SYS_TYPE = 'exp'; % 'lin', exp'

N_kernels = 100;
std_K = 1.05; %1.11;

a_z = 40;
b_z = a_z/4;

USE_PHASE_STOP = true;
a_px = 100; 
a_py = b_z;%b_z;

USE_GOAL_FILT = false;
a_g = 10;

%% Plotting params
fontsize = 14;


