function set_params()

global a_z b_z D N_kernels x0 x_end tau std_K ...
       USE_GOAL_FILT a_g USE_PHASE_STOP a_px a_py ...
       CAN_SYS_TYPE ...
       tol_stop max_iters dt ...
       Kd Dd ...
       APPLY_DISTURBANCE Fdist_min Fdist_max t1_fdist t2_fdist ...
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

CAN_SYS_TYPE = 'lin'; % 'lin', exp', 'spring-damper'

N_kernels = 100;
std_K = 1.05;

a_z = 50; %40   10 50 80 120 160
b_z = a_z/4;

USE_PHASE_STOP = true;
a_px = 100; 
a_py = b_z;%b_z;

USE_GOAL_FILT = false;
a_g = 10;

%% Apply disturbance force
APPLY_DISTURBANCE = false;
Fdist_min = 5;
Fdist_max = 80;
t1_fdist = 0.4;
t2_fdist = 2.2;

%% Plotting params
fontsize = 14;




