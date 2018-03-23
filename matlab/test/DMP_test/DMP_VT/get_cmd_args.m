function cmd_args = get_cmd_args()

cmd_args = struct();

%% Set up DMP params

% Parameters of the linear part of the DMP (spring-damper)
cmd_args.a_z = 20.0;
cmd_args.b_z = 0;

cmd_args.N_kernels = 100; % number of kernels used in the DMP

cmd_args.kernelStdScaling = 1.0; % scaling factor for the kernels std

cmd_args.trainMethod = 'LWR'; % 'LWR', 'LS', 'RLS' , 'RLWR'

cmd_args.lambda = 0.98; % forgetting factor for recursive training methods
cmd_args.P_cov = 1e6; % initial value of covariance matrix for recursive training methods

cmd_args.USE_GOAL_FILT = false;
cmd_args.a_g = 0.0;
cmd_args.USE_PHASE_STOP = false;
cmd_args.a_px = 0.0; 
cmd_args.a_py = 0.0;

cmd_args.K_f_err = 1.0;

%% Reference model
% cmd_args.y0_ref = 0.0;
% cmd_args.g_ref = 1.0;
% cmd_args.a6 = -2.0;
% cmd_args.a7 = 0.0;
% cmd_args.tau_ref = 1.0;

cmd_args.y0_ref = 0.0;
cmd_args.g_ref = 1.5;
cmd_args.a6 = 20.0;
cmd_args.a7 = 0.0;
cmd_args.tau_ref = 0.9;


%% Human model
cmd_args.Mh = 1.0;
cmd_args.Kh = 150.0;
cmd_args.Dh = 2*sqrt(cmd_args.Mh*cmd_args.Kh);

%% Robot model
cmd_args.Mr = 1.0;
cmd_args.Kr = 500.0;
cmd_args.Dr = 2*sqrt(cmd_args.Mr*cmd_args.Kr);

%% Object Model
cmd_args.Mo = 10.0;
cmd_args.robot_load_p = 0.7; % percent of the weight carried by the robot
cmd_args.human_load_p = 0.3; % percent of the weight carried by the human
cmd_args.human_load_p_var = 0;
cmd_args.const_wh_error = false;

cmd_args.Kc = 1000;


%% Robot controller params
cmd_args.Md = 1.0; % translational inertia
cmd_args.Kd = 50.0; % translational stiffness
cmd_args.Dd = 2*sqrt(cmd_args.Kd*cmd_args.Md);  % translational damping

%% Simulation params
cmd_args.dt = 0.008; %simulation time_step;
cmd_args.tol_stop = 0.1; % position error tolerance to stop the simulation
cmd_args.max_iters = 5000; % maximum iteration steps
cmd_args.tau_sim_scale = 1.0; % scaling factor for the time of the DMP simulation

%% Plotting params
cmd_args.fontsize = 14;



