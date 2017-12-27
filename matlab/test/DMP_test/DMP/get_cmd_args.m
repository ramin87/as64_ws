function cmd_args = get_cmd_args()

cmd_args = struct();

%% Set up DMP params

% Do NOT change these
cmd_args.x0 = 0.0; % start of canonical time
cmd_args.x_end = 1.0; % end of canonical time

% Parameters of the linear part of the DMP (spring-damper)
cmd_args.a_z = 20.0;
cmd_args.b_z = cmd_args.a_z/4;

cmd_args.DMP_TYPE = 'DMP-Shannon'; % 'DMP', 'DMP-bio', 'DMP-plus', 'DMP-Shannon'

cmd_args.u0 = 1.0; % starting value of forcing term shaping
if (strcmpi(cmd_args.DMP_TYPE, 'DMP-Shannon'))
    cmd_args.u_end = 0.9995; % ending value of forcing term shaping
else
    cmd_args.u_end = 0.005; % ending value of forcing term shaping
end


cmd_args.N_kernels = 150; % number of kernels used in the DMP

cmd_args.std_K = 1.0; % scaling factor for the kernels std

cmd_args.train_method = 'LWR'; % 'LWR', 'LS', 'RLS' , 'RLWR'

cmd_args.CAN_CLOCK_TYPE = 'lin';
if (strcmpi(cmd_args.DMP_TYPE, 'DMP-Shannon'))
    cmd_args.CAN_FUN_TYPE = 'sigmoid'; % 'lin', 'exp', 'spring-damper', 'sigmoid'
else
    cmd_args.CAN_FUN_TYPE = 'lin'; % 'lin', 'exp', 'spring-damper', 'sigmoid'
end
cmd_args.sigmoid_a_u = 280; % steepness of the sigmoid canonical function (optional)


cmd_args.OFFLINE_DMP_TRAINING_enable = true;
cmd_args.ONLINE_DMP_UPDATE_enable = false;
cmd_args.RLWR_lambda = 0.99;
cmd_args.RLWR_P = 1e8;

cmd_args.USE_GOAL_FILT = true;
cmd_args.a_g = 20.0;
cmd_args.USE_PHASE_STOP = true;
cmd_args.a_px = 50.0; 
cmd_args.a_py = 40.0;

% Parameters for DMP-plus
cmd_args.k_trunc_kernel = 3; % number of stds beyond which the kernel is truncated

% Parameters for DMP-Shannon
cmd_args.Wmin = 0.9999;
cmd_args.Freq_min = 60;
cmd_args.Freq_max = 150;
cmd_args.P1_min = 0.05;


%% demos preprocess params
% these params are expressed in percent of the total number of training points
cmd_args.add_points_percent = 0.06; % percent of points added at the start and end of the demonstrated trajectory to ensure zero initial and final velocities/accelerations
cmd_args.smooth_points_percent = 0.035; % width of the window used to smooth velocity and acceleration (alleviate numerica differentiation noise)


%% Robot controller params
cmd_args.Md = 1.0; % translational inertia
cmd_args.Kd = 50.0; % translational stiffness
cmd_args.Dd = 2*sqrt(cmd_args.Kd*cmd_args.Md);  % translational damping

cmd_args.Md_o = 1.0; % rotational inertia
cmd_args.Kd_o = 4.0; % rotational stiffness
cmd_args.Dd_o = 2*sqrt(cmd_args.Kd_o*cmd_args.Md_o); % rotational damping


%% Simulation params
cmd_args.dt = 0.002; %simulation time_step;
cmd_args.tol_stop = 5e-3; % position error tolerance to stop the simulation
cmd_args.orient_tol_stop = 2e-3; % orientation error tolerance to stop the simulation
cmd_args.max_iters = 3000; % maximum iteration steps
cmd_args.tau_sim_scale = 1.0; % scaling factor for the time of the DMP simulation
cmd_args.goal_scale = 1.0; % scaling factor for the goal in the DMP simulation
cmd_args.ONLINE_GOAL_CHANGE_ENABLE = false;
cmd_args.time_goal_change = [0.5 1.1 1.6]; % vector of timestamps when the goal change occurs
cmd_args.goal_change = [-12.0 -9.0 -13.0]; % vector of scalings for each goal change


%% Apply disturbance force
cmd_args.APPLY_DISTURBANCE = false; % Flag enabling/disabling the introduction of a disturbance in the robot system
cmd_args.Fdist_min = 5.0; % Minimum disturbance value
cmd_args.Fdist_max = 200.0; % Maximum disturbance value
cmd_args.t1_fdist = 0.4; % Start of Fdist_max
cmd_args.t2_fdist = 2.2; % End of Fdist_max


%% Plotting params
cmd_args.fontsize = 14;




