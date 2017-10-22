function set_params()

global cmd_args

cmd_args = struct();


%% Robot controller params
cmd_args.Kd = 150;
cmd_args.Dd = 2;

%% Simulation params
cmd_args.dt = 0.002; %simulation time_step;
cmd_args.tol_stop = 1e-3;
cmd_args.max_iters = 8000;
cmd_args.tau_sim_scale = 1;

%% Set up DMP params
cmd_args.a_z = 50; %40   10 50 80 120 160
cmd_args.b_z = cmd_args.a_z/4;

cmd_args.x0 = 1;
cmd_args.x_end = 0.005;

cmd_args.N_kernels = 100;

cmd_args.std_K = 1.05;

cmd_args.train_method = 'LWR'; % 'LWR', 'LS'

cmd_args.CAN_SYS_TYPE = 'spring-damper'; % 'lin', exp', 'spring-damper'

cmd_args.USE_GOAL_FILT = true;
cmd_args.a_g = 10;
cmd_args.USE_PHASE_STOP = true;
cmd_args.a_px = 100; 
cmd_args.a_py = cmd_args.b_z;%b_z;


%% demos preprocess params
cmd_args.add_points_percent = 0.01;
cmd_args.smooth_points_percent = 0.03;


%% Apply disturbance force
cmd_args.APPLY_DISTURBANCE = true;
cmd_args.Fdist_min = 5;
cmd_args.Fdist_max = 80;
cmd_args.t1_fdist = 0.4;
cmd_args.t2_fdist = 2.2;

%% Plotting params
cmd_args.fontsize = 14;




