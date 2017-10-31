function cmd_args = get_cmd_args()

cmd_args = struct();

%% Set up DMP params
cmd_args.a_z = 40;
cmd_args.b_z = cmd_args.a_z/4;

cmd_args.x0 = 1;
cmd_args.x_end = 0.005;

cmd_args.N_kernels = 60;

cmd_args.std_K = 1;

cmd_args.DMP_TYPE = 'DMP'; % 'DMP', 'DMP-bio', 'DMP-plus'

cmd_args.train_method = 'LWR'; % 'LWR', 'LS', 'RLS' , 'RLWR'

cmd_args.CAN_SYS_TYPE = 'lin'; % 'lin', exp', 'spring-damper'


cmd_args.USE_GOAL_FILT = false;
cmd_args.a_g = 10;
cmd_args.USE_PHASE_STOP = true;
cmd_args.a_px = 100; 
if (cmd_args.USE_PHASE_STOP)
    cmd_args.a_py = cmd_args.b_z;
else
    cmd_args.a_py = 0;
end

%% demos preprocess params
cmd_args.add_points_percent = 0.01;
cmd_args.smooth_points_percent = 0.03;


%% Robot controller params
cmd_args.Kd = 200;
cmd_args.Dd = 10;

%% Simulation params
cmd_args.dt = 0.002; %simulation time_step;
cmd_args.tol_stop = 1e-3;
cmd_args.max_iters = 5000;
cmd_args.tau_sim_scale = 1;


%% Apply disturbance force
cmd_args.APPLY_DISTURBANCE = false;
cmd_args.Fdist_min = 10;
cmd_args.Fdist_max = 100;
cmd_args.t1_fdist = 0.4;
cmd_args.t2_fdist = 2.2;

%% Plotting params
cmd_args.fontsize = 14;




