function cmd_args = get_cmd_args()

cmd_args = struct();

%% Set up DMP params

% Parameters of the linear part of the DMP (spring-damper)
cmd_args.a_z = 20.0;
cmd_args.b_z = cmd_args.a_z/4;

cmd_args.DMP_TYPE = 'DMP'; % 'DMP', 'DMP-bio', 'DMP-plus', 'DMP-Shannon'

cmd_args.N_kernels = 100; % number of kernels used in the DMP

cmd_args.kernelStdScaling = 1.0; % scaling factor for the kernels std

cmd_args.trainMethod = 'LWR'; % 'LWR', 'LS', 'RLS' , 'RLWR'

cmd_args.CAN_CLOCK_TYPE = 'lin';

cmd_args.SHAPE_ATTR_GATTING_TYPE = 'exp'; % 'lin', 'exp', 'spring-damper', 'sigmoid', 'constant'
cmd_args.SHAPE_ATTR_GATTING_u0 = 1.0; % starting value of the shape attractor gating
cmd_args.SHAPE_ATTR_GATTING_u_end = 0.05; % ending value of the shape attractor gating

cmd_args.GOAL_ATTR_GATTING_TYPE = 'lin'; % 'lin', 'exp', 'spring-damper', 'sigmoid', 'constant'
cmd_args.GOAL_ATTR_GATTING_u0 = 1.0; % starting value of the goal attractor gating
cmd_args.GOAL_ATTR_GATTING_u_end = 1.0; % ending value of the goal attractor gating

cmd_args.sigmoid_a_u = 280.0; % steepness of the sigmoid gating function (optional)

cmd_args.OFFLINE_DMP_TRAINING_enable = true;
cmd_args.ONLINE_DMP_UPDATE_enable = false;
cmd_args.lambda = 0.99; % forgetting factor for recursive training methods
cmd_args.P_cov = 1000000.0; % initial value of covariance matrix for recursive training methods

cmd_args.USE_GOAL_FILT = true;
cmd_args.a_g = 20.0;
cmd_args.USE_PHASE_STOP = true;
cmd_args.a_px = 50.0; 
cmd_args.a_py = 40.0; %2*cmd_args.a_z;

% Parameters for DMP-plus
cmd_args.k_trunc_kernel = 3; % number of stds beyond which the kernel is truncated

% Parameters for DMP-Shannon
cmd_args.Wmin = 0.999;
cmd_args.Freq_min = 60.0;
cmd_args.Freq_max = 150.0;
cmd_args.P1_min = 0.1;


%% Robot controller params
cmd_args.Md = 1.0; % translational inertia
cmd_args.Kd = 50.0; % translational stiffness
cmd_args.Dd = 2*sqrt(cmd_args.Kd*cmd_args.Md);  % translational damping

% Cartesian Position
cmd_args.Md_p = eye(3,3)*1.0; % rotational inertia
cmd_args.Kd_p = eye(3,3)*50.0; % rotational stiffness
cmd_args.Dd_p = eye(3,3)*2*sqrt(cmd_args.Kd_p*cmd_args.Md_p); % rotational damping

% Orientation
cmd_args.Md_o = eye(3,3)*1.0; % rotational inertia
cmd_args.Kd_o = eye(3,3)*50.0; % rotational stiffness
cmd_args.Dd_o = eye(3,3)*2*sqrt(cmd_args.Kd_o*cmd_args.Md_o); % rotational damping


%% Simulation params
cmd_args.dt = 0.002; %simulation time_step;
cmd_args.tol_stop = 0.01; % position error tolerance to stop the simulation
cmd_args.orient_tol_stop = 0.005; % orientation error tolerance to stop the simulation
cmd_args.max_iters = 3000; % maximum iteration steps
cmd_args.tau_sim_scale = 1.0; % scaling factor for the time of the DMP simulation
cmd_args.goal_scale = 1.0; % scaling factor for the goal in the DMP simulation
cmd_args.ONLINE_GOAL_CHANGE_ENABLE = false;
cmd_args.time_goal_change = [0.5 1.1 1.6]; % vector of timestamps when the goal change occurs
cmd_args.goal_change = [-12.0 -9.0 -13.0]; % vector of scalings for each goal change

cmd_args.orient_goal_change = [0.2  0.6  0.3
                               0.6  0.3  0.7
                               0.4  0.1  0.5
                               0.7  0.2  0.3];

cmd_args.CartPos_goal_change = [ -9.2  -12.6  -13.3
                                -12.4   -8.3  -14.7
                                -13.9  -11.1  -10.5];

                           
for i=1:size(cmd_args.orient_goal_change,2)
    cmd_args.orient_goal_change(:,i) = cmd_args.orient_goal_change(:,i) / norm(cmd_args.orient_goal_change(:,i));
end


%% Apply disturbance force
cmd_args.APPLY_DISTURBANCE = false; % Flag enabling/disabling the introduction of a disturbance in the robot system
cmd_args.Fdist_min = 2.0; % Minimum disturbance value
cmd_args.Fdist_max = 20.0; % Maximum disturbance value
cmd_args.t1_fdist = 0.4; % Start of Fdist_max
cmd_args.t2_fdist = 2.2; % End of Fdist_max


%% Plotting params
cmd_args.fontsize = 14;



