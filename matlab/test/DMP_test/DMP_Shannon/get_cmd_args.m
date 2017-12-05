function cmd_args = get_cmd_args()

cmd_args = struct();

%% Set up DMP params
cmd_args.a_z = 40;
cmd_args.b_z = cmd_args.a_z/4;

cmd_args.x0 = 1;
cmd_args.x_end = 0.005;

cmd_args.N_kernels = 30;

cmd_args.std_K = 1;%0.95;

cmd_args.DMP_TYPE = 'DMP'; % 'DMP', 'DMP-bio', 'DMP-plus'

cmd_args.train_method = 'LWR'; % 'LWR', 'LS', 'RLS' , 'RLWR'

cmd_args.CAN_SYS_TYPE = 'lin'; % 'lin', exp', 'spring-damper'


%% Fourier params
cmd_args.Freq_max = 150; % remove frequencies beyond 'Freq_max'
cmd_args.Freq_min = 60; % minimum filter frequency to avoid instabilities
cmd_args.Wmin = 0.999; % get at least 'Wmin' percent of the signals total energy
cmd_args.LP_filt_order = 8;


