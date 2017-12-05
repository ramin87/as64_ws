clc;
close all;
clear;
format compact;

global cmd_args

%% =======    initialize cmd params  ============
cmd_args = get_cmd_args();

%% =======    Set the matlab utils paths to use custom built utility functions  ==========
set_matlab_utils_path();

USE_2nd_order_can_sys = false;

%% =========    Create a sample forcing term Fd  ============
Ts = 0.001;
Tend = 2;
Time = 0:Ts:Tend;
a = Time(end)^2/Time(end)^3;
% Fd should start and end at zero
Fd = Time.^2 - a*Time.^3;

% load Fd_data Fd Time Ts

tau = Time(end);

number_of_kernels = cmd_args.N_kernels
n_data = length(Fd)


%% =========  Fourier analysis  ==============

Fs = 1/Ts;
[f, P1] = get_single_sided_Fourier(Fd, Fs);

% ==> Filter out the noise
[filter_b,filter_a] = butter(cmd_args.LP_filt_order, cmd_args.Freq_max/(Fs/2));
Fd_filt = filter(filter_b, filter_a, Fd);
[f, P1_filt] = get_single_sided_Fourier(Fd_filt, Fs);


%% =========  Set up Canonical System  =========

if (strcmpi(cmd_args.CAN_SYS_TYPE,'lin'))
    can_sys_ptr = LinCanonicalSystem();
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'exp'))
    can_sys_ptr = ExpCanonicalSystem();
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = SpringDamperCanonicalSystem();
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',cmd_args.CAN_SYS_TYPE);
end

can_sys_ptr.init(cmd_args.x_end, tau, cmd_args.x0);


%% =========  Set up and train a DMP  =========

% Set up DMP
if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
   dmp = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
    dmp = DMP_bio(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
    dmp = DMP_plus(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
else
    error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
end


% Train DMP
x = dmp.can_sys_ptr.get_continuous_output(Time);
s = x; %dmp.forcing_term_scaling(x, y0, g0);
Psi = dmp.activation_function(x);

disp('DMP training...');
tic
dmp.w = LWR_train(Psi, s, Fd_filt, dmp.zero_tol);
toc

F = zeros(size(Fd_filt));
for i=1:size(F,2)
%   F(i) = dmp.forcing_term(x(i))*dmp.forcing_term_scaling(u(i), y0, g0);
    F(i) = dmp.forcing_term(x(i))*s(i);
end


%% =========  Set up and train a Shanon DMP  =========

disp('DMP-Shanon training...');
tic
% find the maximum required frequency to get at least 'Wmin' percent of the
% total signal's energy
W = sum(P1_filt.^2);
W_temp = 0;
k = 0;
while (W_temp < W*cmd_args.Wmin)
    k = k+1;
    W_temp = W_temp + P1_filt(k)^2;
end

Fmax = f(k);

Freq = max(Fmax, cmd_args.Freq_min);

% ==> Filter the signal retaining at least 'Wmin' energy
[filter_b2, filter_a2] = butter(cmd_args.LP_filt_order, Freq/(Fs/2));
Fd_filt2 = filter(filter_b2, filter_a2, Fd);
[f, P1_filt2] = get_single_sided_Fourier(Fd_filt2, Fs);
T1 = 1/(2*Fmax);
N_sinc = ceil(tau/T1);
T_sinc = 0:T1:tau;
w_sinc = interp1(Time, Fd_filt2, T_sinc);
w_sinc = w_sinc(:); % make it a column vector
toc

N_sinc

Psi2 = [];

F2 = zeros(size(Fd_filt2));
for i=1:size(F2,2)
    t = Time(i);
%     psi = zeros(length(T_sync),1);
%     for j=1:length(psi)
%         psi(j) = my_sinc(t, T_sync(j), T1)';
%     end
    psi = sinc((t-T_sinc)/T1)';
    Psi2 = [Psi2 psi];
    F2(i) = dot(w_sinc, psi);
end

save smain.mat T_sinc w_sinc

%% =======  Plot ============

% 1st filtering process
plot_filtering(filter_b, filter_a, Fs, cmd_args.Freq_max, Time, Fd, Fd_filt, f, P1, P1_filt);

% 2nd filtering process
plot_filtering(filter_b2, filter_a2, Fs, Fmax, Time, Fd_filt, Fd_filt2, f, P1_filt, P1_filt2);


% Plot standard DMP training results
plot_DMP_train(Time, F, Fd_filt, Psi, s);

% Plot DMP-Shanon training results
plot_DMP_train(Time, F2, Fd_filt, Psi2, s);

