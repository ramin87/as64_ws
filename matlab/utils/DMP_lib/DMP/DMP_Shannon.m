%% DMP class
%% DMP class
%  Implements an 1-D DMP.
%  The DMP is driven by a canonical clock. It outputs the phase varialbe 
%  'x' which serves as a substitute for time. Typically, it evolves from 
%  x0=0 at t=0 to x_end=1, at t=tau, where tau is the total movement's 
%  duration. An example of a linear canonical clock is:
%     dx = -ax/tau
%  where x is the phase variable and ax the evolution factor. Other types 
%  of canonical clocks, such as exponential, can be used. However, keeping
%  a linear mapping between the phase variable 'x' and time 't' is more
%  intuitive.
%
%  The DMP has the in general the following form:
%
%     tau*dz = g1(x)*( a_z*(b_z*(g-y) - z ) + g2(x)*fs*f(x) + z_c
%     tau*dy = z + y_c;
%
%  Assuming y_c=z_c=0, we can write equivalently:
%     ddy = g1(x)*( a_z*(b_z*(g-y)-dy*tau) + 2(x)*fs*f(x) ) / tau^2;
%
%  where
%     tau: is scaling factor defining the duration of the motion
%     a_z, b_z: constants relating to a spring-damper system
%     fs: scaling of the forcing term (typically fs = g0-y0)
%     g: the goal-final position
%     y0: the initial position
%     x: the phase variable
%     y,dy,ddy: the position, velocity and accelaration of the motion
%     f(x): the forcing term defined by the normalized weighted sum of the 
%        kernel functions (gaussian kernels), i.e.:
%        f(x) = w'*Psi(x)/ sum(Psi(x));
%     g1(x): the gating factor of the spring-damper term
%     g2(x): the gating factor of non-linear forcing term
%

classdef DMP_Shannon < handle % : public DMP
    properties
        N_kernels % number of kernels (basis functions)

        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        canClock_ptr % handle (pointer) to the canonical clock
        shapeAttrGating_ptr % pointer to gating function for the shape attractor
        goalAttrGating_ptr % pointer to gating function for the goal attractor

        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP

        zero_tol % tolerance value used to avoid divisions with very small numbers

        a_s % scaling factor to ensure smaller changes in the accelaration to improve the training

        % training params
        train_method % training method for weights of the DMP forcing term

        lambda % forgetting factor in recursive training methods
        P_cov% Initial value of covariance matrix in recursive training methods

        Freq_min % minimum allowable filter frequency to avoid instabilities with filtering
        Freq_max % filter out all frequencies beyond 'Freq_max'
        Wmin % minimum energy percent that must be retained after filtering
        P1_min % take all frequency components up to Freq_max with amplitude >= 'P1_min', even in the case that Wmin is satisfied
        
    end

    methods
        %% DMP constructor
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shapeAttrGating_ptr: Pointer to gating function for the shape attractor.
        %  @param[in] goalAttrGating_ptr: Pointer to gating function for the goal attractor.
        %  @param[in] kernel_std_scaling: Scales the std of each kernel (optional, default = 1.0).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function dmp = DMP(N_kernels, a_z, b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling, extraArgName, extraArgValue)

            if (nargin < 6)
                return;
            else
                if (nargin < 7), kernel_std_scaling=1.0; end
                if (nargin < 8)
                    extraArgName = [];
                    extraArgValue = [];
                end
                dmp.init(N_kernels, a_z, b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling, extraArgName, extraArgValue);
            end

        end


        %% Initializes the DMP
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shapeAttrGating_ptr: Pointer to gating function for the shape attractor.
        %  @param[in] goalAttrGating_ptr: Pointer to gating function for the goal attractor.
        %  @param[in] kernel_std_scaling: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function init(dmp, N_kernels, a_z, b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling, extraArgName, extraArgValue)

            if (nargin < 8), kernel_std_scaling=1.0; end
            if (nargin < 9)
                extraArgName = [];
                extraArgValue = [];
            end

            DMP_init(dmp, N_kernels, a_z, b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling, extraArgName, extraArgValue);

        end


        %% Sets the centers for the kernel functions of the DMP according to the canonical system
        function set_centers(dmp)

            DMP_set_centers(dmp);

        end


        %% Sets the standard deviations for the kernel functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] kernel_std_scaling: Scales the variance of each kernel by 'kernel_std_scaling' (optional, default = 1.0).
        function set_stds(dmp, kernel_std_scaling)

            if (nargin < 2), kernel_std_scaling=1.0; end
            dmp.h = kernel_std_scaling;

        end


        %% Trains the DMP
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        %  @param[in] dyd_data: Row vector with the desired velocity.
        %  @param[in] ddyd_data: Row vector with the desired accelaration.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Target-goal position.
        %
        %  \note The timestamps in \a Time and the corresponding position,
        %  velocity and acceleration data in \a yd_data, \a dyd_data and \a
        %  ddyd_data MUST be sequantial in time and sampled with the same frequency.
        function [train_error, F, Fd] = train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g)

            tau = dmp.canClock_ptr.get_tau();
            x = dmp.canClock_ptr.get_phase(Time);

            s = zeros(size(x));
            for i=1:length(s)
                s(i) = dmp.forcing_term_scaling(y0, g) * dmp.shapeAttrGating_ptr.get_output(x(i));
            end

            Fd = zeros(1,length(Time));
            for i=1:length(Fd)
                Fd(i) = dmp.calc_Fd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g) ./ s(i);
            end
            

            Ts = Time(2)-Time(1);
            Fs = 1/Ts;
            [f, P1] = get_single_sided_Fourier(Fd, Fs);

            Freq_max = min(dmp.Freq_max,f(end));

            % find the maximum required frequency to get at least 'Wmin' percent of the
            % total signal's energy
            W = sum(P1(f<=Freq_max).^2);
            W_temp = 0;
            k = 0;
            while (W_temp < W*dmp.Wmin)
                k = k+1;
                W_temp = W_temp + P1(k)^2;
            end

            Freq1 = f(k);
%             fprintf('Frequency to get at least %.3f of the energy: Freq=%.3f Hz\n', dmp.Wmin, Freq1);


            while (k <= length(f))
                if (f(k) >= Freq_max)
                    break;
                end
                %               if (P1(k) > dmp.P1_min)
                %                 W_temp = W_temp + P1(k)^2;
                %               end
                if (P1(k) < dmp.P1_min)
                    break;
                end
                k = k+1;
            end

            Fmax = f(k);

            Freq = Fmax; %max(Fmax, 50);

            Freq2 = Freq;
%             fprintf('Frequency after which the amplitude drops below %.3f: Freq=%.3f Hz\n', dmp.P1_min, Freq2);

            % ==> Filter the signal retaining at least 'Wmin' energy
            [filter_b, filter_a] = butter(6, Freq/(Fs/2), 'low');
            Fd_filt = filtfilt(filter_b, filter_a, Fd);

            %[f, P1_filt] = get_single_sided_Fourier(Fd_filt, Fs);
            T1 = 1/(2*Fmax);
            %N_sync = ceil(tau/T1);
            T_sync = 0:T1:tau;
            dmp.N_kernels = length(T_sync);
            %           dmp.c = T_sync';
            dmp.c = dmp.canClock_ptr.get_phase(T_sync)';
            %           dmp.h = T1;
            dmp.h = T1/tau;
            w_sync = interp1(Time, Fd_filt, T_sync);
            dmp.w = w_sync(:);

            F = zeros(size(Fd));
            for i=1:size(F,2)
                Fd(i) = Fd(i) * dmp.forcing_term_scaling(y0, g) * dmp.shapeAttrGating_ptr.get_output(x(i));
                F(i) = dmp.forcing_term(x(i)) * dmp.forcing_term_scaling(y0, g) * dmp.shapeAttrGating_ptr.get_output(x(i));
            end

            train_error = norm(F-Fd)/length(F);

        end


        %% Sets the high level training parameters of the DMP
        %  @param[in] train_method: Method used to train the DMP weights.
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        %
        %  \remark The extra argument names can be the following:
        %  'lambda': Forgetting factor for recursive training methods.
        %  'P_cov': Initial value of the covariance matrix for recursive training methods.
        function set_training_params(dmp, train_method, extraArgName, extraArgValue)

            if (nargin < 3)
                extraArgName = [];
                extraArgValue = [];
            end
            DMP_set_training_params(dmp, train_method, extraArgName, extraArgValue);

        end


        %% Updates the DMP weights using RLWR (Recursive Locally Weighted Regression)
        %  @param[in] dmp: DMP object.
        %  @param[in] x: The phase variable.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[in,out] P: \a P matrix of RLWR.
        %  @param[in] lambda: Forgetting factor.
        function [P] = update_weights(dmp, x, y, dy, ddy, y0, g, P, lambda)

            P = RLWR_update(dmp, x, y, dy, ddy, y0, g, P, lambda);

        end


        %% Calculates the desired values of the scaled forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter)        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calc_Fd(dmp, x, y, dy, ddy, y0, g)

            v_scale = dmp.get_v_scale();
            Fd = (ddy*v_scale^2 - dmp.goal_attractor(x, y, v_scale*dy, g)); %./ (u.*(g0-y0) + dmp.zero_tol);

        end


        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp,x)

            Psi = dmp.kernel_function(x);
            f = dot(Psi,dmp.w);

        end


        %% Returns the scaling factor of the forcing term.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp, y0, g)

            f_scale = (g-y0);

        end


        %% Returns the goal attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp, x, y, z, g)

            g_attr_gating = dmp.goalAttrGating_ptr.get_output(x);
            goal_attr = g_attr_gating * DMP_goal_attractor(dmp, y, z, g);

        end


        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp, x, y0, g)

%             f = dmp.forcing_term(x);
%             f_scale = dmp.forcing_term_scaling(x, y0, g);
%             shape_attr = f * f_scale;
            s_attr_gating = dmp.shapeAttrGating_ptr.get_output(x);
            shape_attr = s_attr_gating * DMP_shape_attractor(dmp, x, y0, g);

        end


        %% Returns the derivatives of the DMP states
        %  @param[in] x: phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        %  @param[out] dy: derivative of the \a y state of the DMP.
        %  @param[out] dz: derivative of the \a z state of the DMP.
        function [dy, dz] = get_states_dot(dmp, x, y, z, y0, g, y_c, z_c)

            if (nargin < 8), z_c=0; end
            if (nargin < 7), y_c=0; end

            [dy, dz] = DMP_get_states_dot(dmp, x, y, z, y0, g, y_c, z_c);

        end


        %% Returns a column vector with the values of the kernel functions of the DMP
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP
        function psi = kernel_function(dmp,x)

            psi = sinc((x-dmp.c)./dmp.h);

        end


        %% Returns the scaling factor of the DMP
        %  @param[out] v_scale: The scaling factor of the DMP.
        function v_scale = get_v_scale(dmp)

            v_scale = DMP_get_v_scale(dmp);

        end


        %% Returns the time cycle of the DMP
        %  @param[out] tau: The time duration of the DMP.
        function tau = get_tau(dmp)

            tau = DMP_get_tau(dmp);

        end

        %% Parse extra arguments of the DMP
        %  @param[in] extraArgName: Names of extra arguments.
        %  @param[in] extraArgValue: Values of extra arguemnts.
        function parseExtraArgs(dmp, extraArgName, extraArgValue)

            dmp.Wmin = 0.95;
            dmp.Freq_min = 10;
            dmp.Freq_max = 200;
            dmp.P1_min = 0.5;

            for i=1:length(extraArgName)
                if (strcmp(extraArgName{i},'Wmin'))
                    dmp.Wmin = extraArgValue{i};
                elseif (strcmp(extraArgName{i},'Freq_min'))
                    dmp.Freq_min = extraArgValue{i};
                elseif (strcmp(extraArgName{i},'Freq_max'))
                    dmp.Freq_max = extraArgValue{i};
                elseif (strcmp(extraArgName{i},'P1_min'))
                    dmp.P1_min = extraArgValue{i};
                end
            end

            % to do: check if arguemnts are positive
        end


    end
end
