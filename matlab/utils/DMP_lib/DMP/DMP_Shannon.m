%% DMP class
%  Implements an 1-D DMP.
%  The DMP is driven by a canonical system. An example of an exponential
%  canonical system is:
%     dx = -ax*x/tau
%  where x is the phase variable and ax the decay term. Other types of
%  canonical systems, such as a linear canonical system, can be used.
%
%  The DMP has the following form:
%
%     tau*dz = ( a_z*(b_z*(g-y) - z ) + f*s + z_c
%     tau*dy = z + y_c;
%
%  Or equivalently:
%     ddy = ( a_z*(b_z*(g-y)-dy*tau) + f*x*(g-y0) ) / tau^2;
%
%  where
%     tau: is scaling factor defining the duration (time cycle) of the motion
%     a_z, b_z: constants relating to a spring-damper system
%     g0: the final goal
%     g: the continuous goal (or the final goal in case no goal-filtering is used)
%     y0: the initial position
%     x: the phase variable
%     y,dy,ddy: the position, velocity and accelaration of the motion
%     f: the forcing term defined by the weighted sum of the activation
%        functions (gaussian kernels), i.e.:
%        f = w'*Psi/ sum(Psi);
%     s: the scaling of the forcing term. It could be for instance
%        s = u*(g0-y0), where u=x or some other variable that reaches zero
%        as x approaches its final value (which is also close to zero).
%        Another case is s = u*K, where K is the spring-damper stiffness.
%
%   Optionally the goal 'g' fed to the DMP can be filtered to ensure smooth transitions
%   in the accelaration produced by the DMP. The filtering is donce as follows:
%     tau*dg = a_g*(g0 - g)
%   where 'g0' is the DMP goal, 'g' is the continuous goal variable and 'a_g'
%   a time contant determining how fast 'g' converges to 'g0' (the higher 'a_g'
%   is, the faster the convergence).
%

classdef DMP_Shannon < handle % : public DMP
    properties
        N_kernels % number of kernels (basis functions)
        
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system
        
        can_sys_ptr % handle (pointer) to the canonical system
        can_fun % handle to canonical function multiplying the linear term in the transformation system
        
        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP
        
        zero_tol % tolerance value used to avoid divisions with very small numbers
        
        a_s % scaling factor to ensure smaller changes in the accelaration to improve the training
        
        % training params
        train_method % training method for weights of the DMP forcing term
        
        lambda % forgetting factor in recursive training methods
        P_rlwr% Initial value of covariance matrix in recursive training methods
        
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
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function dmp = DMP_Shannon(N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue)

            dmp.can_fun = LinCanonicalFunction(0.01, 1.0);
%             dmp.can_fun = ExpCanonicalFunction(0.01, 1.0);
            
            if (nargin < 4)
                return;
            else
                if (nargin < 5), std_K=1; end
                if (nargin < 6)
                    extraArgName = [];
                    extraArgValue = [];
                end
                dmp.init(N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue);
            end
            
        end
        
        
        %% Initializes the DMP
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue)
            
            if (nargin < 6), std_K=1; end
            if (nargin < 7)
                extraArgName = [];
                extraArgValue = [];
            end
            DMP_init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue);
            
        end
        
        
        %% Sets the centers for the activation functions of the DMP according to the canonical system
        function set_centers(dmp)
            
            DMP_set_centers(dmp);
            
        end
        
        
        %% Sets the kernels of the DMP using the EM algorithm
        function set_kernels_with_EM(dmp, Time, yd_data)
            
            DMP_set_kernels_with_EM(dmp, Time, yd_data)
            
        end
        
        
        %% Sets the standard deviations for the activation functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
        function set_stds(dmp, s)
            
            dmp.h = s;
            
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
            
%             dmp.Wmin = 0.995;
%             dmp.Freq_min = 60;
%             dmp.Freq_max = 150;
%             dmp.P1_min = 0.1;
            
%             dmp.a_z = 10.0;
%             dmp.b_z = dmp.a_z/4;
%             u0 = 1.0;
%             u_end = 0.01;
%             dmp.can_fun = LinCanonicalFunction(u_end, u0);
%             dmp.can_fun = ExpCanonicalFunction(u_end, u0);
            
            dmp.a_s = 1.0/dmp.get_tau(); % 10.0/10;
            
            tau = dmp.can_sys_ptr.get_tau();
            x = dmp.can_sys_ptr.get_phaseVar(Time);

            s = zeros(size(x));
            for i=1:length(s)
                s(i) = dmp.forcing_term_scaling(x(i), y0, g);
            end
            if (length(s) == 1), s = ones(size(x))*s(1); end
            
            Fd = dmp.calc_Fd(x, yd_data, dyd_data, ddyd_data, y0, g) ./ s;
            
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
            fprintf('Frequency to get at least %.3f of the energy: Freq=%.3f Hz\n', dmp.Wmin, Freq1);
            
            
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
            fprintf('Frequency after which the amplitude drops below %.3f: Freq=%.3f Hz\n', dmp.P1_min, Freq2);
            
            % ==> Filter the signal retaining at least 'Wmin' energy
            [filter_b, filter_a] = butter(6, Freq/(Fs/2), 'low');
            Fd_filt = filtfilt(filter_b, filter_a, Fd);
            
            %[f, P1_filt] = get_single_sided_Fourier(Fd_filt, Fs);
            T1 = 1/(2*Fmax);
            %N_sync = ceil(tau/T1);
            T_sync = 0:T1:tau;
            dmp.N_kernels = length(T_sync);
            %           dmp.c = T_sync';
            dmp.c = dmp.can_sys_ptr.get_phaseVar(T_sync)';
            %           dmp.h = T1;
            dmp.h = T1/tau;
            w_sync = interp1(Time, Fd_filt, T_sync);
            dmp.w = w_sync(:);
              
            F = zeros(size(Fd));
            for i=1:size(F,2)
                Fd(i) = Fd(i) * dmp.forcing_term_scaling(x(i), y0, g);
                F(i) = dmp.forcing_term(x(i))*dmp.forcing_term_scaling(x(i), y0, g);
            end
            
            train_error = norm(F-Fd)/length(F);
            
            
            
%             fprintf('Number of kernels/sincs: %i\n', dmp.N_kernels);
%             
%             Fd0 = dmp.calc_Fd2(x, yd_data, dyd_data, ddyd_data, y0, g) ./ s;
%             [f0, P0] = get_single_sided_Fourier(Fd0, Fs);
%             
%             [f1_filt, P1_filt] = get_single_sided_Fourier(Fd_filt, Fs);
%             
%             lineWidth = 1.2;
%             fontsize = 14;
% 
% %             plot_training_data(Time, yd_data, dyd_data, ddyd_data);
%             
%             goal_attr_s = dmp.can_fun.get_output(x);
%             figure;
%             subplot(1,2,1);
%             plot(Time, Fd0, Time, Fd,'LineWidth',lineWidth);
%             legend({'$F_0$','$F_1$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(1,2,2);
%             plot(Time, (1-goal_attr_s),'LineWidth',lineWidth);
%             legend({'$goal.attr.scale$'}, 'Interpreter','latex', 'fontsize',fontsize);
% 
%             figure;
%             subplot(3,4,1);
%             plot(f0,P0, f,P1, 'LineWidth',lineWidth);
%             legend({'$P_0$','$P_1$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,2);
%             plot(f,P0-P1, 'Color',[1 0 0] ,'LineWidth',lineWidth);
%             legend({'$P_0-P_1$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,3);
%             plot(Time,Fd0, Time,Fd, 'LineWidth',lineWidth);
%             legend({'$F_0$','$F_1$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,4);
%             plot(Time,Fd0-Fd, 'Color',[1 0 0], 'LineWidth',lineWidth);
%             legend({'$F_0-F_1$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             
%             subplot(3,4,5);
%             plot(f0,P0, f1_filt,P1_filt, 'LineWidth',lineWidth);
%             legend({'$P_0$','$P_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,6);
%             plot(f,P0-P1_filt, 'Color',[1 0 0] ,'LineWidth',lineWidth);
%             legend({'$P_0-P_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,7);
%             plot(Time,Fd0, Time,Fd_filt, 'LineWidth',lineWidth);
%             legend({'$F_0$','$F_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,8);
%             plot(Time,Fd0-Fd_filt, 'Color',[1 0 0], 'LineWidth',lineWidth);
%             legend({'$F_0-F_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             
%             subplot(3,4,9);
%             plot(f,P1, f1_filt,P1_filt, 'LineWidth',lineWidth);
%             legend({'$P_1$','$P_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,10);
%             plot(f,P1-P1_filt, 'Color',[1 0 0] ,'LineWidth',lineWidth);
%             legend({'$P_1-P_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,11);
%             plot(Time,Fd, Time,Fd_filt, 'LineWidth',lineWidth);
%             legend({'$F_1$','$F_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,4,12);
%             plot(Time,Fd-Fd_filt, 'Color',[1 0 0], 'LineWidth',lineWidth);
%             legend({'$F_1-F_{1-filt}$'}, 'Interpreter','latex', 'fontsize',fontsize);
% 
%             %
%             Psi = [];
%             for i=1:length(x)
%                 Psi = [Psi dmp.activation_function(x(i))];
%             end
%             [f, P1_filt] = get_single_sided_Fourier(Fd_filt, Fs);
%             plot_filtering(filter_b, filter_a, Fs, Fmax, Time, Fd, Fd_filt, f, P1, P1_filt)
%             
%             
%             
%             plot_DMP_train(Time, F, Fd, Psi, x);
%             
%             
%             y_data = zeros(size(yd_data));
%             dy_data = zeros(size(dyd_data));
%             ddy_data = zeros(size(yd_data));
%             z_data = zeros(size(y_data));
%             dz_data = zeros(size(y_data));
%             x_data = zeros(size(Time));
%             u_data = zeros(size(Time));
%             s_data = zeros(size(Time));
%             
%             y = y0;
%             dy = 0;
%             g = g0;
%             z = 0;
%             dz = 0;
%             dt = Ts;
%             x = 0;
%             u = dmp.can_sys_ptr.get_shapeVar(x);
%             s = 0;
%             
%             for i=1:length(Time)
%                 t = Time(i);
%                 
%                 y_data(i) = y;
%                 dy_data(i) = dy;
%                 z_data(i) = z;
%                 dz_data(i) = dz;
%                 x_data(i) = x;
%                 u_data(i) = u;
%                 s_data(i) = s;
%                 
%                 x = dmp.can_sys_ptr.get_phaseVar(t);
%                 [dy, dz] = dmp.get_states_dot(x, y, z, y0, g, 0, 0);
%                 
%                 y = y + dy*dt;
%                 z = z + dz*dt;
%                 
%                 
%             end
%             
%             ddy_data = dz_data/dmp.get_v_scale();
%             
%             figure;
%             subplot(3,1,1);
%             plot(Time,y_data, Time, yd_data, 'LineWidth',lineWidth);
%             legend({'$y$','$y_d$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,1,2);
%             plot(Time,dy_data, Time, dyd_data, 'LineWidth',lineWidth);
%             legend({'$\dot{y}$','$\dot{y}_d$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             subplot(3,1,3);
%             plot(Time,ddy_data, Time, ddyd_data, 'LineWidth',lineWidth);
%             legend({'$\ddot{y}$','$\ddot{y}_d$'}, 'Interpreter','latex', 'fontsize',fontsize);
%             
%             
%             error('stop')
            
        end
        
        
        %% Sets the high level training parameters of the DMP
        %  @param[in] train_method: Method used to train the DMP weights.
        %  @param[in] lambda: Forgetting factor for recursive training methods.
        %  @param[in] P_rlwr: Covariance matrix 'P' for recursive training methods.
        function set_training_params(dmp, train_method, lambda, P_rlwr)
            
            DMP_set_training_params(dmp, train_method, lambda, P_rlwr);
            
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
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            v_scale = dmp.get_v_scale();
            Fd = (ddy*v_scale^2 - dmp.goal_attractor(x, y, v_scale*dy, g)); %./ (u.*(g0-y0) + dmp.zero_tol);
            
        end
        
        function Fd = calc_Fd2(dmp, x, y, dy, ddy, y0, g)
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            v_scale = dmp.get_v_scale();
            Fd = (ddy*v_scale^2 - dmp.goal_attractor2(x, y, v_scale*dy, g)); %./ (u.*(g0-y0) + dmp.zero_tol);
            
        end
        
        function goal_attr = goal_attractor2(dmp, x, y, z, g)
            
            goal_attr = DMP_goal_attractor(dmp, y, z, g);
            
        end
        
        
        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp,x)
            
            Psi = dmp.activation_function(x);
            f = dot(Psi,dmp.w);
            
        end
        
        
        %% Returns the scaling factor of the forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp, x, y0, g)
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            f_scale = (g-y0).*u;
            
        end
        
        
        %% Returns the goal attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp, x, y, z, g)
            
            s = dmp.can_fun.get_output(x); 
            goal_attr = (1-s).*(dmp.a_z*dmp.b_z*(g-y)) + dmp.a_z*(-z);
%             goal_attr = (1-s).*DMP_goal_attractor(dmp, y, z, g);
            
        end
        
        
        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp, x, y0, g)
            
            f = dmp.forcing_term(x);
            f_scale = dmp.forcing_term_scaling(x, y0, g);
            shape_attr = f * f_scale;
            
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
            
            if (nargin < 10), z_c=0; end
            if (nargin < 9), y_c=0; end
            
            [dy, dz] = DMP_get_states_dot(dmp, x, y, z, y0, g, y_c, z_c);
            
        end
        
        
        %% Returns a column vector with the values of the activation functions of the DMP
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the activation functions of the DMP
        function psi = activation_function(dmp,x)
            
            psi = sinc((x-dmp.c)./dmp.h);
            
        end
        
        
        %% Returns the scaling factor of the DMP
        %  @param[out] v_scale: The scaling factor of the DMP.
        function v_scale = get_v_scale(dmp)
            
            v_scale = DMP_get_v_scale(dmp);
            
        end
        
        
        %% Returns the time cycle of the DMP
        %  @param[out] tau: The time cycle of the DMP.
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




