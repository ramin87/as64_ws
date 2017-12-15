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

classdef DMPo < handle
    properties
        N_kernels % number of kernels (basis functions)
        
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system
        
        can_sys_ptr % handle (pointer) to the canonical system
        
        w % 3x1 cell, where each cell is a N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP
        
        zero_tol % tolerance value used to avoid divisions with very small numbers
        
        a_s % scaling factor to ensure smaller changes in the accelaration to improve the training
        
        % training params
        train_method % training method for weights of the DMP forcing term
        
        USE_GOAL_FILT % flag indicating whether to apply goal filtering in training or not
        a_g % filtering gain in goal filtering
        
        lambda % forgetting factor in recursive training methods
        P_rlwr% Initial value of covariance matrix in recursive training methods
        
    end
    
    methods
        %% DMP constructor
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        function dmp = DMPo(N_kernels, a_z, b_z, can_sys_ptr, std_K)
            
            if (nargin < 4)
                return;
            else
                if (nargin < 5), std_K=1; end
                dmp.init(N_kernels, a_z, b_z, can_sys_ptr, std_K)
            end
            
            
        end
        
        
        %% Initializes the DMP
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        function init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K)
            
            DMP_init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K);
            
            dmp.w = cell(3,1);
            for k=1:3
                dmp.w{k} = zeros(dmp.N_kernels);
            end
            
        end
        
        
        %% Sets the centers for the activation functions of the DMP according to the partition method specified
        %  @param[in] part_type: Partitioning method for the kernel centers.
        function set_centers(dmp, part_type)
            
            if (nargin < 2)
                DMP_set_centers(dmp);
            else
                DMP_set_centers(dmp, part_type);
            end
            
        end
        
        
        %% Sets the kernels of the DMP using the EM algorithm
        function set_kernels_with_EM(dmp, Time, yd_data)
            
            DMP_set_kernels_with_EM(dmp, Time, yd_data)
            
        end
        
        
        %% Sets the standard deviations for the activation functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
        function set_stds(dmp, s)
            
            DMP_set_stds(dmp, s)
            
        end
        
        
        %% Trains the DMP
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        %  @param[in] dyd_data: Row vector with the desired velocity.
        %  @param[in] ddyd_data: Row vector with the desired accelaration.
        %  @param[in] y0: Initial position.
        %  @param[in] g0: Target-goal position.
        %
        %  \note The timestamps in \a Time and the corresponding position,
        %  velocity and acceleration data in \a yd_data, \a dyd_data and \a
        %  ddyd_data need not be sequantial in time.
        function [train_error, F, Fd] = train(dmp, Time, Q_data, v_rot_data, dv_rot_data, Q0, Qg0)
            
            Qg = Qg0;
            
            tau = dmp.can_sys_ptr.get_tau();
            
            x = dmp.can_sys_ptr.get_phaseVar(Time);
            
            Qg = repmat(Qg, 1, length(Time));
            if (dmp.USE_GOAL_FILT)
                %             g = y0*exp(-dmp.a_g*Time/tau) + g0*(1 - exp(-dmp.a_g*Time/tau));
            end
            
            s = dmp.forcing_term_scaling(x, Q0, Qg0);
            if (length(s) == 1), s = ones(size(x))*s(1); end
            
            Fd = dmp.calc_Fd(Q_data, v_rot_data, dv_rot_data, x, Q0, Qg0, Qg);
            
            %         figure
            %         plot(Fd')
            
            Psi = dmp.activation_function(x);
            
            F = zeros(size(Fd));
            
            train_error = zeros(3,1);
            
            train_method = dmp.train_method;
            for k=1:3
                
                if (strcmpi(train_method,'LWR'))
                    
                    dmp.w{k} = LWR(Psi, s(k,:), Fd(k,:), dmp.zero_tol);
                    
                elseif (strcmpi(train_method,'RLWR'))
                    
                    dmp.w{k} = RLWR(Psi, s(k,:), Fd(k,:));
                    
                elseif (strcmpi(train_method,'LS'))
                    
                    dmp.w{k} = leastSquares(Psi, s(k,:), Fd(k,:), dmp.zero_tol);
                    
                else
                    error('Unsopported training method ''%s''', train_method);
                end
                
            end
            
            %         dmp.w{1}(end-1:end) = 0;
            
            %         for i=1:size(F,2)
            %           F(:,i) = dmp.forcing_term(x(i)).*dmp.forcing_term_scaling(u(i), Q0, Qg0);
            %         end
            
            for i=1:length(dmp.w)
                scale = s(i,:);
                forcing_tem = zeros(size(scale));
                for j=1:size(Psi,2)
                    forcing_tem(j) = dot(Psi(:,j),dmp.w{i}) / (sum(Psi(:,j))+dmp.zero_tol);
                end
                
                F(i,:) = scale.*forcing_tem;
            end
            
            for k=1:3
                train_error(k) = norm(F(k,:)-Fd(k,:))/length(F(k,:));
            end
            
            %     F = F(1,:);
            %     Fd = Fd(1,:);
            
            %       train_error
            %         n_sp = 4;
            %         figure;
            %         hold on;
            %         subplot(n_sp,1,1);
            %         plot(Time,F, Time, Fd);
            %         legend('F','F_d');
            %         hold off;
            %         Fmax = max(abs([F(:); Fd(:)]));
            %         subplot(n_sp,1,2);
            %         hold on
            %         plot(Time,F/Fmax, Time, Fd/Fmax);
            %         for i=1:size(Psi,1)
            %             plot(Time,Psi(i,:));
            %         end
            %         hold off
            %         subplot(n_sp,1,3);
            %         hold on;
            %         for i=1:size(Psi,1)
            %             temp = s.*Psi(i,:);
            %             plot(Time,temp);
            %         end
            %         hold off;
            %         subplot(n_sp,1,n_sp);
            %         plot(Time, s);
            %         legend('s')
            
        end
        
        
        %% Sets the high level training parameters of the DMP
        %  @param[in] train_method: Method used to train the DMP weights.
        %  @param[in] USE_GOAL_FILT: Flag indicating whether to use goal filtering.
        %  @param[in] a_g: Goal filtering gain.
        %  @param[in] lambda: Forgetting factor for recursive training methods.
        %  @param[in] P_rlwr: Covariance matrix 'P' for recursive training methods.
        function set_training_params(dmp, train_method, USE_GOAL_FILT, a_g, lambda, P_rlwr)
            
            DMP_set_training_params(dmp, train_method, USE_GOAL_FILT, a_g, lambda, P_rlwr);
            
        end
        
        
        %% Updates the DMP weights using RLWR (Recursive Locally Weighted Regression)
        %  @param[in] dmp: DMP object.
        %  @param[in] x: The phase variable.
        %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] y0: Initial position.
        %  @param[in] g0: Final goal.
        %  @param[in] g: Current goal.
        %  @param[in,out] P: \a P matrix of RLWR.
        %  @param[in] lambda: Forgetting factor.
        function [P] = update_weights(dmp, x, u, y, dy, ddy, y0, g0, g, P, lambda)
            
            P = RLWR_update(dmp, x, u, y, dy, ddy, y0, g0, g, P, lambda);
            
        end
        
        %% Calculates the desired values of the scaled forcing term.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
        %  @param[in] y0: initial position.
        %  @param[in] g0: final goal.
        %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter).
        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calc_Fd(dmp, Q, v_rot, ddv_rot, x, Q0, Qg0, Qg)
            
            v_scale = dmp.get_v_scale();
            Fd = (ddv_rot*v_scale^2 - dmp.goal_attractor(Q, v_scale*v_rot, Qg));
            
        end
        
        
        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp,x)
            
            f = zeros(3,1);
            
            Psi = dmp.activation_function(x);
            
            for i=1:3
                f(i) = dot(Psi,dmp.w{i}) / (sum(Psi)+dmp.zero_tol); % add 'zero_tol' to avoid numerical issues
            end
            
        end
        
        %% Returns the scaling factor of the forcing term.
        %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
        %  @param[in] y0: initial position.
        %  @param[in] g0: final goal.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp, x, Q0, Qg0)
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            f_scale = quatLog(quatProd(Qg0,quatInv(Q0)))*u;
            
        end
        
        %% Returns the goal attractor of the DMP.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp, Q, eta, Qg)
            
            goal_attr = zeros(3,size(Q,2));
            for i=1:size(goal_attr,2)
                goal_attr(:,i) = dmp.a_z*(dmp.b_z*quatLog(quatProd(Qg(:,i),quatInv(Q(:,i))))-eta(:,i));
            end
            
        end
        
        
        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
        %  @param[in] y0: initial position.
        %  @param[in] g0: final goal.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp, x, Q0, Qg0)
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            f = dmp.forcing_term(x);
            f_scale = dmp.forcing_term_scaling(u, Q0, Qg0);
            shape_attr = f .* f_scale;
            
        end
        
        
        %% Returns the derivatives of the DMP states
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] x: phase variable.
        %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
        %  @param[in] y0: initial position.
        %  @param[in] g0: final goal.
        %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter).
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        %  @param[out] dy: derivative of the \a y state of the DMP.
        %  @param[out] dz: derivative of the \a z state of the DMP.
        function [dQ, deta] = get_states_dot(dmp, Q, eta, x, Q0, Qg0, Qg, Q_c, eta_c)
            
            if (nargin < 10), eta_c=0; end
            if (nargin < 9), Q_c=0; end
            %if (nargin < 8), g=g0; end
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            v_scale = dmp.get_v_scale();
            shape_attr = dmp.shape_attractor(x, Q0, Qg0);
            goal_attr = dmp.goal_attractor(Q, eta, Qg);
            
            deta = ( goal_attr + shape_attr + eta_c) / v_scale;
            dQ = 0.5*quatProd([0; (eta+Q_c)/ v_scale], Q);
            
        end
        
        
        %% Returns a column vector with the values of the activation functions of the DMP
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the activation functions of the DMP
        function psi = activation_function(dmp,x)
            
            psi = DMP_gaussian_kernel(dmp,x);
            
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
        
        
    end
end




