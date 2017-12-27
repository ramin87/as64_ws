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

classdef DMP < handle % : public DMP_
    properties
        N_kernels % number of kernels (basis functions)
        
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system
        
        can_sys_ptr % handle (pointer) to the canonical system
        
        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP
        
        zero_tol % tolerance value used to avoid divisions with very small numbers
        
        a_s % scaling factor to ensure smaller changes in the accelaration to improve the training
        
        % training params
        train_method % training method for weights of the DMP forcing term
        
        lambda % forgetting factor in recursive training methods
        P_rlwr% Initial value of covariance matrix in recursive training methods
        
        can_fun % handle to canonical function multiplying the linear term in the transformation system
        
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
        function dmp = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue)
            
%              dmp.can_fun = ExpCanonicalFunction(0.01, 1.0);       
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
            
            DMP_set_kernels_with_EM(dmp, Time, yd_data);
            
        end
        
        
        %% Sets the standard deviations for the activation functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
        function set_stds(dmp, s)
            
            DMP_set_stds(dmp, s);
            
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
        %  ddyd_data need not be sequantial in time.
        function [train_error, F, Fd] = train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g)

            [train_error, F, Fd] = DMP_train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g);
            
%             Ts = Time(2) - Time(1);
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
%                 [dy, dz] = dmp.get_states_dot(x, y, z, y0, g);
%                 
%                 y = y + dy*dt;
%                 z = z + dz*dt;
%                 
%                 
%             end
%             
%             ddy_data = dz_data/dmp.get_v_scale();
%             
%             lineWidth = 1.2;
%             fontsize = 14;
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
        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calc_Fd(dmp, x, y, dy, ddy, y0, g)
            
            v_scale = dmp.get_v_scale();
            Fd = (ddy*v_scale^2 - dmp.goal_attractor(x, y, v_scale*dy, g));
            
        end
        
        
        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp,x)
            
            f = DMP_forcing_term(dmp,x);
            
        end
        
        
        %% Returns the scaling factor of the forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp, x, y0, g)
            
            u = dmp.can_sys_ptr.get_shapeVar(x);
            f_scale = u*(g-y0);
            
        end
        
        
        %% Returns the goal attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp, x, y, z, g)
            
            goal_attr = DMP_goal_attractor(dmp, y, z, g);
            
        end
        
        
        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: initial position.
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
            
        %% Parse extra arguments of the DMP
        %  @param[in] extraArgName: Names of extra arguments.
        %  @param[in] extraArgValue: Values of extra arguemnts.
        function parseExtraArgs(dmp, extraArgName, extraArgValue)

        end
        
    end
end




