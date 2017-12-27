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
        dmp % vector 3x1 
        can_sys_ptr % handle (pointer) to the canonical system
    end
    
    methods
        %% DMP constructor
        %  @param[in] DMP_TYPE: the type of DMP, i.e. 'DMP', 'DMP-bio' etc.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function dmp_o = DMPo(DMP_TYPE, N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue)
                
            if (nargin < 5)
                return;
            else
                if (nargin < 6), std_K=1; end
                if (nargin < 7)
                    extraArgName = [];
                    extraArgValue = [];
                end
                dmp_o.init(DMP_TYPE, N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue);
            end
            
        end
        
        
        %% Initializes the DMP
        %  @param[in] DMP_TYPE: the type of DMP, i.e. 'DMP', 'DMP-bio' etc.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function init(dmp_o, DMP_TYPE, N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue)
            
            if (nargin < 7), std_K=1; end
            if (nargin < 8)
                extraArgName = [];
                extraArgValue = [];
            end
            
            dmp_o.can_sys_ptr = can_sys_ptr;
            
            for i=1:3
                if (strcmpi(DMP_TYPE,'DMP'))
                    dmp_o.dmp{i} = DMP();
                elseif (strcmpi(DMP_TYPE,'DMP-bio'))
                    dmp_o.dmp{i} = DMP_bio();
                elseif (strcmpi(DMP_TYPE,'DMP-plus'))
                    dmp_o.dmp{i} = DMP_plus();
                elseif (strcmpi(DMP_TYPE,'DMP-Shannon'))
                    dmp_o.dmp{i} = DMP_Shannon();
                else
                    error('Unsupported DMP type ''%s''', DMP_TYPE);
                end 
                
                dmp_o.dmp{i}.init(N_kernels, a_z, b_z, can_sys_ptr, std_K, extraArgName, extraArgValue);
            end
            
        end
        
        
        %% Sets the centers for the activation functions of the DMP according to the canonical system
        function set_centers(dmp_o)
            
            for i=1:3
                dmp_o.dmp{i}.set_centers();
            end
            
        end
        
        
        %% Sets the kernels of the DMP using the EM algorithm
        function set_kernels_with_EM(dmp_o, Time, yd_data)
            
%             DMP_set_kernels_with_EM(dmp, Time, yd_data)
            
        end
        
        
        %% Sets the standard deviations for the activation functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
        function set_stds(dmp_o, s)
            
            for i=1:3
                dmp_o.dmp{i}.set_stds(s);
            end
            
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
        function [train_error, F, Fd] = train(dmp_o, Time, Qd_data, v_rot_data, dv_rot_data, Q0, Qg)
            
            train_error = zeros(3,1);
            F = zeros(3, length(Time));
            Fd = zeros(3, length(Time));
            
            yd_data = zeros(3, size(Qd_data,2));
            for i=1:size(yd_data,2)
               yd_data(:,i) = -quatLog(quatProd(Qg,quatInv(Qd_data(:,i))));
            end
            
            y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
            
            for i=1:3
                [train_error(i), F(i,:), Fd(i,:)] = dmp_o.dmp{i}.train(Time, yd_data(i,:), v_rot_data(i,:), dv_rot_data(i,:), y0(i), 0);
            end

%             
%             yd_data = zeros(3,length(Time));
%             dyd_data = v_rot_data;
%             ddyd_data = dv_rot_data;
%             
%             Q_data = zeros(size(Qd_data));
%             y_data = zeros(size(yd_data));
%             dy_data = zeros(size(dyd_data));
%             ddy_data = zeros(size(ddyd_data));
%             F_sim = zeros(size(F));
%             
% 
%             x = 0.0;
%             dx = 0;
%             Q0 = Qd_data(:,1);
%             Qg = Qd_data(:,end);
%             dv_rot = zeros(3,1);
%             v_rot = zeros(3,1);
%             Q = Q0;
%             t = 0;
%             Q_robot = Q0;
%             deta = zeros(3,1);
%             eta = zeros(3,1);
%             
%             Ts = Time(2) - Time(1);
%             dt = Ts;
%             
%             tic
%             for i=1:length(Time)
%                 
%                 Q_data(:,i) = Q;
%                 dy_data(:,i) = v_rot;
%                 ddy_data(:,i) = dv_rot;
%                 F_sim(:,i) = dmp_o.shape_attractor(x, Q0, Qg);
%                 
% %                 dmp_o.forcing_term_scaling(x, Q0, Qg).*dmp_o.forcing_term(x);
%                 
% 
%                 %% Orientation DMP
% %                 scaled_forcing_term = dmp_o.forcing_term(x).*dmp_o.forcing_term_scaling(x, Q0, Qg);
%                 
%                 Q_c = 0;
%                 eta_c = 0;
%                 [dQ, deta] = dmp_o.get_states_dot(x, Q, eta, Q0, Qg, Q_c, eta_c);
%                 v_rot_temp = 2*quatProd(dQ,quatInv(Q));
%                 
%                 v_rot = v_rot_temp(2:4);
%                 dv_rot = deta / dmp_o.get_v_scale();
%                          
%                 %% Update phase variable
%                 dx = dmp_o.can_sys_ptr.get_phaseVar_dot(x);
% 
%                 %% Numerical integration
%                 t = t + dt;
%                 
% %                 t
% %                 v_rot
%                 
% %                 v_rot = dyd_data(:,i);
% %                 dv_rot = ddyd_data(:,i);
% %                 eta = dv_rot*dmp_o.get_v_scale();
%                 
% %                 v_rot
% %                 
% %                 pause
%                 
%                 Q = quatProd(quatExp(v_rot*dt), Q);
% %                 Q = Q + dQ*dt;
% %                 Q = Q/norm(Q);
%                 
%                 eta = eta + deta*dt;
%                 
%                 x = x + dx*dt;
%                 
%             end
% 
%             
%             for i=1:length(Time)
%                 yd_data(:,i) = quatLog(quatProd(Qg, quatInv(Qd_data(:,i))));
%                 y_data(:,i) = quatLog(quatProd(Qg, quatInv(Q_data(:,i))));
%             end
%             
% %             plot_training_data(Time, yd_data, dyd_data, ddyd_data);
% 
%             lineWidth = 1.2;
%             plot_signals_and_errorSignal(Time,y_data, Time,yd_data, 'DMP', 'demo', 'Position', lineWidth);
%             plot_signals_and_errorSignal(Time,dy_data, Time,dyd_data, 'DMP', 'demo', 'Velocity', lineWidth);
%             plot_signals_and_errorSignal(Time,ddy_data, Time,ddyd_data, 'DMP', 'demo', 'Acceleration', lineWidth);
%             
%             figure;
%             for i=1:3
%                 subplot(3,1,i);
%                 hold on;
%                 plot(Time, Fd(i,:));
%                 plot(Time, F(i,:));
%                 plot(Time, F_sim(i,:));
%                 legend({'$F_{d_{train}}$','$F_{train}$','$F_{sim}$'},'Interpreter','latex','fontsize',14);
%                 hold off;
%             end
% 
% %             figure;
% %             for i=1:3
% %                 subplot(3,1,i);
% %                 hold on;
% %                 plot(Time, F_sim(i,:));
% %                 legend({'$F_{sim}$'},'Interpreter','latex','fontsize',14);
% %                 hold off;
% %             end
% %             
% %             plot_line_path(y_data, yd_data, 'DMP', 'demo', 2, 10);
%             
% %             error('stop');
            
        end
        
        
        %% Sets the high level training parameters of the DMP
        %  @param[in] train_method: Method used to train the DMP weights.
        %  @param[in] lambda: Forgetting factor for recursive training methods.
        %  @param[in] P_rlwr: Covariance matrix 'P' for recursive training methods.
        function set_training_params(dmp_o, train_method, lambda, P_rlwr)
            
            for i=1:3
                dmp_o.dmp{i}.set_training_params(train_method, lambda, P_rlwr);
            end
            
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
        function [P] = update_weights(dmp_o, x, y, dy, ddy, y0, g, P, lambda)
            
            P = RLWR_update(dmp_o, x, y, dy, ddy, y0, g, P, lambda);
            
        end
        
        %% Calculates the desired values of the scaled forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calc_Fd(dmp_o, x, Q, v_rot, ddv_rot, Q0, Qg)
            
%             v_scale = dmp_o.get_v_scale();
%             Fd = (ddv_rot*v_scale^2 - dmp_o.goal_attractor(x, Q, v_scale*v_rot, Qg));

            y = -quatLog(quatProd(Qg,quatInv(Q)));
            y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
            g = zeros(3,1);
            dy = v_rot;
            ddy = ddv_rot;
            
            Fd = zeros(3, 1);
            for i=1:3
                Fd(i) = dmp_o.dmp{i}.calc_Fd(x, y(i), dy(i), ddy(i), y0(i), g);
            end
            
        end
        
        
        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp_o, x)
            
            f = zeros(3,1);
            for i=1:3
                f(i) = dmp_o.dmp{i}.forcing_term(x);
            end
            
        end
        
        %% Returns the scaling factor of the forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp_o, x, Q0, Qg)
            
%             u = dmp_o.can_sys_ptr.get_shapeVar(x);
%             f_scale = quatLog(quatProd(Qg,quatInv(Q0)))*u;
            
            y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
            g = zeros(3,1);
            
            f_scale = zeros(3,1);
            for i=1:3
                f_scale(i) = dmp_o.dmp{i}.forcing_term_scaling(x, y0(i), g(i));
            end
            
        end
        
        %% Returns the goal attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp_o, x, Q, eta, Qg)
            
            goal_attr = zeros(3, 1);
            y = -quatLog(quatProd(Qg, quatInv(Q)));
            for i=1:3
                goal_attr(i) = dmp_o.dmp{i}.goal_attractor(x, y(i), eta(i), 0);
            end
            
        end
        
        
        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp_o, x, Q0, Qg)

            y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
            g = zeros(3,1);
            
            shape_attr = zeros(3,1);
            for i=1:3
                shape_attr(i) = dmp_o.dmp{i}.shape_attractor(x, y0(i), g(i));
            end
            
%             f = dmp_o.forcing_term(x);
%             f_scale = dmp_o.forcing_term_scaling(x, Q0, Qg);
%             shape_attr = f .* f_scale;
            
        end
        
        
        %% Returns the derivatives of the DMP states
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the DMP.
        %  @param[in] z: \a z state of the DMP.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Goal position.
        %  @param[in] y_c: Coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: Coupling term for the dynamical equation of the \a z state.
        %  @param[out] dy: Derivative of the \a y state of the DMP.
        %  @param[out] dz: Derivative of the \a z state of the DMP.
        function [dQ, deta] = get_states_dot(dmp_o, x, Q, eta, Q0, Qg, Q_c, eta_c)
            
            if (nargin < 10), eta_c=0; end
            if (nargin < 9), Q_c=0; end
            %if (nargin < 8), g=g0; end
            
            v_scale = dmp_o.get_v_scale();
            shape_attr = dmp_o.shape_attractor(x, Q0, Qg);
            goal_attr = dmp_o.goal_attractor(x, Q, eta, Qg);
            
            deta = ( goal_attr + shape_attr + eta_c) / v_scale;
            dQ = 0.5*quatProd([0; (eta+Q_c)/ v_scale], Q);
            
        end
        
        
        %% Returns a column vector with the values of the activation functions of the DMP
        %  @param[in] x: phase variable.
        %  @param[out] psi: column vector with the values of the activation functions of the DMP.
        function psi = activation_function(dmp_o, x)
            
            psi = dmp_o.dmp{1}.activation_function(x);
            
        end
        
        
        %% Returns the scaling factor of the DMP
        %  @param[out] v_scale: The scaling factor of the DMP.
        function v_scale = get_v_scale(dmp_o)
            
            v_scale = dmp_o.dmp{1}.get_v_scale();
            
        end
        
        
        %% Returns the time cycle of the DMP
        %  @param[out] tau: The time cycle of the DMP.
        function tau = get_tau(dmp_o)
            
            tau = dmp_o.dmp{1}.get_tau();
            
        end
        
        
    end
end




