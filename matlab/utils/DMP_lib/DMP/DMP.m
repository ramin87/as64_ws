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

classdef DMP < handle % : public DMP_
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
        %  @param[in] kernel_std_scaling: Scales the std of each kernel by 'kernel_std_scaling' (optional, default = 1.0).
        function set_stds(dmp, kernel_std_scaling)

            DMP_set_stds(dmp, kernel_std_scaling);

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
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp, x, y0, g)
            
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

            if (nargin < 8), z_c=0.0; end
            if (nargin < 7), y_c=0.0; end

            [dy, dz] = DMP_get_states_dot(dmp, x, y, z, y0, g, y_c, z_c);

        end


        %% Returns a column vector with the values of the kernel functions of the DMP
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP
        function psi = kernel_function(dmp,x)

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
