%% DMP Cartesian Position class
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
%     f: the forcing term defined by the weighted sum of the kernel
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

classdef DMP_CartPos < handle
    properties
        dmp % vector 3x1
        canClock_ptr % handle (pointer) to the canonical clock
    end

    methods
        %% DMP constructor
        %  @param[in] DMP_TYPE: the type of DMP, i.e. 'DMP', 'DMP-bio' etc.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_scale_factor: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function dmp_CartPos = DMP_CartPose(DMP_TYPE, N_kernels, a_z, b_z, canClock_ptr, std_scale_factor, extraArgName, extraArgValue)

            if (nargin < 5)
                return;
            else
                if (nargin < 6), std_scale_factor=1; end
                if (nargin < 7)
                    extraArgName = [];
                    extraArgValue = [];
                end
                dmp_CartPos.init(DMP_TYPE, N_kernels, a_z, b_z, canClock_ptr, std_scale_factor, extraArgName, extraArgValue);
            end

        end


        %% Initializes the DMP
        %  @param[in] DMP_TYPE: the type of DMP, i.e. 'DMP', 'DMP-bio' etc.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] std_scale_factor: Scales the std of each kernel (optional, default = 1).
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        function init(dmp_CartPos, DMP_TYPE, N_kernels, a_z, b_z, canClock_ptr, std_scale_factor, extraArgName, extraArgValue)

            if (nargin < 7), std_scale_factor=1; end
            if (nargin < 8)
                extraArgName = [];
                extraArgValue = [];
            end

            dmp_CartPos.canClock_ptr = canClock_ptr;

            for i=1:3
                if (strcmpi(DMP_TYPE,'DMP'))
                    dmp_CartPos.dmp{i} = DMP();
                elseif (strcmpi(DMP_TYPE,'DMP-bio'))
                    dmp_CartPos.dmp{i} = DMP_bio();
                elseif (strcmpi(DMP_TYPE,'DMP-plus'))
                    dmp_CartPos.dmp{i} = DMP_plus();
                elseif (strcmpi(DMP_TYPE,'DMP-Shannon'))
                    dmp_CartPos.dmp{i} = DMP_Shannon();
                else
                    error('Unsupported DMP type ''%s''', DMP_TYPE);
                end

                dmp_CartPos.dmp{i}.init(N_kernels, a_z, b_z, canClock_ptr, std_scale_factor, extraArgName, extraArgValue);
            end

        end


        %% Sets the centers for the kernel functions of the DMP according to the canonical system
        function set_centers(dmp_CartPos)

            for i=1:3
                dmp_CartPos.dmp{i}.set_centers();
            end

        end


        %% Sets the standard deviations for the kernel functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
        function set_stds(dmp_CartPos, s)

            for i=1:3
                dmp_CartPos.dmp{i}.set_stds(s);
            end

        end


        %% Trains the DMP
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] Y_data: Matrix with the Cartesian position in each column.
        %  @param[in] dY_data: Matrix with the Cartesian velocity in each column.
        %  @param[in] ddY_data: Matrix with the Cartesian acceleration in each column.
        %  @param[in] Y0: Initial Cartesian position.
        %  @param[in] Yg: Target-goal Cartesian position.
        %
        %  \note The timestamps in \a Time and the corresponding position,
        %  velocity and acceleration data in \a Y_data, \a dY_data and \a
        %  ddY_data need not be sequantial in time.
        function [train_error, F, Fd] = train(dmp_CartPos, Time, Y_data, dY_data, ddY_data, Y0, Yg)

            train_error = zeros(3,1);
            F = zeros(3, length(Time));
            Fd = zeros(3, length(Time));

            for i=1:3
                [train_error(i), F(i,:), Fd(i,:)] = dmp_CartPos.dmp{i}.train(Time, Y_data(i,:), dY_data(i,:), ddY_data(i,:), Y0(i), Yg(i));
            end

        end


        %% Sets the high level training parameters of the DMP
        %  @param[in] train_method: Method used to train the DMP weights.
        %  @param[in] extraArgName: Names of extra arguments (optional, default = []).
        %  @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
        %
        %  \remark The extra argument names can be the following:
        %  'lambda': Forgetting factor for recursive training methods.
        %  'P_cov': Initial value of the covariance matrix for recursive training methods.
        function set_training_params(dmp_o, train_method, extraArgName, extraArgValue)

            for i=1:3
                dmp_o.dmp{i}.set_training_params(train_method, extraArgName, extraArgValue);
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
        function [P] = update_weights(dmp_CartPos, x, y, dy, ddy, y0, g, P, lambda)

            P = RLWR_update(dmp_CartPos, x, y, dy, ddy, y0, g, P, lambda);

        end

        %% Calculates the desired values of the scaled forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] Y: Cartesian position.
        %  @param[in] dY: Cartesian velocity.
        %  @param[in] ddY: Cartesian acceleration.
        %  @param[in] Y0: Initial Cartesian position.
        %  @param[in] Yg: Goal Cartesian position.
        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calc_Fd(dmp_CartPos, x, Y, dY, ddY, Y0, Yg)

            Fd = zeros(3, 1);
            for i=1:3
                Fd(i) = dmp_CartPos.dmp{i}.calc_Fd(x, Y(i), dY(i), ddY(i), Y0(i), Yg(i));
            end

        end


        %% Returns the forcing term of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcing_term(dmp_CartPos, x)

            f = zeros(3,1);
            for i=1:3
                f(i) = dmp_CartPos.dmp{i}.forcing_term(x);
            end

        end

        %% Returns the scaling factor of the forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] Y0: Initial Cartesian position.
        %  @param[in] Yg: Goal Cartesian position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcing_term_scaling(dmp_CartPos, x, Y0, Yg)

            f_scale = zeros(3,1);
            for i=1:3
                f_scale(i) = dmp_CartPos.dmp{i}.forcing_term_scaling(x, Y0(i), Yg(i));
            end

        end

        %% Returns the goal attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] Y: \a y state of the DMP.
        %  @param[in] Z: \a z state of the DMP.
        %  @param[in] Yg: Goal Cartesian position.
        %  @param[out] goal_attr: The goal attractor of the DMP.
        function goal_attr = goal_attractor(dmp_CartPos, x, Y, Z, Yg)

            goal_attr = zeros(3, 1);
            for i=1:3
                goal_attr(i) = dmp_CartPos.dmp{i}.goal_attractor(x, Y(i), Z(i), Yg(i));
            end

        end


        %% Returns the shape attractor of the DMP.
        %  @param[in] x: The phase variable.
        %  @param[in] Y0: Initial Cartesian position.
        %  @param[in] Yg: Goal Cartesian position.
        %  @param[out] shape_attr: The shape_attr of the DMP.
        function shape_attr = shape_attractor(dmp_CartPos, x, Y0, Yg)

            shape_attr = zeros(3,1);
            for i=1:3
                shape_attr(i) = dmp_CartPos.dmp{i}.shape_attractor(x, Y0(i), Yg(i));
            end

        end


        %% Returns the derivatives of the DMP states
        %  @param[in] x: The phase variable.
        %  @param[in] Y: \a y state of the DMP.
        %  @param[in] Z: \a z state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yg: Goal position.
        %  @param[in] Y_c: Coupling term for the dynamical equation of the \a y state.
        %  @param[in] Z_c: Coupling term for the dynamical equation of the \a z state.
        %  @param[out] dY: Derivative of the \a y state of the DMP.
        %  @param[out] dZ: Derivative of the \a z state of the DMP.
        function [dY, dZ] = get_states_dot(dmp_CartPos, x, Y, Z, Y0, Yg, Y_c, Z_c)

            if (nargin < 8), Z_c=0; end
            if (nargin < 7), Y_c=0; end

            v_scale = dmp_CartPos.get_v_scale();
            shape_attr = dmp_CartPos.shape_attractor(x, Y0, Yg);
            goal_attr = dmp_CartPos.goal_attractor(x, Y, Z, Yg);

            dZ = ( goal_attr + shape_attr + Z_c) / v_scale;
            dY = ( Z + Y_c) / v_scale;

        end


        %% Returns a column vector with the values of the kernel functions of the DMP
        %  @param[in] x: phase variable.
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP.
        function psi = kernel_function(dmp_CartPos, x)

            psi = dmp_CartPos.dmp{1}.kernel_function(x);

        end


        %% Returns the scaling factor of the DMP
        %  @param[out] v_scale: The scaling factor of the DMP.
        function v_scale = get_v_scale(dmp_CartPos)

            v_scale = dmp_CartPos.dmp{1}.get_v_scale();

        end


        %% Returns the time cycle of the DMP
        %  @param[out] tau: The time cycle of the DMP.
        function tau = get_tau(dmp_CartPos)

            tau = dmp_CartPos.dmp{1}.get_tau();

        end


    end
end
