%% Initializes the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] N_kernels: the number of kernels
%  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
%  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
%  @param[in] canClock_ptr: Pointer to a DMP canonical system object.
%  @param[in] shapeAttrGating_ptr: Pointer to gating function for the shape attractor.
%  @param[in] goalAttrGating_ptr: Pointer to gating function for the goal attractor.
%  @param[in] kernel_std_scaling: Scales the std of each kernel (optional, default = 1.0).
%  @param[in] extraArgName: Names of extra arguments.
%  @param[in] extraArgValue: Values of extra arguemnts.
function DMP_init(dmp, N_kernels, a_z, b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling, extraArgName, extraArgValue)

    dmp.zero_tol = 1e-30;%realmin;

    if (nargin < 8), kernel_std_scaling = 1.0; end

    dmp.shapeAttrGating_ptr = shapeAttrGating_ptr;
    dmp.goalAttrGating_ptr = goalAttrGating_ptr;

    dmp.N_kernels = N_kernels;
    dmp.a_z = a_z;
    dmp.b_z = b_z;
    dmp.canClock_ptr = canClock_ptr;

    dmp.parseExtraArgs(extraArgName, extraArgValue);

%     tau = dmp.get_tau();
%     if (tau > 1)
%       dmp.a_s = 1 / (dmp.canClock_ptr.tau^2);
%     else
%       dmp.a_s = (dmp.canClock_ptr.tau^2);
%     end
% 	dmp.a_s = 1.0/10;
    dmp.a_s = 1.0/canClock_ptr.get_tau();

    dmp.w = zeros(dmp.N_kernels,1); %rand(dmp.N_kernels,1);
    dmp.set_centers();
    dmp.set_stds(kernel_std_scaling);

    trainParamsName = {'lambda', 'P_cov'};
    trainParamsValue = {0.99, 1e6};
    dmp.set_training_params('LWR', trainParamsName, trainParamsValue);

end
