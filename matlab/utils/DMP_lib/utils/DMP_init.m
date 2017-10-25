%% Initializes the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] N_kernels: the number of kernels
%  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
%  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
%  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
%  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
function DMP_init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K)

    dmp.zero_tol = realmin;

    if (nargin < 5), std_K = 1; end

    dmp.N_kernels = N_kernels;
    dmp.a_z = a_z;
    dmp.b_z = b_z;
    dmp.can_sys_ptr = can_sys_ptr;

    tau = dmp.get_tau();
    if (tau > 1)
      dmp.a_s = 1 / (dmp.can_sys_ptr.tau^2);
    else
      dmp.a_s = (dmp.can_sys_ptr.tau^2);
    end

    dmp.w = zeros(dmp.N_kernels,1); %rand(dmp.N_kernels,1);
    dmp.set_centers();
    dmp.set_stds(std_K);

end