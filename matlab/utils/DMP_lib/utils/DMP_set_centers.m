%% Sets the centers for the activation functions of the DMP according to the canonical system
%  @param[in] dmp: The DMP object.
function DMP_set_centers(dmp)

    t = ((1:dmp.N_kernels)-1)/(dmp.N_kernels-1);
    x = dmp.can_sys_ptr.get_phaseVar(t*dmp.can_sys_ptr.get_tau());
    dmp.c = x(1,:)';

end