%% Returns the time cycle of the DMP
%  @param[in] dmp: The DMP object.
%  @param[out] tau: The time cycle of the DMP.
function tau = DMP_get_tau(dmp)

    tau = dmp.can_sys_ptr.get_tau();

end