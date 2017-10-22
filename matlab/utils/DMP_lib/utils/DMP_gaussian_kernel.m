%% Returns a column vector with the values of the activation functions of the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] x: phase variable
%  @param[out] psi: column vector with the values of the activation functions of the DMP
function psi = DMP_gaussian_kernel(dmp,x)

    psi = exp(-dmp.h.*((x-dmp.c).^2));

end