%% Returns the forcing term of the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] x: The phase variable.
%  @param[out] f: The normalized weighted sum of Gaussians.
function f = DMP_forcing_term(dmp,x)

    Psi = dmp.activation_function(x);
    
    f = dot(Psi,dmp.w) / (sum(Psi)+dmp.zero_tol); % add 'zero_tol' to avoid numerical issues

end