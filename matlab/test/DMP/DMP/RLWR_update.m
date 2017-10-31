%% Trains the DMP weights using RLWR (Recursive Locally Weighted Regression)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function [P] = RLWR_update(dmp, x, s, Fd, P, lambda)

    Psi = dmp.activation_function(x);
    error = Fd - dmp.w*s;
    P = (P - (P.^2*s^2) ./ (lambda./Psi + P*s^2)) / lambda;
    dmp.w = dmp.w + Psi.*P*s.*error; 
  
end


