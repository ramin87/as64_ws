%% Trains the DMP weights using LWR (Locally Weighted Regression)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function LWR_train(dmp, x, s, Fd)

  s = s(:);  
  
  for k=1:dmp.N_kernels
	  Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
	  temp = s'.*Psi;
	  dmp.w(k) = (temp*Fd(:)) / (temp*s + dmp.zero_tol);

	  %Psi = diag( exp(-dmp.h(k)*(x-dmp.c(k)).^2) );
	  %dmp.w(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + dmp.zero_tol);
  end
  
end

