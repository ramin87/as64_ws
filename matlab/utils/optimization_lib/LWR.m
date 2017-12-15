%% Trains the DMP weights using LWR (Locally Weighted Regression)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function w = LWR(Psi, s, Fd, zero_tol)
  
  N_kernels = size(Psi,1);
  w = zeros(N_kernels,1);

  for k=1:N_kernels
	  temp = s.*Psi(k,:);
	  w(k) = (temp*Fd(:)) / (temp*s' + zero_tol);
	  %Psi = diag( exp(-dmp.h(k)*(x-dmp.c(k)).^2) );
	  %dmp.w(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + dmp.zero_tol);
  end
  
end

