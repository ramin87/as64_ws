%% Trains the DMP weights using LS (Least Squares)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function LS_train(dmp, x, s, Fd)
  
  s = s(:);   
  
  n_data = length(x);
  
  H = zeros(dmp.N_kernels, n_data);

  for k=1:dmp.N_kernels
	  Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
	  H(k,:) = Psi; 
  end
  H = H.*repmat(s',size(H,1),1) ./ (repmat(sum(H,1),size(H,1),1) + dmp.zero_tol);

%               for i=1:n_data
%                   Psi = exp(-dmp.h.*(x(i)-dmp.c).^2);
%                   Psi = s(i)*Psi / (sum(Psi) + dmp.zero_tol);                 
%                   H(:,i) = Psi(:);
%               end

  dmp.w = (Fd/H)';
  
end
