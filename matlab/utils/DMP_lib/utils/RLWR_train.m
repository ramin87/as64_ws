%% Trains the DMP weights using RLWR (Recursive Locally Weighted Regression)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function RLWR_train(dmp, x, s, Fd)

  s = s(:);   
  number_of_points = length(Fd);
  
  lambda = 1;
  P = ones(dmp.N_kernels, 1);
  dmp.w = zeros(dmp.N_kernels, 1);
  
  for i = 1:number_of_points
      
      for k = 1:dmp.N_kernels
          
          Psi = exp(-dmp.h(k) * (x(i) - dmp.c(k)).^2);
          error = Fd(i) - dmp.w(k) * s(i);
          
          P(k) = (P(k) - (P(k)^2 * s(i)^2) / (lambda / Psi + P(k) * s(i) ^ 2)) / lambda;
          dmp.w(k) = dmp.w(k) + Psi * P(k) * s(i) * error;
          
          
      end
      
  end
  
end