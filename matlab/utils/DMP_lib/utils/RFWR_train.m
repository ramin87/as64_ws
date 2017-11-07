%% Trains the DMP weights using RLWR (Receptive Field Weighted Regression)
%  For the i-th data point the k-th weight is updated as w_k = w_k+Psi*P_k*s_i*e_i, 
%  where Psi = exp(-h(k)*(x_i-c(k)).^2), e_i = Fd_i-w_k*s_i, 
%  P_k = P_k - (P_k^2*s_i^2/((l/Psi)+P_k*s_i))/l
%  P_k is initialized in 1 and lambda is a forgetting factor in (0, 1].
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function RFWR_train(dmp, x, s, Fd)
  
  n = length(Fd);
  
  lambda = 0.8;
  P = ones(dmp.N_kernels, 1) * 1000;
  dmp.w = zeros(dmp.N_kernels, 1);
  
  for i = 1:n
      
      Psi = dmp.activation_function(x(i));
      
      error = Fd(i) - dmp.w * s(i);
      
      P = (P - (P.^2 * s(i)^2)./ (lambda./ Psi + P * s(i) ^ 2)) / lambda;
      
      dmp.w = dmp.w + Psi.* P * s(i).* error;
      
  end
  
end



% function RLWR_train(dmp, x, s, Fd)
% 
%   s = s(:);   
%   n = length(Fd);
%   
%   lambda = 1;
%   
%   P = zeros(dmp.N_kernels, 1);
%   Q = zeros(dmp.N_kernels, 1);
%   Gamma = 0.1;
%   dt = 0.001;
%   
%   dmp.w = zeros(dmp.N_kernels, 1);
%   
%   for i = 1:n
%       
%       psi = dmp.activation_function(x(i));
%       
%       w_dot = -Gamma*(P.*dmp.w - Q);
%       P_dot = -lambda*P + psi*s(i)^2;
%       Q_dot = -lambda*Q + psi*Fd(i)*s(i);
%       
%       dmp.w = dmp.w + w_dot*dt;
%       P = P + P_dot*dt;
%       Q = Q + Q_dot*dt;
%       
%   end
%   
% end