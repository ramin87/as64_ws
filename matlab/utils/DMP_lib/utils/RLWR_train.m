% %% Trains the DMP weights using RLWR (Recursive Locally Weighted Regression)
% %  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
% %  where Psi = exp(-h(k)*(x-c(k)).^2)
% %  @param[in] x: Row vector with the values of the phase variable.
% %  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
% %  @param[in] Fd: Row vector with the desired values of the shape attractor.
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




%% Trains the DMP weights using RLWR (Recursive Locally Weighted Regression)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function RLWR_train(dmp, x, s, Fd)

  s = s(:);   
  n = length(Fd);
  
  lambda = 0.987;
  P = ones(dmp.N_kernels, 1)*1;
  dmp.w = zeros(dmp.N_kernels, 1);
  
%   P_data = [];
%   U_data = [];
  
  for i = 1:n
      
      %u_data = zeros(dmp.N_kernels,1);
      
      Psi = dmp.activation_function(x(i));
      for k = 1:dmp.N_kernels
          
          psi = Psi(k);
          error = Fd(i) - dmp.w(k) * s(i);
          
          P(k) = (P(k) - (P(k)^2 * s(i)^2) / (lambda / psi + P(k) * s(i) ^ 2)) / lambda;
          dmp.w(k) = dmp.w(k) + psi * P(k) * s(i) * error;
          
         %u_data(k) = psi * P(k) * s(i) * error;
          
      end
      
%       U_data = [U_data u_data];
%       P_data = [P_data P];
      
  end
  
%   plot(P_data');
%   ind = [10:20 90:100];
%   figure
%   subplot(2,1,1);
%   plot(U_data(10:20,:)');
%   subplot(2,1,2);
%   plot(U_data(90:100,:)');
  
end


