%% Trains the DMP weights using LS (Least Squares)
%  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
%  where Psi = exp(-h(k)*(x-c(k)).^2)
%  @param[in] x: Row vector with the values of the phase variable.
%  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
%  @param[in] Fd: Row vector with the desired values of the shape attractor.
function w = LS_train(Psi, s, Fd, zero_tol)
  
    N_kernels = size(Psi,1);
    w = zeros(N_kernels,1);
    
    H = Psi;

    H = H.*repmat(s, size(H,1),1) ./ (repmat(sum(H,1),size(H,1),1) + zero_tol);

%               for i=1:n_data
%                   Psi = exp(-dmp.h.*(x(i)-dmp.c).^2);
%                   Psi = s(i)*Psi / (sum(Psi) + dmp.zero_tol);                 
%                   H(:,i) = Psi(:);
%               end

    w = (Fd/H)';
  
end
