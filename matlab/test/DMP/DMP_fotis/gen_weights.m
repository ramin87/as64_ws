function w = gen_weights(f_target, Ts, N)

    % calculate x and psi
    [x_track, ~] = rollout(Ts, N);
    
    if length(w)==1
       w = w*ones(1,N);  
    end
    
    for n=1:N
       Psi(n) = exp(-h(n)*(x_track-c(n))^2);
    end
    
%     psi_track = self.gen_psi(x_track)
% 
%     % efficiently calculate BF weights using weighted linear regression
%     self.w = np.zeros((self.dmps, self.bfs))
%     for d in range(self.dmps):
%         % spatial scaling term
%         k = (self.goal[d] - self.y0[d])
%         for b in range(self.bfs):
%             numer = np.sum(x_track * psi_track[:, b] * f_target[:, d])
%             denom = np.sum(x_track**2 * psi_track[:, b])
%             self.w[d, b] = numer / (k * denom)