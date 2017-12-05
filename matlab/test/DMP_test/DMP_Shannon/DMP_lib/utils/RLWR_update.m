%% Updates the DMP weights using RLWR (Recursive Locally Weighted Regression)
%  @param[in] dmp: DMP object.
%  @param[in] x: The phase variable.
%  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
%  @param[in] y: Position.
%  @param[in] dy: Velocity.
%  @param[in] ddy: Acceleration.
%  @param[in] y0: Initial position.
%  @param[in] g0: Final goal.
%  @param[in] g: Current goal.
%  @param[in,out] P: \a P matrix of RLWR.
%  @param[in] lambda: Forgetting factor.
function [P] = RLWR_update(dmp, x, u, y, dy, ddy, y0, g0, g, P, lambda)

    Fd = dmp.calc_Fd(y, dy, ddy, u, y0, g0, g);
    s = dmp.forcing_term_scaling(u, y0, g0);
    psi = dmp.activation_function(x);
    
    error = Fd - dmp.w*s;
    
    P = (P - (P.^2.*s.^2) ./ (lambda./psi + P.*s.^2)) / lambda;
    dmp.w = dmp.w + psi.*P.*s.*error; 
    
end


