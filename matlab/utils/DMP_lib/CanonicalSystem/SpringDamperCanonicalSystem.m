%% DMP spring-damper canonical system class
%  Implements a 2nd order spring-damper canonical system for a DMP.
%  The canonical system is defined as:
%     tau*du = a_u*(-b_u*x - u)
%     tau*dx = u
%  where:
%     'x' is the phase variable
%     'a_u', 'b_u' are the spring-damper params 
%     'tau' is a scaling factor defining the duration of the motion.
%
%

classdef SpringDamperCanonicalSystem < handle
   properties
       x0 % initial value of the phase variable
       a_u % param of the spring-damper
       b_u % param of the spring-damper
       tau % movement duration (can be used to scale temporally the motion)
   end
   
   methods
      %% Spring-Damper Canonical System Constructor
      %  param[in] x_end: Value of phase variable at the end of the motion.
      %  param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  param[in] x0: Initial value of the phase variable (optional, default = 1).
      %  param[out] can_sys: canonical system object
      function can_sys = SpringDamperCanonicalSystem(x_end, tau, x0)
          
          if (nargin < 2), return; end
          if (nargin < 3), x0=1; end
          
          can_sys.init(x_end, tau, x0);
          
      end 
      
      %% initializes the canonical system
      %  param[in] x_end: value of phase variable at the end of the motion
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %  param[in] x0: Initial value of the phase variable (optional, default = 1).
      function init(can_sys, x_end, tau, x0)
          
          if (nargin < 4), x0=1; end
          
          can_sys.x0 = x0;
          can_sys.set_tau(tau);
          can_sys.set_can_sys_params(x_end);

      end

      %% sets the canonical system's cycle time
      %  param[in] tau: the canonical system's cycle time
      function set_tau(can_sys, tau)
          
          can_sys.tau = tau;
          
      end
      
      %% returns the canonical system's cycle time
      %  param[out] tau: the canonical system's cycle time
      function tau = get_tau(can_sys)
          
          tau = can_sys.tau;
          
      end
      
      
      %% sets the canonical system's time constants based on the value of the phase variable at the end of the movement
      %  param[in] x_end: value of the phase variable at the end of the movement (t = tau)
      function set_can_sys_params(can_sys, x_end)
          
          g = @(x) (1 + x/2)*exp(-x/2) - x_end;
          dg = @(x) -(x/2).^2*exp(-x/2);
          
          a1 = 0;
          a3 = 800;
          a2 = (a1+a3)/2;
          a2_prev = a2;
          
          tol_stop = 1e-12;

          iters = 0;

          while (true)
              a2_prev = a2;

              a2 = (a1+a3)/2;

              g1 = g(a1);

              g2 = g(a2);

              g3 = g(a3);

              if (g1*g2<0)
                  a3 = a2;
              elseif (g2*g3<0)
                  a1 = a2;
              else
                  error('No feasible solution exists in this interval');
              end

              a2 = (a1+a3)/2;

              iters = iters+1;

              if (abs(a2-a2_prev) < tol_stop), break; end
          end
          
          can_sys.a_u = a2;
          can_sys.b_u = can_sys.a_u/4;
          
      end

      %% Returns the derivative of the canonical system
      %  param[in] x: current value of the phase variable
      %  param[out] dx: derivative
      function dX = get_derivative(can_sys, X_in)
          
          x = X_in(1);
          u = X_in(2);
          
          du = can_sys.a_u*(-can_sys.b_u*x - u) / can_sys.get_tau();
          dx = u / can_sys.get_tau();
          
          dX = [dx; du];

      end
      
      %% Returns the output of the canonical system for a continuous time interval
      %  param[in] t: the time interval
      %  param[out] X: the output of the canonical system for the timeinterval 't'.
      %                X has so many rows as the order of the canonical. If
      %                it has order n, then the output is the 0-th till the
      %                (n-1)-th order output of the canonical system.
      function X = get_continuous_output(can_sys, t)
          
          t = t(:)';
          
          a = can_sys.a_u/(2*can_sys.get_tau());
          exp_at = exp(-a *t);         
          % to do
          % check if x < 0*t);
          
          x = can_sys.x0 * (1+a*t).*exp_at;

          dx = can_sys.x0 * ( -a^2*t.*exp_at );
          
          u = dx * can_sys.get_tau();
          
          X = [x; u];

      end

     
   end
end




