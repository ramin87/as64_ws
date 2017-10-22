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
%  Optinally the canonical system can include a phase stopping term to halt
%  the evolution of the phase variable when the reference position produced
%  by the DMP and the actual position differ. In this case the canonical
%  system formulation is as follows:
%
%     tau*du = a_u*(-b_u-u) * 1/(1 + a_pu*norm(y-y_real)^2)
%
%   where:
%     'a_pu' is a constant determining how fast the phase variable stops
%     'y' is the reference position produced by the DMP
%     'y_real' is the actual position
%

classdef DMP_spring_damper_canonical_system < handle
   properties
       a_u % param of the spring-damper
       b_u % param of the spring-damper
       tau % movement duration (can be used to scale temporally the motion)
       
       a_pu % parameter for the phase stop term
       USE_PHASE_STOP % flag indicating to use phase stopping
   end
   
   methods
      %% initializes the canonical system
      %
      %  param[in] x_end: value of phase variable at the end of the motion
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %  param[in] USE_PHASE_STOP: flag indicating to use phase stopping
      %  param[in] a_px: constant determining how fast the phase variable stops
      %
      %  param[out] can_sys: canonical system object
      %
      function can_sys = DMP_spring_damper_canonical_system(x_end, tau, USE_PHASE_STOP, a_pu)
          
          if (nargin < 3)
              USE_PHASE_STOP = false;
              a_pu = 0;
          end
          
          can_sys.init(x_end, tau, USE_PHASE_STOP, a_pu);
          
      end 
      
      %% initializes the canonical system
      %
      %  param[in] a_x: decay factor of the phase variable
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %  param[in] USE_PHASE_STOP: flag indicating to use phase stopping
      %  param[in] a_px: constant determining how fast the phase variable stops
      %
      function init(can_sys, x_end, tau, USE_PHASE_STOP, a_pu)
          
          if (nargin < 4)
              USE_PHASE_STOP = false;
              a_pu = 0;
          end
          
          can_sys.set_can_sys_params(x_end);
          can_sys.tau = tau;
          can_sys.USE_PHASE_STOP = USE_PHASE_STOP;
          can_sys.a_pu = a_pu;
      end
      
      %% sets the canonical system's time constants based on the value of the phase variable at the end of the movement
      %
      %  param[in] x_end: value of the phase variable at the end of the movement (t = tau)
      %
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

      %% Returns the output of the canonical system for a single time step using Euler numerical integration
      %
      %  param[in] dt: integration time step
      %  param[in] x: current value of the phase variable
      %  param[in] y: DMP reference position (optional param when phase stop is active)
      %  param[in] y_real: actual position (optional param when phase stop is active)
      %
      %  param[out] x: next value of the phase variable
      %
      function X_out = get_single_step_output(can_sys, dt, X_in, y, y_real)
          
          x = X_in(1);
          dx = X_in(2);
          
          du = can_sys.a_u*(-can_sys.b_u*x - dx*can_sys.tau) / can_sys.tau^2;

          ddx = can_sys.a_u*(-can_sys.b_u*x - dx*can_sys.tau) / can_sys.tau^2;
          
          if (can_sys.USE_PHASE_STOP && (nargin == 5))  
              k = norm(y-y_real) / (norm(y) + norm(y_real) + 1e-30);
              ddx = (1-k)*ddx - can_sys.a_pu*k*dx;
          end
          
          x = x + dx*dt;
          dx = dx + ddx*dt;

          X_out = [x; dx];

      end
      
      %% Returns the output of the canonical system for a continuous time interval
      %
      %  param[in] t: the time interval
      %  param[in] x0: initial value of the phase variable
      %
      %  param[out] X: the output of the canonical system for the timeinterval 't'.
      %                X has so many rows as the order of the canonical. If
      %                it has order n, then the output is the 0-th till the
      %                (n-1)-th order output of the canonical system.
      %
      function X = get_continuous_output(can_sys, t, x0)
          
          t = t(:)';
          
          a = can_sys.a_u/(2*can_sys.tau);
          exp_at = exp(-a*t);
          
          x = x0 * (1+a*t).*exp_at;

          dx = x0 * ( -a^2*t.*exp_at );
          
          u = dx;% / can_sys.tau;
          
          X = [x; u];

      end

     
   end
end




