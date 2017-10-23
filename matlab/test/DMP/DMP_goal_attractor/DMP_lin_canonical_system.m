%% DMP linear canonical system class
%  Implements a 1st order linearly decaying canonical system for a DMP.
%  The canonical system is defined as:
%     tau*dx = -a_x
%  where x is the phase variable and a_x the decay term and tau is scaling a 
%  factor defining the duration of the motion.
%
%  Optinally the canonical system can include a phase stopping term to halt
%  the evolution of the phase variable when the reference position produced
%  by the DMP and the actual position differ. In this case the canonical
%  system formulation is as follows:
%
%     dx = -ax/tau * 1/(1 + a_px*norm(y-y_real)^2)
%
%   where:
%     'a_px' is a constant determining how fast the phase variable stops
%     'y' is the reference position produced by the DMP
%     'y_real' is the actual position
%

classdef DMP_lin_canonical_system < handle
   properties
       a_x % the decay factor of the phase variable
       tau % movement duration (can be used to scale temporally the motion)
       
       a_px % parameter for the phase stop term
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
      function can_sys = DMP_lin_canonical_system(x_end, tau, USE_PHASE_STOP, a_px)
          
          if (nargin < 3)
              USE_PHASE_STOP = false;
              a_px = 0;
          end
          
          can_sys.init(x_end, tau, USE_PHASE_STOP, a_px);
          
      end 
      
      %% initializes the canonical system
      %
      %  param[in] a_x: decay factor of the phase variable
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %  param[in] USE_PHASE_STOP: flag indicating to use phase stopping
      %  param[in] a_px: constant determining how fast the phase variable stops
      %
      function init(can_sys, x_end, tau, USE_PHASE_STOP, a_px)
          
          if (nargin < 4)
              USE_PHASE_STOP = false;
              a_px = 0;
          end
          
          can_sys.set_can_sys_params(x_end);
          can_sys.tau = tau;
          can_sys.USE_PHASE_STOP = USE_PHASE_STOP;
          can_sys.a_px = a_px;
      end
      
      %% sets the canonical system's time constants based on the value of the phase variable at the end of the movement
      %
      %  param[in] x_end: value of the phase variable at the end of the movement (t = tau)
      %
      function set_can_sys_params(can_sys, x_end)
          
          can_sys.a_x = 1-x_end;
          
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
      function [x, dx] = get_single_step_output(can_sys, dt, x, y, y_real)
          
          dx = -can_sys.a_x/can_sys.tau;
          
          if (can_sys.USE_PHASE_STOP && (nargin == 5))
              dx = dx * 1/(1+can_sys.a_px*norm(y-y_real)^2);
          end
          
          x = x + dx*dt;
          
          if (x < 0), x=0; end

      end
      
      %% Returns the output of the canonical system for a continuous time interval
      %
      %  param[in] t: the time interval
      %  param[in] x0: initial value of the phase variable
      %
      %  param[out] x: the output of the canonical system for the time interval 't'
      %
      function x = get_continuous_output(can_sys, t, x0)
          
          x = x0 - can_sys.a_x*t/can_sys.tau;

      end

     
   end
end




