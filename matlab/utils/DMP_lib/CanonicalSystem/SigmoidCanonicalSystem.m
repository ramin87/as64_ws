%% DMP linear canonical system class
%  Implements a 1st order linearly decaying canonical system for a DMP.
%  The canonical system is defined as:
%     tau*dx = -a_x
%  where x is the phase variable and a_x the decay term and tau is scaling a 
%  factor defining the duration of the motion.
%

classdef SigmoidCanonicalSystem < handle
   properties
       x0 % initial value of the phase variable
       a_x % the decay factor of the phase variable
       c % center of the exponential in the sigmoid
       tau % movement duration (can be used to scale temporally the motion)
   end
   
   methods
      %% Linear Canonical System Constructor
      %  param[in] x_end: Value of phase variable at the end of the motion.
      %  param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  param[in] x0: Initial value of the phase variable (optional, default = 1).
      %  param[out] can_sys: canonical system object
      function can_sys = SigmoidCanonicalSystem(x_end, tau, x0)
          
          if (nargin < 2), return; end
          if (nargin < 3), x0=1; end
          
          can_sys.init(x_end, tau, x0);
          
      end 
      
      %% initializes the canonical system
      %  param[in] x_end: Value of phase variable at the end of the motion.
      %  param[in] tau: Movement duration (can be used to scale temporally the motion).
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
          
          can_sys.a_x = 150;
          
          can_sys.c = can_sys.tau - (can_sys.tau/can_sys.a_x)*log((can_sys.x0-x_end)/x_end);
          
      end

      %% Returns the derivative of the canonical system
      %  param[in] X: current value of the phase variable
      %  param[out] dx: derivative
      function dX = get_derivative(can_sys, X)
          
           x = X(1);
           dx = -can_sys.a_x*x*(can_sys.x0-x)/(can_sys.x0*can_sys.get_tau());
           du = dx;

          dX = [dx; du];

      end
      
      %% Returns the output of the canonical system for a continuous time interval
      %  param[in] t: the time interval
      %  param[out] x: the output of the canonical system for the time interval 't'
      function X = get_continuous_output(can_sys, t)
          
          x = can_sys.x0 * 1 ./ (1 + exp((can_sys.a_x/can_sys.tau)*(t-can_sys.tau)));
          u = x;

          X = [x; u];

      end

     
   end
end




