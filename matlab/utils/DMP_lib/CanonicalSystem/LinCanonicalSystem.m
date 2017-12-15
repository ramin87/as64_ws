%% DMP Linear Canonical System class
%  Implements a 1st order linearly decaying canonical system for a DMP.
%  The outputs of the canonical system are:
%     x: the phase variable (substitute for time 't')
%     u: the shape variable (multiplies the DMP's forcing term to ensure 
%                           that the forcing term ultimately becomes zero)
%  The canonical system is defined as:
%     x_dot = df_clock(x)
%     u = u0 - a_u*x;
%  where f_clock(x) and df_clock(x) is the function and derivative of the 
%  DMP's clock. The clock serves as a substiture for time starting at 
%  x(t=0) = 0 and ending at x(t=tau) = 1, where 'tau' is the movements 
%  total duration.
%  u is the canonical system's output used to ensure that the forcing term
%  of the DMP vanishes at t=tau. 
%

classdef LinCanonicalSystem < handle
   properties
       u0 % initial value of the shape variable
       a_u % the decay factor of the shape variable
       can_clock % clock, substitutes time, starts at 0, ends at 1
   end
   
   methods
      %% Linear Canonical System Constructor.
      %  @param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  @param[in] u_end: Value of the shape variable at the end of the motion.
      %  @param[in] u0: Initial value of the shape variable (optional, default = 1).
      %  @param[out] can_sys: Canonical system object.
      function can_sys = LinCanonicalSystem(tau, u_end, u0)
          
          can_sys.can_clock = CanonicalClock();

          if (nargin < 2), return; end
          if (nargin < 3), u0=1; end
          
          can_sys.init(tau, u_end, u0);
          
      end 
      
      %% Initializes the canonical system.
      %  @param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  @param[in] u_end: Value of phase variable at the end of the motion.
      %  @param[in] u0: Initial value of the phase variable (optional, default = 1.0).
      function init(can_sys, tau, u_end, u0)
          
          if (nargin < 3), u_end=0.0; end
          if (nargin < 4), u0=1.0; end
          
          can_sys.can_clock.init(tau);

          can_sys.u0 = u0;
          can_sys.set_can_sys_params(u_end);
          
      end
      
      %% Sets the canonical system's cycle time.
      %  @param[in] tau: The canonical system's cycle time.
      function set_tau(can_sys, tau)
          
          can_sys.can_clock.set_tau(tau);
          
      end
      
      %% Returns the canonical system's cycle time.
      %  @param[out] tau: The canonical system's cycle time.
      function tau = get_tau(can_sys)
          
          tau = can_sys.can_clock.get_tau();
          
      end
      
      %% Sets the canonical system's time constants based on the value of 
      %% the phase variable at the end of the movement.
      %  @param[in] u_end: Value of the phase variable at the end of the movement (t = tau)
      function set_can_sys_params(can_sys, u_end)
          
          can_sys.a_u = can_sys.u0 - u_end;
          
      end

      %% Returns the derivated output of the canonical system for the phase variable values.
      %  @param[in] x: Vector with the phase variable values.
      %  @param[out] dx: Vector with the phase variable derivative values.
      %  @param[out] u: Vector with the shape variable values.
      function [dx, u] = get_output_dot(can_sys, x)

           dx = can_sys.get_phaseVar_dot(x);
           u = can_sys.get_shapeVar(x);

      end
      
      %% Returns the output of the canonical system for the specified timestamps.
      %  @param[in] t: Vector of timestamps.
      %  @param[out] x: Vector with phase variable values for the timestamps in 't'.
      %  @param[out] u: Vector with shape variable values for the timestamps in 't'.
      function [x, u] = get_output(can_sys, t)

          x = can_sys.get_phaseVar(t);
          u = can_sys.get_shapeVar(x);

      end

      %% Returns the canonical system's phase variable for the specified timestamps.
      %  @param[in] t: Vector of timestamps.
      %  @param[out] x: The canonical system's phase variable.
      function x = get_phaseVar(can_sys, t)

          x = can_sys.can_clock.get_phase(t);

      end

      %% Returns the canonical system's phase variable derivative for the specified timestamps.
      %  @param[in] t: Vector of timestamps.
      %  @param[out] x: The canonical system's phase variable derivative.
      function dx = get_phaseVar_dot(can_sys, x)

          dx = can_sys.can_clock.get_phase_dot(x);

      end

      %% Returns the canonical system's shape variable for the specified values of the phase variable.
      %  @param[in] x: Vector of phase variable values.
      %  @param[out] u: Vector of shape variable values.
      function u = get_shapeVar(can_sys, x)

          u = can_sys.u0 - can_sys.a_u*x;

      end

     
   end
end




