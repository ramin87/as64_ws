%% DMP Canonical System class
%  Implements a canonical system for a DMP.
%  The outputs of the canonical system are:
%     x: the phase variable (substitute for time 't')
%     u: the shape variable (multiplies the DMP's forcing term to ensure 
%                           that the forcing term ultimately becomes zero)
%  The canonical system is defined as:
%     x_dot = df_clock(x)
%     u = f_shape(x);
%  where 
%  * f_clock(), df_clock(): the function and derivative of the DMP's
%  clock. A monotonically increasing function from 0 to 1 is a
%  reasonable choice for f_clock(). The  clock serves as a substiture for 
%  time starting at x(t=0) = 0 and ending at x(t=tau) = 1, where 'tau' is 
%  the movements total duration.
%  * f_shape: the canonical function producing the shape variable. 
%  u is the canonical system's output used to ensure that the forcing term
%  of the DMP vanishes at t=tau. Therefore any function that starts from
%  some initial value at x=0 and reaches zero at x=1 is a good choice.
%

classdef CanonicalSystem < handle
   properties
       can_clock % canonical clock, producing the phase variable
       can_fun % canonical function, producing the shape variable
   end
   
   methods
      %% Linear Canonical System Constructor.
      %  @param[in] can_clock_type: Type of clock for the phase variable.
      %  @param[in] can_fun_type: Type of function for the shape variable.
      %  @param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  @param[in] u_end: Value of the shape variable at the end of the motion.
      %  @param[in] u0: Initial value of the shape variable (optional, default = 1).
      %  @param[out] can_sys: Canonical system object.
      function can_sys = CanonicalSystem(can_clock_type, can_fun_type, tau, u_end, u0)

          if (nargin < 4)
              can_clock_type = 'lin';
              can_fun_type = 'lin';
              tau = 1.0;
              u_end = 0.005;
          end

          if (nargin < 5), u0=1; end
          
          can_sys.init(can_clock_type, can_fun_type, tau, u_end, u0);
          
      end 
      
      %% Initializes the canonical system.
      %  @param[in] can_clock_type: Type of clock for the phase variable.
      %  @param[in] can_fun_type: Type of function for the shape variable.
      %  @param[in] tau: Movement duration (can be used to scale temporally the motion).
      %  @param[in] u_end: Value of phase variable at the end of the motion (optional, default = 0.005).
      %  @param[in] u0: Initial value of the phase variable (optional, default = 1.0).
      function init(can_sys, can_clock_type, can_fun_type, tau, u_end, u0)

          if (nargin < 5), u_end = 0.005; end
          if (nargin < 6), u0 = 1.0; end

          % Set up the canonical system clock
          if (strcmpi(can_clock_type,'lin'))
            can_sys.can_clock = LinCanonicalClock();          
          elseif (strcmpi(can_clock_type,'exp'))
            % can_sys.can_clock = ExpCanonicalClock();
          else
          	error('CanonicalSystem: init: Unsupported canonical clock type: ''%s''', can_clock_type);
          end
          can_sys.can_clock.init(tau);

          % Set up the canonical system function
          if (strcmpi(can_fun_type,'lin'))
            can_sys.can_fun = LinCanonicalFunction();
          elseif (strcmpi(can_fun_type,'exp'))
            can_sys.can_fun = ExpCanonicalFunction();
          elseif (strcmpi(can_fun_type,'spring-damper'))
            can_sys.can_fun = SpringDamperCanonicalFunction();
          elseif (strcmpi(can_fun_type,'sigmoid'))
            can_sys.can_fun = SigmoidCanonicalFunction();
          else
          	error('CanonicalSystem: init: Unsupported canonical function type: ''%s''', can_fun_type);
          end
          can_sys.can_fun.init(u_end, u0);
          
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

          u = can_sys.can_fun.get_output(x);
          u(u<0) = 0;

      end

     
   end
end




