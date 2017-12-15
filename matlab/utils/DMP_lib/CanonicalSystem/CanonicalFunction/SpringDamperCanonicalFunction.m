%% Spring-Damper Canonical Function class
%  Implements a spring-damper canonical function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the canonical function is:
%     u = u0*(x*exp(-a_u*x));
%    du = u0*exp(-a_u*x)*(1-a_u*x);
%

classdef SpringDamperCanonicalFunction < handle
   properties
       u0 % scaling of the canonical function's output
       a_u % the rate of evolution the canonical function
   end
   
   methods
      %% Spring-Damper Canonical Function Constructor.
      %  @param[in] u_end: Final value of the canonical function.
      %  @param[in] u0: Initial value oof the canonical function (optional, default = 1).
      %  @param[out] can_fun: Canonical function object.
      function can_fun = SpringDamperCanonicalFunction(u_end, u0)

          if (nargin < 1), u_end = 0.005; end
          if (nargin < 2), u0 = 1.0; end
          
          can_fun.init(u_end, u0);
          
      end 
      
      %% Initializes the canonical function.
      %  @param[in] u_end: Final value of the canonical function.
      %  @param[in] u0: Initial value of the canonical function (optional, default = 1.0).
      function init(can_fun, u_end, u0)
          
          if (nargin < 2), u_end = 0.005; end
          if (nargin < 3), u0 = 1.0; end

          if (u0*u_end <= 0)
              error('SpringDamperCanonicalFunction: init: u_end and u0 must have the same sign');
          end

          if (abs(u_end) > abs(u0))
              error('SpringDamperCanonicalFunction: init: |u_end| must be less than |u0|');
          end

          can_fun.u0 = u0;
          can_fun.set_can_fun_params(u_end);
          
      end
      
      %% Sets the canonical function's time constants based on the value of 
      %% the phase variable at the end of the movement.
      %  @param[in] u_end: Final value of the canonical function.
      function set_can_fun_params(can_fun, u_end)

          if (u_end == 0)
              error('SpringDamperCanonicalFunction: set_can_fun_params: u_end must be != 0');
          end

          can_fun.a_u = -log(u_end/can_fun.u0);
          
      end

      %% Returns the canonical function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the canonical function's output.
      function u = get_output(can_fun, x)

          exp_at = exp(-can_fun.a_u * x);
          s = 1 / (exp(-1)/can_fun.a_u);
          u = can_fun.u0*s*(x.*exp_at );

      end

      %% Returns the canonical function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the canonical function's derivated output.
      function du = get_output_dot(can_fun, x)

          exp_at = exp(-can_fun.a_u * x);
          du = can_fun.u0*exp_at .* (1 - can_fun.a_u*x );

      end

     
   end
end




