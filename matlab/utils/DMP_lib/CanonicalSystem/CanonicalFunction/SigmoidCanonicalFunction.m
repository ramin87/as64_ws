%% Sigmoid Canonical Function class
%  Implements a sigmoidal canonical function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the canonical function is:
%     u = u0 * ( 1 / (1 + exp(-a_u*(x-c)) ) );
%    du = -a_u*u0 * ( exp(-a_u*(x-c)) / (1 + exp(-a_u*(x-c)) )^2 );
%

classdef SigmoidCanonicalFunction < handle
   properties
       u0 % initial value of the canonical function
       a_u % the rate of evolution of the canonical function
       c % center of the exponential in the sigmoid
   end
   
   methods
      %% Sigmoid Canonical Function Constructor.
      %  @param[in] u_end: Final value of the canonical function.
      %  @param[in] u0: Initial value of the canonical function (optional, default = 1).
      %  @param[out] can_fun: Canonical function object.
      function can_fun = SigmoidCanonicalFunction(u_end, u0)

          if (nargin < 1), u_end = 0.99; end
          if (nargin < 2), u0 = 1.0; end
          
          can_fun.a_u = 150.0;
          
          can_fun.init(u_end, u0);
          
      end 
      
      %% Initializes the canonical function.
      %  @param[in] u_end: Final value of the canonical function.
      %  @param[in] u0: Initial value of the canonical function (optional, default = 1.0).
      function init(can_fun, u_end, u0)

          if (nargin < 2), u_end = 0.99; end
          if (nargin < 3), u0 = 1.0; end

          can_fun.u0 = u0;
          can_fun.set_can_fun_params(u_end);
          
      end
      
      %% Sets the canonical function's time constants based on the value of 
      %% the phase variable at the end of the movement.
      %  @param[in] u_end: Final value of the canonical function.
      function set_can_fun_params(can_fun, u_end)

          can_fun.c = 1.0 - (1.0/can_fun.a_u)*log((can_fun.u0-u_end)/u_end);
          
      end

      %% Returns the canonical function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the canonical function's output.
      function u = get_output(can_fun, x)

          exp_t = exp((can_fun.a_u)*(x-can_fun.c));
          u = can_fun.u0 * 1.0 ./ (1.0 + exp_t);

      end

      %% Returns the canonical function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the canonical function's derivated output.
      function du = get_output_dot(can_fun, x)

          exp_t = exp((can_fun.a_u/can_fun.tau)*(x-can_fun.c));
          du = -can_fun.u0 * (can_fun.a_u) * exp_t ./ (1.0 + exp_t).^2;

      end

     
   end
end




