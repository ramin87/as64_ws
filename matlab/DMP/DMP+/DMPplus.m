%% DMPplus class
%  Implements an 1-D DMP plus.
%  The DMP is driven by the canonical system:
%     dx = -ax*x/ts
%  where x is the phase variable and ax the decay term.
%
%  The DMP has the following form:
%
%     ddy = ts^2 * ( a_z*(b_z*(g-y)-dy/ts) + f ); 
%
%  where
%     ts: is scaling factor defining the duration of the motion
%     a_z, b_z: constants relating to a spring-damper system
%     g: the goal position
%     y0: the initial position
%     x: the phase variable
%     y,dy,ddy: the position, velocity and accelaration of the motion
%     f: the forcing term defined by the weighted sum of the activation
%        functions (gaussian kernels), i.e.: 
%        f = w'*Psi/ sum(Psi);
%     
%  The output of the DMP can be writtern compactly in the following form:
%
%     ddy = ts^2 * ( goal_attractor(y,dy,g) + shape_attractor(x,g,y0) );
%     
%   where:
%     goal_attractor(y,dy,g) = a_z*(b_z*(g-y)-dy/ts)
%     shape_attractor(x,g,y0) = f*x*(g-y0)
%

classdef DMPplus < handle
   properties
       N_kernels % number of kernels (basis functions)
       
       a_z % parameter 'a_z' relating to the spring-damper system
       b_z % parameter 'b_z' relating to the spring-damper system
       
       ax % the decay factor of the phase variable
       ts % movement duration (can be used to scale temporally the motion)
    
       w % N_kernels x 1 vector with the weights of the DMP
       b % N_kernels x 1 vector with the bias term of the DMP
       theta % N_kernels x 1 vector with the limits for each kernel
       c % N_kernels x 1 vector with the kernel centers of the DMP
       h % N_kernels x 1 vector with the kernel stds of the DMP
       
       zero_tol % tolerance value used to avoid divisions with very small numbers
    
   end
   
   methods
      %% initializes a DMP
      %  N_kernels: contains the number of kernels
      %  a_z: param 'a_z'
      %  b_z: param 'b_z'
      %  ax: the decay factor of the phase variable
      % centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
      % std_K: scales the std of each kernel
      % theta_K: number of stds each gaussian spans around its center
      function dmp = DMP(N_kernels, a_z, b_z, ts, ax, centers_part_type, std_K, theta_K)
          
          if (nargin < 5), centers_part_type = 'exp'; end
          if (nargin < 6), std_K = 1; end
          if (nargin < 7), theta_K = 3; end % each kernels spans 3 stds
          
          dmp.N_kernels = N_kernels;
          dmp.a_z = a_z;
          dmp.b_z = b_z;
          dmp.ts = ts;
          dmp.ax = ax;
          dmp.w = rand(dmp.N_kernels,1);
          dmp.b = rand(dmp.N_kernels,1);
          dmp.c = dmp.set_centers(centers_part_type, dmp.N_kernels, dmp.ax);
          dmp.h = dmp.set_stds(dmp.c, std_K);
          dmp.theta = theta_K / sqrt(dmp.h);
          
          dmp.zero_tol = realmin;
          
      end
      
      %% Sets the centers for the activation functions  of the DMP
      function c = set_centers(dmp, part_type, n_centers, ax)

        if (strcmpi(part_type,'lin'))
            c = ((1:n_centers)'-1)/(n_centers-1);
        elseif (strcmpi(part_type,'exp'))
            if (nargin < 3), error('Not enough input arguments for exponential partition'); end
            c = exp(-ax*((1:n_centers)'-1)/(n_centers-1)); 
        else
            error('Unsupported partition type %s',part_type);
        end
        
      end
      
      %% Sets the standard deviations for the activation functions  of the DMP
      function h = set_stds(dmp, c, s)

        if (nargin < 3), s=1; end
        
        h = s./(c(2:end)-c(1:end-1)).^2;
        h = [h; h(end)];

      end

      %% Trains the DMP
      function train_error = train(dmp, yd_data, dyd_data, ddyd_data, Ts, train_method)
          
          train_error = 0;
          n_data = size(yd_data,2);
          
          t = (0:n_data-1)*Ts;
          dmp.ts = t(end);
          g = yd_data(end);
          y0 = yd_data(1);
          x0 = 1;
          
          Fd = ddyd_data/dmp.ts^2 - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data/dmp.ts);
          x = x0*exp(-dmp.ax*t/dmp.ts);
          s = x*(g-y0);
          s = s(:);
              
          if (strcmpi(train_method,'LWR'))
              
              s = s(:);
              for k=1:dmp.N_kernels
                  Psi = diag( exp(-dmp.h(k)*(x-dmp.c(k)).^2) );
                  dmp.w(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + dmp.zero_tol);
              end

              F = zeros(size(Fd));
              for i=1:size(F,2)
                  F(i) = dmp.shape_attractor(x(i),g,y0);
              end
              
              train_error = norm(F-Fd)/length(F);
              
%           elseif (strcmpi(train_method,'LS'))
% 
%               H = zeros(dmp.N_kernels, n_data);
%               for i=1:n_data
%                   Psi = exp(-dmp.h.*(x(i)-dmp.c).^2);
%                   Psi = s(i)*Psi / (sum(Psi) + dmp.zero_tol);
%                   H(:,i) = Psi(:)';
%               end
% 
%               dmp.w = (Fd/H)';
%               
%               F = zeros(size(Fd));
%               for i=1:size(F,2)
%                   F(i) = dmp.shape_attractor(x(i),g,y0);
%               end
%               
%               train_error = norm(F-Fd)/length(F);
%           else    
              error('Unsopported training method %s', train_method);
          end
          
      end
      
      %% Returns the shape-attractor of the DMP
      function s_attr = shape_attractor(dmp,x,g,y0)
          
          f = dmp.forcing_term(x);
          s_attr = f*x*(g-y0);
          
      end
      
      %% Returns the goal-attractor of the DMP
      function g_attr = goal_attractor(dmp,y,dy,g)
          
          g_attr = dmp.a_z*(dmp.b_z*(g-y)-dy/dmp.ts);
          
      end
      
      %% Returns the output of the DMP
      function dmp_out = get_output(dmp,y,dy,g,y0,x)
          dmp_out = dmp.ts^2 * ( dmp.goal_attractor(y,dy,g) + dmp.shape_attractor(x,g,y0) );
      end
      
      %% Returns the forcing term of the DMP
      function f = forcing_term(dmp,x)
          Psi = dmp.activation_function(x);
          f = dot(Psi,dmp.w*x + dmp.b) / (sum(Psi)+dmp.zero_tol); % add realmin to avoid numerical issues
      end
        
      %% Returns a vector with the activation functions for the DMP
      function psi = activation_function(dmp,x)
          psi = exp(-0.5*dmp.h.*((x-dmp.c).^2));
          psi( abs(x-dmp.c) > dmp.theta_) = 0;
      end
     
   end
end




