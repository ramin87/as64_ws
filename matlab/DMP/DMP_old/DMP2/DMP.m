%% DMP class
%  Implements an 1-D DMP.
%  The DMP is driven by the canonical system:
%     dx = -ax*x/ts
%  where x is the phase variable and ax the decay term.
%
%  The DMP has the following form:
%
%     ddy = ts^2 * ( a_z*(b_z*(g-y)-dy/ts) + f*x*(g-y0) ); 
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

classdef DMP < handle
   properties
       N_kernels % number of kernels (basis functions)
       
       a_z % parameter 'a_z' relating to the spring-damper system
       b_z % parameter 'b_z' relating to the spring-damper system
       
       a_x % the decay factor of the phase variable
       
       can_sys_ptr % handle (pointer) to the canonical system
       
       a_g
    
       w % N_kernelsx1 vector with the weights of the DMP
       c % N_kernelsx1 vector with the kernel centers of the DMP
       h % N_kernelsx1 vector with the kernel stds of the DMP
    
       zero_tol % tolerance value used to avoid divisions with very small numbers
       
       USE_GOAL_FILT
   end
   
   methods
      %% initializes a DMP
      %  N_kernels: contains the number of kernels
      %  a_z: param 'a_z'
      %  b_z: param 'b_z'
      %  ax: the decay factor of the phase variable
      % centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
      % std_K: scales the std of each kernel
      function dmp = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K)
          
          if (nargin < 5), std_K = 1; end
          
          dmp.N_kernels = N_kernels;
          dmp.a_z = a_z;
          dmp.b_z = b_z;
          dmp.can_sys_ptr = can_sys_ptr;
          
          dmp.zero_tol = realmin;
          
          %dmp.a_g = a_g;
          %dmp.USE_GOAL_FILT = USE_GOAL_FILT;
          
          dmp.w = rand(dmp.N_kernels,1);
          dmp.set_centers();
          dmp.set_stds(std_K);

      end
      
      %% Sets the centers for the activation functions  of the DMP
      function set_centers(dmp, part_type, a_x)

        t = ((1:dmp.N_kernels)-1)/(dmp.N_kernels-1);
        
        if (nargin < 2)
            part_type = 'can_sys_like';
        end
        
        if (strcmpi(part_type,'lin'))
            dmp.c = t';
        elseif (strcmpi(part_type,'exp'))
            if (nargin < 3)
                error('Not enough input arguments for center partion type ''exp''...\n');
            end
            dmp.c = exp(-a_x*t)'; 
        elseif (strcmpi(part_type,'can_sys_like'))
            x0 = 1;
            x = dmp.can_sys_ptr.get_continuous_output(t*dmp.can_sys_ptr.tau, x0);
            dmp.c = x(1,:)';
        else
            error('Unsupported partition type %s',part_type);
        end
        
      end
      
      %% Sets the kernels of the DMP using the EM algorithm
      function set_kernels_with_EM(dmp, Time, yd_data)
          
          error('Function ''set_kernels_with_EM'' is not supported yet...');
          
%           ts = Time(end) - Time(1);
%           
%           x = exp(-dmp.ax*Time/ts);
%           
%           Data = [x; yd_data];
%           
%           disp('EM_init_kmeans');
%           tic
%           [Priors, Mu, Sigma] = EM_init_kmeans(Data, dmp.N_kernels);
%           toc
%           
%           disp('EM');
%           tic
%           [Priors, Mu, Sigma] = EM(Data, Priors, Mu, Sigma);
%           toc
% 
%           dmp.c = Mu(1,:)';
%           for k=1:dmp.N_kernels
%               dmp.h(k) = 1/(2*Sigma(1,1,k));
%           end
      end
      
      %% Sets the standard deviations for the activation functions  of the DMP
      function set_stds(dmp, s)

        if (nargin < 2), s=1; end
        
        dmp.h = s./(dmp.c(2:end)-dmp.c(1:end-1)).^2;
        dmp.h = [dmp.h; dmp.h(end)];

      end

      %% Trains the DMP
      function [train_error, F, Fd] = train(dmp, yd_data, dyd_data, ddyd_data, Ts, train_method)
          
          n_data = size(yd_data,2);

          t = (0:n_data-1)*Ts;
          dmp.can_sys_ptr.tau = t(end);
          g = yd_data(end);
          y0 = yd_data(1);
          x0 = 1;
          g0 = g;
          tau = dmp.can_sys_ptr.tau;
          
          %x = x0*exp(-dmp.a_x*t/dmp.tau);
          X = dmp.can_sys_ptr.get_continuous_output(t, x0);
          if (size(X,1) == 1)
              x = X;
              u = x;
          else
              x = X(1,:);
              u = X(2,:);
          end
          
          s = u*(g0-y0);
          s = s(:);
          
          g = g * ones(size(x));
%           if (dmp.USE_GOAL_FILT)
%           	g = y0*exp(-dmp.a_g*t/tau) + g0*(1 - exp(-dmp.a_g*t/tau));
%           end
          
          Fd = ddyd_data/tau^2 - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data/tau);       
              
          if (strcmpi(train_method,'LWR'))
              
              for k=1:dmp.N_kernels
                  Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
                  temp = s'.*Psi;
                  dmp.w(k) = (temp*Fd(:)) / (temp*s + dmp.zero_tol);
                  
                  %Psi = diag( exp(-dmp.h(k)*(x-dmp.c(k)).^2) );
                  %dmp.w(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + dmp.zero_tol);
              end

          elseif (strcmpi(train_method,'LS'))

              H = zeros(dmp.N_kernels, n_data);
              
              for k=1:dmp.N_kernels
                  Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
                  H(k,:) = Psi; 
              end
              H = H.*repmat(s',size(H,1),1) ./ (repmat(sum(H,1),size(H,1),1) + dmp.zero_tol);

%               for i=1:n_data
%                   Psi = exp(-dmp.h.*(x(i)-dmp.c).^2);
%                   Psi = s(i)*Psi / (sum(Psi) + dmp.zero_tol);                 
%                   H(:,i) = Psi(:);
%               end
              
              dmp.w = (Fd/H)';

          else    
              error('Unsopported training method ''%s''', train_method);
          end
          
          F = zeros(size(Fd));
          for i=1:size(F,2)
              F(i) = dmp.shape_attractor(X(i),g(i),y0);
          end

          train_error = norm(F-Fd)/length(F);
          
      end
      
      %% Returns the shape-attractor of the DMP
      function s_attr = shape_attractor(dmp,X,g,y0)
          
          x = X(1);
          if (length(X) == 1)
            u = x;
          else
              u = X(2);
          end
          
          f = dmp.forcing_term(x);
          s_attr = f*u*(g-y0);
          
      end
      
      %% Returns the goal-attractor of the DMP
      function g_attr = goal_attractor(dmp,y,dy,g)
          
          g_attr = dmp.a_z*(dmp.b_z*(g-y)-dy/dmp.can_sys_ptr.tau);
          
      end
      
      %% Returns the output of the DMP
      function dmp_out = get_output(dmp,y,dy,g,y0,x)
          
          dmp_out = dmp.can_sys_ptr.tau^2 * ( dmp.goal_attractor(y,dy,g) + dmp.shape_attractor(x,g,y0) );
      end
      
      %% Returns the forcing term of the DMP
      function f = forcing_term(dmp,x)
          Psi = dmp.activation_function(x);
          f = dot(Psi,dmp.w) / (sum(Psi)+dmp.zero_tol); % add realmin to avoid numerical issues
      end
        
      %% Returns a vector with the activation functions for the DMP
      function psi = activation_function(dmp,x)
          psi = exp(-dmp.h.*((x-dmp.c).^2));
      end
     
   end
end




