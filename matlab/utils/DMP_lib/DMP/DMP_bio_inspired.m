%% DMP class
%  Implements an 1-D DMP.
%  The DMP is driven by a canonical system. An example of an exponential
%  canonical system is:
%     dx = -ax*x/tau
%  where x is the phase variable and ax the decay term. Other types of
%  canonical systems, such as a linear canonical system, can be used.
%
%  The DMP has the following form:
%
%     tau*dz = ( a_z*(b_z*(g-y) - z ) + f*s + z_c
%     tau*dy = z + y_c; 
%
%  Or equivalently:
%     ddy = ( a_z*(b_z*(g-y)-dy*tau) + f*x*(g-y0) ) / tau^2; 
%
%  where
%     tau: is scaling factor defining the duration (time cycle) of the motion
%     a_z, b_z: constants relating to a spring-damper system
%     g0: the final goal
%     g: the continuous goal (or the final goal in case no goal-filtering is used)
%     y0: the initial position
%     x: the phase variable
%     y,dy,ddy: the position, velocity and accelaration of the motion
%     f: the forcing term defined by the weighted sum of the activation
%        functions (gaussian kernels), i.e.: 
%        f = w'*Psi/ sum(Psi);
%     s: the scaling of the forcing term. It could be for instance 
%        s = u*(g0-y0), where u=x or some other variable that reaches zero
%        as x approaches its final value (which is also close to zero).
%        Another case is s = u*K, where K is the spring-damper stiffness.
%     
%   Optionally the goal 'g' fed to the DMP can be filtered to ensure smooth transitions 
%   in the accelaration produced by the DMP. The filtering is donce as follows:
%     tau*dg = a_g*(g0 - g)
%   where 'g0' is the DMP goal, 'g' is the continuous goal variable and 'a_g'
%   a time contant determining how fast 'g' converges to 'g0' (the higher 'a_g'
%   is, the faster the convergence).
%

classdef DMP_bio_inspired < handle
   properties
       N_kernels % number of kernels (basis functions)
       
       a_z % parameter 'a_z' relating to the spring-damper system
       b_z % parameter 'b_z' relating to the spring-damper system
       
       can_sys_ptr % handle (pointer) to the canonical system
    
       w % N_kernelsx1 vector with the weights of the DMP
       c % N_kernelsx1 vector with the kernel centers of the DMP
       h % N_kernelsx1 vector with the kernel stds of the DMP
    
       zero_tol % tolerance value used to avoid divisions with very small numbers
       
       a_s % scaling factor to ensure smaller changes in the accelaration to improve the training
   end
   
   methods
      %% DMP constructor
      %  @param[in] N_kernels: the number of kernels
      %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
      %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
      %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
      %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
      function dmp = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K)
          
          if (nargin < 4)
              return;
          else
              if (nargin < 5), std_K=1; end
              dmp.init(N_kernels, a_z, b_z, can_sys_ptr, std_K)
          end
          

      end
      
      %% Initializes the DMP
      %  @param[in] N_kernels: the number of kernels
      %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
      %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
      %  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
      %  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
      function init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K)
          
          dmp.zero_tol = realmin;
          
          if (nargin < 5), std_K = 1; end
          
          dmp.N_kernels = N_kernels;
          dmp.a_z = a_z;
          dmp.b_z = b_z;
          dmp.can_sys_ptr = can_sys_ptr;
          
          tau = dmp.get_tau();
          if (tau > 1)
              dmp.a_s = 1 / (dmp.can_sys_ptr.tau^2);
          else
              dmp.a_s = (dmp.can_sys_ptr.tau^2);
          end
          
          dmp.w = zeros(dmp.N_kernels,1); %rand(dmp.N_kernels,1);
          dmp.set_centers();
          dmp.set_stds(std_K);
          
      end

      %% Sets the centers for the activation functions of the DMP according to the partition method specified
      %  @param[in] part_type: Partitioning method for the kernel centers.
      function set_centers(dmp, part_type)

        t = ((1:dmp.N_kernels)-1)/(dmp.N_kernels-1);
        
        if (nargin < 2)
            % Partitions the centers according to the canonical system type
            % so that the centers are equally spaced in time.
            part_type = 'can_sys_like';
        end
        
        if (strcmpi(part_type,'lin'))
            dmp.c = t';
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
      %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
      %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
      function set_stds(dmp, s)

        if (nargin < 2), s=1; end
        
        dmp.h = 1./(s*(dmp.c(2:end)-dmp.c(1:end-1))).^2;
        dmp.h = [dmp.h; dmp.h(end)];

      end
      
      %% Trains the DMP weights using LWR (Locally Weighted Regression)
      %  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
      %  where Psi = exp(-h(k)*(x-c(k)).^2)
      %  @param[in] x: Row vector with the values of the phase variable.
      %  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
      %  @param[in] Fd: Row vector with the desired values of the shape attractor.
      function LWR_train(dmp, x, s, Fd)
          
          s = s(:);
          
          for k=1:dmp.N_kernels
              Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
              temp = s'.*Psi;
              dmp.w(k) = (temp*Fd(:)) / (temp*s + dmp.zero_tol);

              %Psi = diag( exp(-dmp.h(k)*(x-dmp.c(k)).^2) );
              %dmp.w(k) = (s'*Psi*Fd(:)) / (s'*Psi*s + dmp.zero_tol);
          end
          
      end
      
      %% Trains the DMP weights using LS (Least Squares)
      %  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
      %  where Psi = exp(-h(k)*(x-c(k)).^2)
      %  @param[in] x: Row vector with the values of the phase variable.
      %  @param[in] s: Row vector with the values of the term that is multiplied by the weighted sum of Gaussians.
      %  @param[in] Fd: Row vector with the desired values of the shape attractor.
      function LS_train(dmp, x, s, Fd)
          
          s = s(:);
          
          n_data = length(x);
          
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
          
      end

      %% Trains the DMP
      %  @param[in] yd_data: Row vector with the desired potition.
      %  @param[in] dyd_data: Row vector with the desired velocity.
      %  @param[in] ddyd_data: Row vector with the desired accelaration.
      %  @param[in] Ts: Sampling time of the training data.
      %  @param[in] train_method: Method used to train the DMP weights.
      %  @param[in] USE_GOAL_FILT: flag indicating whether to use filtered goal (optional, default = false).
      %  @param[in] a_g: Parameter of the goal filter.
      function [train_error, F, Fd] = train(dmp, yd_data, dyd_data, ddyd_data, Ts, train_method, USE_GOAL_FILT, a_g)
          
          n_data = size(yd_data,2);

          Time = (0:n_data-1)*Ts;  
          tau = Time(end);
          
          istart = 1;
          iend = length(Time)-0;
          
          Time = Time(istart:iend);
          yd_data = yd_data(istart:iend);
          dyd_data = dyd_data(istart:iend);
          ddyd_data = ddyd_data(istart:iend);
          
          dmp.can_sys_ptr.tau = tau;
          g = yd_data(end);
          y0 = yd_data(1);
          x0 = 1;
          g0 = g;
          tau = dmp.can_sys_ptr.tau;
          
          
          X = dmp.can_sys_ptr.get_continuous_output(Time, x0);
          
          if (size(X,1) == 1)
              x = X;
              u = x;
          else
              x = X(1,:);
              u = X(2,:);
          end

          s = u*(g0-y0);
          
          g = g * ones(size(x));
          if (USE_GOAL_FILT)
          	g = y0*exp(-a_g*Time/tau) + g0*(1 - exp(-a_g*Time/tau));
          end
          
          v_scale = dmp.get_v_scale();
          
          ddzd_data = ddyd_data*v_scale^2;
          g_attr_data = - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data*v_scale);
          Fd = (ddzd_data + g_attr_data);
          
          if (strcmpi(train_method,'LWR'))
              
              dmp.LWR_train(x, s, Fd);

          elseif (strcmpi(train_method,'LS'))
              
              dmp.LS_train(x, s, Fd);

          else    
              error('Unsopported training method ''%s''', train_method);
          end
          
          F = zeros(size(Fd));
          for i=1:size(F,2)
              F(i) = dmp.forcing_term(x(i))*u(i)*(g0-y0);
          end

          train_error = norm(F-Fd)/length(F);
          
      end
      
%       %% Returns the shape-attractor of the DMP
%       function s_attr = shape_attractor(dmp,x,u,g0,y0)
%           
%           f = dmp.forcing_term(x);
%           s_attr = f*u*(g0-y0);
%           
%       end
      
%       %% Returns the goal-attractor of the DMP
%       function g_attr = goal_attractor(dmp,y,dy,g)
%           
%           v_scale = dmp.get_v_scale();
%           g_attr = dmp.a_z*(dmp.b_z*(g-y)-dy*v_scale);
%           
%       end
      
%       %% Returns the output of the DMP
%       function dmp_out = get_output(dmp,y,dy,g,y0,x)
%           
%           v_scale = dmp.get_v_scale();
%           dmp_out = ( dmp.goal_attractor(y,dy,g) + dmp.shape_attractor(x,dmp.g0,y0) ) / v_scale^2;
%       end
      
      %% Returns the forcing term of the DMP
      %  @param[in] x: The phase variable.
      %  @param[out] f: The normalized weighted sum of Gaussians.
      function f = forcing_term(dmp,x)
          
          Psi = dmp.activation_function(x);
          f = dot(Psi,dmp.w) / (sum(Psi)+dmp.zero_tol); % add 'zero_tol' to avoid numerical issues
          
      end
      
      %% Returns the derivatives of the DMP states
      %  @param[in] y: 'y' state of the DMP
      %  @param[in] z: 'z' state of the DMP
      %  @param[in] x: phase variable
      %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion
      %  @param[in] y0: initial position
      %  @param[in] g0: final goal
      %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter)
      %  @param[in] y_c: coupling term for the dynamical equation of the 'y' state
      %  @param[in] z_c: coupling term for the dynamical equation of the 'z' state
      %  @param[out] dy: derivative of the 'y' state of the DMP
      %  @param[out] dz: derivative of the 'z' state of the DMP
      function [dy, dz] = get_states_dot(dmp, y, z, x, u, y0, g0, g, y_c, z_c)
          
          if (nargin < 10), z_c=0; end
          if (nargin < 9), y_c=0; end
          %if (nargin < 8), g=g0; end
          
          v_scale = dmp.get_v_scale();
          
          force_term = dmp.forcing_term(x)*u*(g0-y0);
          
          dz = ( dmp.a_z*(dmp.b_z*(g-y)-z) + force_term + z_c) / v_scale;
        
          dy = ( z + y_c) / v_scale;
        
      end
      
      %% Returns a column vector with the values of the activation functions of the DMP
      %  @param[in] x: phase variable
      function psi = activation_function(dmp,x)
          
          psi = exp(-dmp.h.*((x-dmp.c).^2));
          
      end
     
      %% Returns the scaling factor of the DMP
      %  @param[out] v_scale: The scaling factor of the DMP.
      function v_scale = get_v_scale(dmp)
          v_scale = dmp.get_tau() * dmp.a_s;
      end
      
      %% Returns the time cycle of the DMP
      %  @param[out] tau: The time cycle of the DMP.
      function tau = get_tau(dmp)
          
          tau = dmp.can_sys_ptr.get_tau();
          
      end
      
   end
end




