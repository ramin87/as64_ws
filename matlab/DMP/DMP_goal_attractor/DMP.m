%% DMP class
%  Implements an 1-D DMP.
%  The DMP is driven by the canonical system:
%     dx = -ax*x/tau
%  where x is the phase variable and ax the decay term.
%
%  The DMP has the following form:
%
%     ddy = ( a_z*(b_z*(g-y)-dy*tau) + f*x*(g-y0) ) / tau^2; 
%
%  where
%     tau: is scaling factor defining the duration of the motion
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
%     ddy = ( goal_attractor(y,dy,g) + shape_attractor(x,g,y0) ) / tau^2;
%     
%   where:
%     goal_attractor(y,dy,g) = a_z*(b_z*(g-y)-dy*tau)
%     shape_attractor(x,g,y0) = f*x*(g-y0)
%
%   Optionally the goal 'g' fed to the DMP can be filtered to ensure smooth transitions 
%   in the accelaration produced by the DMP. The filtering is donce as follows:
%     tau*dg = a_g*(g0 - g)
%   where 'g0' is the DMP goal, 'g' is the continuous goal variable and 'a_g'
%   a time contant determining how fast 'g' converges to 'g0' (the higher 'a_g'
%   is, the faster the convergence).
%

classdef DMP < handle
   properties
       N_kernels % number of kernels (basis functions)
       
       a_z % parameter 'a_z' relating to the spring-damper system
       b_z % parameter 'b_z' relating to the spring-damper system
       
       g0 % goal of the DMP
       
       can_sys_ptr % handle (pointer) to the canonical system
    
       w % N_kernelsx1 vector with the weights of the DMP
       c % N_kernelsx1 vector with the kernel centers of the DMP
       h % N_kernelsx1 vector with the kernel stds of the DMP
    
       zero_tol % tolerance value used to avoid divisions with very small numbers
       
       USE_GOAL_FILT % flag determining whether to use goal filtering
       a_g % time contant determining how fast 'g' converges to 'g0' (the higher 'a_g'
           % is, the faster the convergence).
           
       a_s % scaling factor to ensure smaller changes in the accelaration
       
       v_scale
   end
   
   methods
      %% initializes a DMP
      %  N_kernels: contains the number of kernels
      %  a_z: param 'a_z'
      %  b_z: param 'b_z'
      %  can_sys_ptr: pointer to a DMP canonical system object
      %  std_K: scales the std of each kernel
      function dmp = DMP(N_kernels, a_z, b_z, can_sys_ptr, std_K, USE_GOAL_FILT, a_g)
          
          if (nargin < 5), std_K = 1; end
          if (nargin < 6), USE_GOAL_FILT = false; end
          
          dmp.N_kernels = N_kernels;
          dmp.a_z = a_z;
          dmp.b_z = b_z;
          dmp.can_sys_ptr = can_sys_ptr;
          
          dmp.a_s = 1 / (dmp.can_sys_ptr.tau * 2);
          
          dmp.zero_tol = realmin;
          
          if (USE_GOAL_FILT)
              dmp.USE_GOAL_FILT = true;
              dmp.a_g = a_g;
          end
          
          dmp.w = zeros(dmp.N_kernels,1); %rand(dmp.N_kernels,1);
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
          
          n_data
          
          istart = 1;
          iend = length(t)-0;
          
          t = t(istart:iend);
          yd_data = yd_data(istart:iend);
          dyd_data = dyd_data(istart:iend);
          ddyd_data = ddyd_data(istart:iend);
          
%           Fd = Fd(offset:end);
%           ddyd_data = ddyd_data(offset:end);
%           g_attr_data = g_attr_data(offset:end);
%           s = s(offset:end);
%           x = x(offset:end);
          
          dmp.can_sys_ptr.tau = t(end);
          g = yd_data(end);
          y0 = yd_data(1);
          x0 = 1;
          g0 = g;
          tau = dmp.can_sys_ptr.tau;
          
          
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
          if (dmp.USE_GOAL_FILT)
          	g = y0*exp(-dmp.a_g*t/tau) + g0*(1 - exp(-dmp.a_g*t/tau));
          end
          
          ddzd_data = ddyd_data*tau^2 * dmp.a_s^2;
          g_attr_data = - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data*tau*dmp.a_s);
          Fd = ddzd_data + g_attr_data;
          
          
%           ddzd_data2 = ddyd_data/tau^2;
%           g_attr_data2 = - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data/tau);
%           Fd2 = ddzd_data2 + g_attr_data2;
          
%           Ji_prev = zeros(dmp.N_kernels,1);
%           for k=1:dmp.N_kernels
%               Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
%               Ji_prev(k) = sum( Psi .* (Fd - dmp.w(k)*(x.*s')).^2 );
%           end

          %Fd = ddyd_data*tau^2 - dmp.a_z*(dmp.b_z*(g-yd_data)-dyd_data*tau);
          
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
              F(i) = dmp.shape_attractor(X(i),g0,y0) * (dmp.can_sys_ptr.tau^2 * dmp.a_s^2);
          end

          train_error = norm(F-Fd)/length(F);
          
          t = (0:(length(F)-1))*Ts;
%           figure;
%           plot(t,F,t,Fd);
%           legend('F','Fd');

%           Ji = zeros(dmp.N_kernels,1);
%           Psi_data = [];
%           
%           for k=1:dmp.N_kernels
%               Psi = exp(-dmp.h(k)*(x-dmp.c(k)).^2);
%               Ji(k) = sum( Psi .* (Fd - dmp.w(k)*(x.*s')).^2 );
%               
%               Psi_data = [Psi_data; Psi];
%           end
          
          %Ji'
          %dmp.w'
          %dmp.c'
          %find(Ji>Ji_prev)
          
          
%           if (dmp.USE_GOAL_FILT)
%           	figure;
%             plot(t,g);
%             title('goal evolution');
%           end
          
%           figure;
%           plot(t,ddzd_data,t,g_attr_data,t,Fd,t,F);
%           legend('ddz','g-attr','F_d','F');
          
%           figure;
%           subplot(2,1,1);
%           plot(t,Fd,t,F);
%           legend('F_d','F');
%           subplot(2,1,2);
%           plot(t,Fd-F);
%           legend('F_d-F');         
          
          
          
%           figure;
%           plot(t,ddzd_data,t,ddzd_data2);
%           legend('ddzd','ddzd_2');
          
%           figure;
%           plot(t,Fd,t,Fd2);
%           legend('F_d','F_{d2}');
          
%           figure;
%           subplot(3,1,1);
%           hold on;
%           plot(x,F/max(abs(F)),'LineWidth',1.2,'Color',[0 1 0]);
%           plot(x,Fd/max(abs(Fd)),'LineWidth',1.2,'Color',[1 0 0]);
%           for i=1:size(Psi_data,1)
%             plot(x,Psi_data(i,:));
%           end
%           bar(dmp.c,dmp.w/max(abs(dmp.w)));
%           title('Weighted summation');
%           set(gca,'Xdir','reverse');
%           xlim([0 1]);
%           axis tight;
%           hold off;
%           subplot(3,1,2);
%           bar(dmp.c,Ji);
%           set(gca,'Xdir','reverse');
%           xlim([0 1]);
%           subplot(3,1,3);
%           bar(dmp.c,Ji_prev);
%           set(gca,'Xdir','reverse');
%           xlim([0 1]);

          %error('Stop');
          
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
          s_attr = f*u*(g-y0)  / (dmp.can_sys_ptr.tau^2 * dmp.a_s^2);
          
      end
      
      %% Returns the goal-attractor of the DMP
      function g_attr = goal_attractor(dmp,y,dy,g)
          
          g_attr = dmp.a_z*(dmp.b_z*(g-y)-dy*dmp.can_sys_ptr.tau * dmp.a_s) / (dmp.can_sys_ptr.tau^2 * dmp.a_s^2) ;
          
      end
      
      %% Returns the output of the DMP
      function dmp_out = get_output(dmp,y,dy,g,y0,x)
          
          dmp_out = ( dmp.goal_attractor(y,dy,g) + dmp.shape_attractor(x,dmp.g0,y0) );
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




