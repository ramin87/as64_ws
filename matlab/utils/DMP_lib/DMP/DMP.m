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

classdef DMP < handle
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
          
          DMP_init(dmp, N_kernels, a_z, b_z, can_sys_ptr, std_K);
          
      end
      

      %% Sets the centers for the activation functions of the DMP according to the partition method specified
      %  @param[in] part_type: Partitioning method for the kernel centers.
      function set_centers(dmp, part_type)
          
          if (nargin < 2)
              DMP_set_centers(dmp);
          else
              DMP_set_centers(dmp, part_type);
          end
        
      end
      
      
      %% Sets the kernels of the DMP using the EM algorithm
      function set_kernels_with_EM(dmp, Time, yd_data)
          
          DMP_set_kernels_with_EM(dmp, Time, yd_data)

      end
      
      
      %% Sets the standard deviations for the activation functions  of the DMP
      %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
      %  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
      function set_stds(dmp, s)

          DMP_set_stds(dmp, s)

      end
      

      %% Trains the DMP
      %  @param[in] Time: Row vector with the timestamps of the training data points.
      %  @param[in] yd_data: Row vector with the desired potition.
      %  @param[in] dyd_data: Row vector with the desired velocity.
      %  @param[in] ddyd_data: Row vector with the desired accelaration.
      %  @param[in] y0: Initial position.
      %  @param[in] g0: Target-goal position.
      %  @param[in] train_method: Method used to train the DMP weights.
      %  @param[in] USE_GOAL_FILT: flag indicating whether to use filtered goal (optional, default = false).
      %  @param[in] a_g: Parameter of the goal filter.
      %
      %  \note The timestamps in \a Time and the corresponding position,
      %  velocity and acceleration data in \a yd_data, \a dyd_data and \a
      %  ddyd_data need not be sequantial in time.
      function [train_error, F, Fd] = train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g0, train_method, USE_GOAL_FILT, a_g)
          
          [train_error, F, Fd] = DMP_train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g0, train_method, USE_GOAL_FILT, a_g);    
          
      end
      
      %% Calculates the desired values of the scaled forcing term.
      %  @param[in] y: Position.
      %  @param[in] dy: Velocity.
      %  @param[in] ddy: Acceleration.
      %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
      %  @param[in] y0: initial position.
      %  @param[in] g0: final goal.
      %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter).
      %  @param[out] Fd: Desired value of the scaled forcing term.
      function Fd = calc_Fd(dmp, y, dy, ddy, u, y0, g0, g)
          
          v_scale = dmp.get_v_scale();
          Fd = (ddy*v_scale^2 + -dmp.goal_attractor(y, v_scale*dy, g));
          
      end
      
            
      %% Returns the forcing term of the DMP.
      %  @param[in] x: The phase variable.
      %  @param[out] f: The normalized weighted sum of Gaussians.
      function f = forcing_term(dmp,x)
          
          f = DMP_forcing_term(dmp,x);

      end
      
      %% Returns the scaling factor of the forcing term.
      %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
      %  @param[in] y0: initial position.
      %  @param[in] g0: final goal.
      %  @param[out] f_scale: The scaling factor of the forcing term.
      function f_scale = forcing_term_scaling(dmp, u, y0, g0)
          
          f_scale = u*(g0-y0);
          
<<<<<<< HEAD
      end
      
      %% Returns the goal attractor of the DMP.
      %  @param[in] y: \a y state of the DMP.
      %  @param[in] z: \a z state of the DMP.
      %  @param[in] g: Goal position.
      %  @param[out] goal_attr: The goal attractor of the DMP.
      function goal_attr = goal_attractor(dmp, y, z, g)
=======
          if (strcmpi(train_method,'LWR'))
              
              LWR_train(dmp,x, s, Fd);

          elseif (strcmpi(train_method,'LS'))
              
              LS_train(dmp,x, s, Fd);

          else    DMP_get_v_scale
              error('Unsopported training method ''%s''', train_method);
          end
>>>>>>> 55523b2ca1692ae5396e5d396a251d9973da6348
          
          goal_attr = dmp.a_z*(dmp.b_z*(g-y)-z);
          
      end
      
      
      %% Returns the shape attractor of the DMP.
      %  @param[in] x: The phase variable.
      %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
      %  @param[in] y0: initial position.
      %  @param[in] g0: final goal.
      %  @param[out] shape_attr: The shape_attr of the DMP.
      function shape_attr = shape_attractor(dmp, x, u, y0, g0)
          
          f = DMP_forcing_term(dmp,x);
          f_scale = dmp.forcing_term_scaling(u, y0, g0);  
          shape_attr = f * f_scale;
          
      end
      
      
      %% Returns the derivatives of the DMP states
      %  @param[in] y: \a y state of the DMP.
      %  @param[in] z: \a z state of the DMP.
      %  @param[in] x: phase variable.
      %  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
      %  @param[in] y0: initial position.
      %  @param[in] g0: final goal.
      %  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter).
      %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
      %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
      %  @param[out] dy: derivative of the \a y state of the DMP.
      %  @param[out] dz: derivative of the \a z state of the DMP.
      function [dy, dz] = get_states_dot(dmp, y, z, x, u, y0, g0, g, y_c, z_c)
          
          if (nargin < 10), z_c=0; end
          if (nargin < 9), y_c=0; end
          %if (nargin < 8), g=g0; end
          
          v_scale = dmp.get_v_scale();
<<<<<<< HEAD
          shape_attr = dmp.shape_attractor(x, u, y0, g0);
          goal_attr = dmp.goal_attractor(y, z, g);
          
          dz = ( goal_attr + shape_attr + z_c) / v_scale;
=======

          force_term = dmp.forcing_term(x)*u*(g0-y0);
          
          dz = ( dmp.a_z*(dmp.b_z*(g-y)-z) + force_term + z_c) / v_scale;
        
>>>>>>> 55523b2ca1692ae5396e5d396a251d9973da6348
          dy = ( z + y_c) / v_scale;
        
      end
      
      
      %% Returns a column vector with the values of the activation functions of the DMP
      %  @param[in] x: phase variable
      %  @param[out] psi: column vector with the values of the activation functions of the DMP
      function psi = activation_function(dmp,x)
          
          psi = DMP_gaussian_kernel(dmp,x);
          
      end
      
     
      %% Returns the scaling factor of the DMP
      %  @param[out] v_scale: The scaling factor of the DMP.
      function v_scale = get_v_scale(dmp)
          
          v_scale = DMP_get_v_scale(dmp);
          
      end
      
      
      %% Returns the time cycle of the DMP
      %  @param[out] tau: The time cycle of the DMP.
      function tau = get_tau(dmp)
          
          tau = DMP_get_tau(dmp);
          
      end
      
      
   end
end




