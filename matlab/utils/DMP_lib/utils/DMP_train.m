%% Trains the DMP
%  @param[in] dmp: The DMP object.
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
function [train_error, F, Fd] = DMP_train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g0, train_method, USE_GOAL_FILT, a_g)

    g = g0;
    x0 = 1;
    tau = dmp.can_sys_ptr.tau;

    X = dmp.can_sys_ptr.get_continuous_output(Time, x0);

    if (size(X,1) == 1)
      x = X;
      u = x;
    else
      x = X(1,:);
      u = X(2,:);
    end

    g = g * ones(size(x));
    if (USE_GOAL_FILT)
    g = y0*exp(-a_g*Time/tau) + g0*(1 - exp(-a_g*Time/tau));
    end

    s = dmp.forcing_term_scaling(u, y0, g0);
    if (length(s) == 1), s = ones(size(x))*s(1); end

    Fd = dmp.calc_Fd(yd_data, dyd_data, ddyd_data, u, y0, g0, g);

    if (strcmpi(train_method,'LWR'))
  
      LWR_train(dmp,x, s, Fd);
      
    elseif (strcmpi(train_method,'RFWR'))
        
      RFWR_train(dmp,x, s, Fd);

    elseif (strcmpi(train_method,'LS'))

      LS_train(dmp,x, s, Fd);

    else    
      error('Unsopported training method ''%s''', train_method);
    end

    F = zeros(size(Fd));
    for i=1:size(F,2)
      F(i) = dmp.forcing_term(x(i))*dmp.forcing_term_scaling(u(i), y0, g0);
    end

    train_error = norm(F-Fd)/length(F);

end
