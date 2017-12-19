%% Trains the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] Time: Row vector with the timestamps of the training data points.
%  @param[in] yd_data: Row vector with the desired potition.
%  @param[in] dyd_data: Row vector with the desired velocity.
%  @param[in] ddyd_data: Row vector with the desired accelaration.
%  @param[in] y0: Initial position.
%  @param[in] g0: Target-goal position.
%
%  \note The timestamps in \a Time and the corresponding position,
%  velocity and acceleration data in \a yd_data, \a dyd_data and \a
%  ddyd_data need not be sequantial in time.
function [train_error, F, Fd] = DMP_train(dmp, Time, yd_data, dyd_data, ddyd_data, y0, g0)

g = g0;
tau = dmp.can_sys_ptr.get_tau();

x = dmp.can_sys_ptr.get_phaseVar(Time);

g = g * ones(size(x));
if (dmp.USE_GOAL_FILT)
    g = y0*exp(-dmp.a_g*Time/tau) + g0*(1 - exp(-dmp.a_g*Time/tau));
end

s = zeros(size(x));
for i=1:length(s)
    s(i) = dmp.forcing_term_scaling(x(i), y0, g(i));
end
% s = dmp.forcing_term_scaling(x, y0, g0);
if (length(s) == 1), s = ones(size(x))*s(1); end

Fd = dmp.calc_Fd(yd_data, dyd_data, ddyd_data, x, y0, g0, g);

Psi = dmp.activation_function(x);

train_method = dmp.train_method;
if (strcmpi(train_method,'LWR'))
    
    dmp.w = LWR(Psi, s, Fd, dmp.zero_tol);
    
elseif (strcmpi(train_method,'RLWR'))
    
    dmp.w = RLWR(Psi, s, Fd, dmp.lambda, dmp.P_rlwr);
    
elseif (strcmpi(train_method,'LS'))
    
    dmp.w = leastSquares(Psi, s, Fd, dmp.zero_tol);
    
else
    error('Unsopported training method ''%s''', train_method);
end

F = zeros(size(Fd));
for i=1:size(F,2)
    F(i) = dmp.forcing_term(x(i))*dmp.forcing_term_scaling(x(i), y0, g(i));
end

%     n_sp = 4;
%     figure;
%     hold on;
%     subplot(n_sp,1,1);
%     plot(Time,F, Time, Fd);
%     legend('F','F_d');
%     hold off;
%     Fmax = max(abs([F(:); Fd(:)]));
%     subplot(n_sp,1,2);
%     hold on
%     plot(Time,F/Fmax, Time, Fd/Fmax);
%     for i=1:size(Psi,1)
%         plot(Time,Psi(i,:));
%     end
%     hold off
%     subplot(n_sp,1,3);
%     hold on;
%     for i=1:size(Psi,1)
%         temp = s.*Psi(i,:);
%         plot(Time,temp);
%     end
%     hold off;
%     subplot(n_sp,1,n_sp);
%     plot(Time, s);
%     legend('s')

train_error = norm(F-Fd)/length(F);

end
