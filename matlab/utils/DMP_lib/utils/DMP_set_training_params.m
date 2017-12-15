%% Sets the high level training parameters of the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] train_method: Method used to train the DMP weights.
%  @param[in] USE_GOAL_FILT: Flag indicating whether to use goal filtering.
%  @param[in] a_g: Goal filtering gain.
%  @param[in] lambda: Forgetting factor for recursive training methods.
%  @param[in] P_rlwr: Covariance matrix 'P' for recursive training methods.
function DMP_set_training_params(dmp, train_method, USE_GOAL_FILT, a_g, lambda, P_rlwr)

dmp.train_method = train_method;
dmp.USE_GOAL_FILT = USE_GOAL_FILT;
dmp.a_g = a_g;
dmp.lambda = lambda;
dmp.P_rlwr = P_rlwr;

end