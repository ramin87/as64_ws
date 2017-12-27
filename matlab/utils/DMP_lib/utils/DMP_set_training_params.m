%% Sets the high level training parameters of the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] train_method: Method used to train the DMP weights.
%  @param[in] lambda: Forgetting factor for recursive training methods.
%  @param[in] P_rlwr: Covariance matrix 'P' for recursive training methods.
function DMP_set_training_params(dmp, train_method, lambda, P_rlwr)

dmp.train_method = train_method;
dmp.lambda = lambda;
dmp.P_rlwr = P_rlwr;

end