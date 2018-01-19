function [canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp] = get_canClock_gatingFuns_DMP(cmd_args, D, tau)

%% ========================================================
%% Init canonical clock
canClockPtr = getCanClock(cmd_args.CAN_CLOCK_TYPE, tau);


%% ========================================================
%% Init shape attractor gating function
shapeAttrGatingPtr = getGatingFun(cmd_args.SHAPE_ATTR_GATTING_TYPE, cmd_args.SHAPE_ATTR_GATTING_u0, cmd_args.SHAPE_ATTR_GATTING_u_end);


%% ========================================================
%% Init goal attractor gating function
goalAttrGatingPtr = getGatingFun(cmd_args.GOAL_ATTR_GATTING_TYPE, cmd_args.GOAL_ATTR_GATTING_u0, cmd_args.GOAL_ATTR_GATTING_u_end);


%% ========================================================
%% Extra args for the DMP
extraArgNames = {'k_trunc_kernel', 'Wmin', 'Freq_min', 'Freq_max', 'P1_min'};
extraArgValues = {cmd_args.k_trunc_kernel, cmd_args.Wmin, cmd_args.Freq_min, cmd_args.Freq_max, cmd_args.P1_min};

dmp = cell(D,1);
for i=1:D
    dmp{i} = getDMP(cmd_args.DMP_TYPE, cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, ...
        canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, cmd_args.kernelStdScaling, extraArgNames, extraArgValues);
end



end

