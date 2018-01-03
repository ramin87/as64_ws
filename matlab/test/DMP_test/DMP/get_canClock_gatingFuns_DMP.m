function [canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, dmp] = get_canClock_gatingFuns_DMP(cmd_args, D, tau)

%% ========================================================
%% Init canonical clock
if (strcmpi(cmd_args.CAN_CLOCK_TYPE,'lin'))
    canClock_ptr = LinCanonicalClock();
else
    error('Unsupported canonical clock type ''%s''', cmd_args.CAN_CLOCK_TYPE);
end
canClock_ptr.init(tau);


%% ========================================================
%% Init shape attractor gating function
if (strcmpi(cmd_args.SHAPE_ATTR_GATTING_TYPE,'lin'))
    shapeAttrGating_ptr = LinGatingFunction();
elseif (strcmpi(cmd_args.SHAPE_ATTR_GATTING_TYPE,'exp'))
    shapeAttrGating_ptr = ExpGatingFunction();
elseif (strcmpi(cmd_args.SHAPE_ATTR_GATTING_TYPE,'sigmoid'))
    shapeAttrGating_ptr = SigmoidGatingFunction();
elseif (strcmpi(cmd_args.SHAPE_ATTR_GATTING_TYPE,'spring-damper'))
    shapeAttrGating_ptr = SpringDamperGatingFunction();
elseif (strcmpi(cmd_args.SHAPE_ATTR_GATTING_TYPE,'constant'))
    shapeAttrGating_ptr = ConstGatingFunction();
else
    error('Unsupported gating function type ''%s''', cmd_args.SHAPE_ATTR_GATTING_TYPE);
end
shapeAttrGating_ptr.init(cmd_args.SHAPE_ATTR_GATTING_u0, cmd_args.SHAPE_ATTR_GATTING_u_end);

% Optionally, one can set the steepness of the sigmoid, but in this case 'init' must be called again
if (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'sigmoid'))
    shapeAttrGating_ptr.a_u = 500;
end

%% ========================================================
%% Init goal attractor gating function
if (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'lin'))
    goalAttrGating_ptr = LinGatingFunction();
elseif (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'exp'))
    goalAttrGating_ptr = ExpGatingFunction();
elseif (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'sigmoid'))
    goalAttrGating_ptr = SigmoidGatingFunction();
elseif (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'spring-damper'))
    goalAttrGating_ptr = SpringDamperGatingFunction();
elseif (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'constant'))
    goalAttrGating_ptr = ConstGatingFunction();
else
    error('Unsupported gating function type ''%s''', cmd_args.GOAL_ATTR_GATTING_TYPE);
end
goalAttrGating_ptr.init(cmd_args.GOAL_ATTR_GATTING_u0, cmd_args.GOAL_ATTR_GATTING_u_end);

% Optionally, one can set the steepness of the sigmoid, but in this case 'init' must be called again
if (strcmpi(cmd_args.GOAL_ATTR_GATTING_TYPE,'sigmoid'))
    goalAttrGating_ptr.a_u = 500;
end


%% ========================================================
%% Extra args for the DMP
extraArgNames = {'k_trunc_kernel', 'Wmin', 'Freq_min', 'Freq_max', 'P1_min'};
extraArgValues = {cmd_args.k_trunc_kernel, cmd_args.Wmin, cmd_args.Freq_min, cmd_args.Freq_max, cmd_args.P1_min};

dmp = cell(D,1);
for i=1:D
    if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
       dmp{i} = DMP();
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
        dmp{i} = DMP_bio();
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
        dmp{i} = DMP_plus();
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-Shannon'))
        dmp{i} = DMP_Shannon();
    else
        error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
    end
    
    dmp{i}.init(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr,cmd_args.kernel_std_scaling, extraArgNames, extraArgValues);
end



end

