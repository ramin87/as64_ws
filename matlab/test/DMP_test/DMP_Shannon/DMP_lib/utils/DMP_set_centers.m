%% Sets the centers for the activation functions of the DMP according to the partition method specified
%  @param[in] dmp: The DMP object.
%  @param[in] part_type: Partitioning method for the kernel centers.
function DMP_set_centers(dmp, part_type)

    t = ((1:dmp.N_kernels)-1)/(dmp.N_kernels-1);

    if (nargin < 2)
        % Partitions the centers according to the canonical system type
        % so that the centers are equally spaced in time.
        part_type = 'can_sys_like';
    end

    if (strcmpi(part_type,'lin'))
        dmp.c = t';
    elseif (strcmpi(part_type,'can_sys_like'))
        x = dmp.can_sys_ptr.get_continuous_output(t*dmp.can_sys_ptr.tau);
        dmp.c = x(1,:)';
    else
        error('Unsupported partition type %s',part_type);
    end

end