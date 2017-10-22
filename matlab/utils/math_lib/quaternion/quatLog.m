function v_rot = quatLog(Q)
%   v_rot = quatLog(Q) calculates the logarithmic map of quaternion Q,
%   which is the rotational velocity, v_rot, that in unit time causes
%   rotation equal to Q.

    n = Q(1);
    e = Q(2:4);
    
    if (norm(e) > 1e-15)
        % use real(acos(η)) because for n close but greater than 1, acos(n) becomes complex number
        v_rot = 2*real(acos(n))*e/norm(e);
    else
        v_rot = zeros(size(e));
    end

end



