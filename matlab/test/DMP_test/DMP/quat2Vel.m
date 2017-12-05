function v_rot = quat2Vel(Q1, Q2)

v_rot = zeros(3, size(Q1,2));

for i=1:size(v_rot,2)
    v_rot(:,i) = quatLog(quatProd(Q1(:,i),quatInv(Q2(:,i))));
end

end