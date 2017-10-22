function d = d2_metric(V1, V2)

v1 = V1(1:3);
v1_rot = V1(4:6);
v2 = V2(1:3);
v2_rot = V2(4:6);

d = 0.5 * ( cosDist(v1,v2) + cosDist(v1_rot,v2_rot) );

end

