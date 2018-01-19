function [sim_mse, sim_mse_dtw] = get_sim_mse_with_temp_and_spatial_align(Time1, y1, Time2, y2)


D = size(y1, 1);

y1_a = cell(D,1);
y2_a = cell(D,1);

if (Time1(end) > Time2(end))
    Time = Time1;
else
    Time = Time2;
end

sim_mse = zeros(D,1);
for i=1:D
   [y1_a{i}, y2_a{i}] = makeSignalsEqualLength(Time1, y1(i,:), Time2, y2(i,:), Time); 
   sim_mse(i) = norm(y1_a{i} - y2_a{i});
end

sim_mse_dtw = zeros(D,1);
[Time1, z1, Time2, z2] = alignSignalsWithDTW(Time1, y1, Time2, y2);
for i=1:D
   sim_mse_dtw(i) = norm(z1(i,:) - z2(i,:))/length(z1(i,:));
end

end

