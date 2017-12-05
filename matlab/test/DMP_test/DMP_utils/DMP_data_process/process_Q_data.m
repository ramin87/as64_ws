function [Q_data, v_rot_data, dv_rot_data] = process_Q_data(Q_data,time_step, add_points_percent, smooth_points_percent)

n_data = size(Q_data,2);
add_points = ceil(n_data*add_points_percent);
Q_data = [repmat(Q_data(:,1),1,add_points) Q_data repmat(Q_data(:,end),1,add_points)];

n_data = size(Q_data,2);
smooth_points = ceil(n_data*smooth_points_percent);
smooth_times = 2;

% v_rot_data = [zeros(D,1) diff(Q_data')']/time_step;
v_rot_data = zeros(3,n_data);
for i=1:n_data-1
   v_rot_data(:,i+1) = quatLog(quatProd(Q_data(:,i+1),quatInv(Q_data(:,i)))) / time_step;
end

dv_rot_data = [zeros(3,1) diff(v_rot_data')']/time_step;

for i=1:3
    for k=1:smooth_times
        v_rot_data(i,:) = movingAverageFilter(v_rot_data(i,:),smooth_points);
        %dyd_data(i,:) = smooth(dyd_data(i,:),smooth_points,'moving');

        %ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
        %ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
        dv_rot_data(i,:) = movingAverageFilter(dv_rot_data(i,:),smooth_points);
    end
end

end

