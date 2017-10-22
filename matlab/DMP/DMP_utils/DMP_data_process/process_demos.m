function [yd_data, dyd_data, ddyd_data] = process_demos(data,time_step, add_points_percent, smooth_points_percent)

yd_data = data;
n_data = size(yd_data,2);
add_points = ceil(n_data*add_points_percent);
yd_data = [repmat(yd_data(:,1),1,add_points) yd_data repmat(yd_data(:,end),1,add_points)];

D = size(yd_data,1);
n_data = size(yd_data,2);
smooth_points = ceil(n_data*smooth_points_percent);
smooth_times = 2;

dyd_data = [zeros(D,1) diff(yd_data')']/time_step;
ddyd_data = [zeros(D,1) diff(dyd_data')']/time_step;
for i=1:D
    for k=1:smooth_times
        dyd_data(i,:) = movingAverageFilter(dyd_data(i,:),smooth_points);
        %dyd_data(i,:) = smooth(dyd_data(i,:),smooth_points,'moving');

        %ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
        %ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
        ddyd_data(i,:) = movingAverageFilter(ddyd_data(i,:),smooth_points);
    end
end

end

