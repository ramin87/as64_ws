function [yd_data, dyd_data, ddyd_data] = process_demos(data,time_step)

yd_data = data;
n_data = size(yd_data,2);
add_points = ceil(n_data*0.04);
yd_data = [repmat(yd_data(:,1),1,add_points) yd_data repmat(yd_data(:,end),1,add_points)];

D = size(yd_data,1);
n_data = size(yd_data,2);
smooth_points = ceil(n_data*0.08);

dyd_data = [zeros(D,1) diff(yd_data')']/time_step;
ddyd_data = [zeros(D,1) diff(dyd_data')']/time_step;
for i=1:D
    dyd_data(i,:) = smooth(dyd_data(i,:),smooth_points,'moving');
    dyd_data(i,:) = smooth(dyd_data(i,:),smooth_points,'moving');
    
    ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
    ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
    ddyd_data(i,:) = smooth(ddyd_data(i,:),smooth_points,'moving');
end

end

