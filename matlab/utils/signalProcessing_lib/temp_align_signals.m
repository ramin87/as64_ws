function [Time, z1, z2] = temp_align_signals(Time1, y1, Time2, y2)

% dist_f = @(s1,s2) norm(s1-s2);
% dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
% [dtw_dist, ind_y, ind_yd] = dtw(y_data, yd_data, dtw_win, dist_f);
% sim_mse = dtw_dist/length(ind_y);

dt = min([abs(diff(Time1)) abs(diff(Time2))]);

if (Time2(end) > Time1(end))
    t = Time1(end):dt:Time2(end);
    Time1 = [Time1 t];
    y1 = [y1 repmat(y1(end),1,length(t))];
    
    z2 = interp1(Time2, y2, Time1);
    z1 = y1;
    Time = Time1;
else
    t = Time2(end):dt:Time1(end);
    Time2 = [Time2 t];
    y2 = [y2 repmat(y2(end),1,length(t))];
    
    z1 = interp1(Time1, y1, Time2);
    z2 =  y2; 
    Time = Time2;
end

end

