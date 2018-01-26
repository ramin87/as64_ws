CartPos_filename = 'data/CartPos_data';

load('dora_Pos_data.mat','Pos_data');
Yd_pos_data = Pos_data;
Yd_vel_data = zeros(size(Yd_pos_data));
Yd_accel_data = zeros(size(Yd_pos_data));
Ts = 0.001;
for i=1:size(Yd_pos_data,1)
    Yd_vel_data(i,2:end) = diff(Yd_pos_data(i,:))/Ts;
    Yd_accel_data(i,2:end) = diff(Yd_vel_data(i,:))/Ts;
end

Time = (0:(size(Yd_pos_data,2)-1))*Ts;
CartPos_data = {Yd_pos_data; Yd_vel_data; Yd_accel_data};

save_binary(CartPos_data,Time,CartPos_filename);

figure
for i=1:3
    subplot(3,1,i);
    plot(Time, Yd_pos_data(i,:));
end

figure
for i=1:3
    subplot(3,1,i);
    plot(Time, Yd_vel_data(i,:));
end

figure
for i=1:3
    subplot(3,1,i);
    plot(Time, Yd_accel_data(i,:));
end



function save_binary(data,Time,data_filename)

    fid = fopen([data_filename '.bin'],'w');
    
    write_mat(Time, fid, true);
    
    D = int32(length(data));
    write_scalar(D, fid, true);
    
    for i=1:length(data)
        write_mat(data{i}, fid, true);
    end

    fclose(fid);

end