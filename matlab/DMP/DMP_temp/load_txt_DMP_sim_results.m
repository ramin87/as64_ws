function get_logData_struct();

filename = 'data_out.txt';
fid = fopen(filename,'r');

if (fid < 0), error(['Failed to open file ' filename]); end

log_data = get_logData_struct();

log_data.D = fscanf(fid,'%i', 1);

log_data.Ts = fscanf(fid,'%f', 1);

log_data.g0 = read_vec(fid);

% ========   Time_demo, yd_data, dyd_data, ddyd_data  ==========

log_data.Time_demo = read_rowVec(fid);

log_data.yd_data = read_mat(fid);
log_data.dyd_data = read_mat(fid);
log_data.ddyd_data = read_mat(fid);

% ========   Time_train, F_train_data, Fd_train_data  ==========

log_data.Time_train = read_rowVec(fid);

log_data.F_train_data = read_mat(fid);
log_data.Fd_train_data = read_mat(fid);

% ===========  Time, y_data, dy_data, y_robot_data, dy_robot_data, z_data, dz_data  ============

log_data.Time = read_rowVec(fid);

log_data.y_data = read_mat(fid);
log_data.dy_data = read_mat(fid);

log_data.y_robot_data = read_mat(fid);
log_data.dy_robot_data = read_mat(fid);

log_data.z_data = read_mat(fid);
log_data.dz_data = read_mat(fid);

% =========== x_data, u_data, Fdist_data, Force_term_data, g_data  =============

log_data.x_data = read_rowVec(fid);
log_data.u_data = read_rowVec(fid);
log_data.Fdist_data = read_rowVec(fid);
log_data.Force_term_data = read_mat(fid);
log_data.g_data = read_mat(fid);

% =========== Psi_data, shape_attr_data, goal_attr_data  =============

log_data.shape_attr_data = read_mat(fid);
log_data.goal_attr_data = read_mat(fid);
log_data.Psi_data = read_vec_mat(fid);

fclose(fid);

cmd_args = get_cmd_args();

save dmp_results.mat log_data cmd_args;

end




