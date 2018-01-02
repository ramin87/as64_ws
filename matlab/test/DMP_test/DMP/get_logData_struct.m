function log_data = get_logData_struct()

log_data = struct('Time_demo',[], 'yd_data',[], 'dyd_data',[], 'ddyd_data',[], ...
                  'D',[], 'Ts',[], 'g0', [], ...
                  'Time_offline_train',[], 'F_offline_train_data',[], 'Fd_offline_train_data',[], ...
                  'Time_online_train',[], 'F_online_train_data',[], 'Fd_online_train_data',[], ...
                  'Time',[], 'y_data',[], 'dy_data',[], 'ddy_data',[], 'z_data',[], 'dz_data',[], ...
                  'x_data',[], 'shapeAttrGating_data',[], 'goalAttrGating_data',[], ...
                  'y_robot_data',[], 'dy_robot_data',[], 'ddy_robot_data',[], ...
                  'Fdist_data',[], 'Force_term_data',[], 'g_data',[], ...
                  'Psi_data',[], 'shape_attr_data',[], 'goal_attr_data',[], ...
                  'dmp',[], 'DMP_w',[], 'P_lwr',[]);

end

