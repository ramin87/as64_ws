#include <cmd_args.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args()
{
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("a_z", a_z)) a_z = 40;
  if (!nh_.getParam("b_z", b_z)) b_z = 10;
  if (!nh_.getParam("x0", x0)) x0 = 1;
  if (!nh_.getParam("x_end", x_end)) x_end = 0.005;
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 50;
  if (!nh_.getParam("std_K", std_K)) std_K = 0.95;
  if (!nh_.getParam("DMP_TYPE", DMP_TYPE)) DMP_TYPE = "DMP";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  if (!nh_.getParam("CAN_SYS_TYPE", CAN_SYS_TYPE)) CAN_SYS_TYPE = "lin";
  if (!nh_.getParam("OFFLINE_DMP_TRAINING_enable", OFFLINE_DMP_TRAINING_enable)) OFFLINE_DMP_TRAINING_enable = true;
  if (!nh_.getParam("ONLINE_DMP_UPDATE_enable", ONLINE_DMP_UPDATE_enable)) ONLINE_DMP_UPDATE_enable = false;
  if (!nh_.getParam("RLWR_lambda", RLWR_lambda)) RLWR_lambda = 0.99;
  if (!nh_.getParam("RLWR_P", RLWR_P)) RLWR_P = 1e8;
  if (!nh_.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = false;
  if (!nh_.getParam("a_g", a_g)) a_g = 10;
  if (!nh_.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = true;
  if (!nh_.getParam("a_px", a_px)) a_px = 100;
  if (!nh_.getParam("a_py", a_py)) a_py = -80;
  if (!nh_.getParam("add_points_percent", add_points_percent)) add_points_percent = 0.01;
  if (!nh_.getParam("smooth_points_percent", smooth_points_percent)) smooth_points_percent = 0.03;
  if (!nh_.getParam("Kd", Kd)) Kd = 100;
  if (!nh_.getParam("Dd", Dd)) Dd = 10;
  if (!nh_.getParam("APPLY_DISTURBANCE", APPLY_DISTURBANCE)) APPLY_DISTURBANCE = true;
  if (!nh_.getParam("Fdist_min", Fdist_min)) Fdist_min = 5;
  if (!nh_.getParam("Fdist_max", Fdist_max)) Fdist_max = 80;
  if (!nh_.getParam("t1_fdist", t1_fdist)) t1_fdist = 0.4;
  if (!nh_.getParam("t2_fdist", t2_fdist)) t2_fdist = 2.2;
  if (!nh_.getParam("sim_time_step", sim_time_step)) sim_time_step = 0.002;
  if (!nh_.getParam("sim_tol_stop", sim_tol_stop)) sim_tol_stop = 0.001;
  if (!nh_.getParam("sim_max_iters", sim_max_iters)) sim_max_iters = 5000;
  if (!nh_.getParam("tau_sim_scale", tau_sim_scale)) tau_sim_scale = 1;
  if (!nh_.getParam("goal_sim_scale", goal_sim_scale)) goal_sim_scale = 1;
  if (!nh_.getParam("binary", binary)) binary = false;
  if (!nh_.getParam("data_input_path", data_input_path)) data_input_path = "/home/slifer/Dropbox/64631466/lib/matlab/DMP/DMP_temp/";
  if (!nh_.getParam("data_output_path", data_output_path)) data_output_path = "/home/slifer/Dropbox/64631466/lib/matlab/DMP/DMP_temp/";
  if (!nh_.getParam("in_data_filename", in_data_filename)) in_data_filename = "data.txt";
  if (!nh_.getParam("out_data_filename", out_data_filename)) out_data_filename = "data_out.txt";

}

void CMD_ARGS::print(std::ostream &out) const
{
  out << "a_z: " << a_z << "\n";
  out << "b_z: " << b_z << "\n";
  out << "x0: " << x0 << "\n";
  out << "x_end: " << x_end << "\n";
  out << "N_kernels: " << N_kernels << "\n";
  out << "std_K: " << std_K << "\n";
  out << "DMP_TYPE: " << DMP_TYPE << "\n";
  out << "train_method: " << train_method << "\n";
  out << "CAN_SYS_TYPE: " << CAN_SYS_TYPE << "\n";
  out << "OFFLINE_DMP_TRAINING_enable: " << OFFLINE_DMP_TRAINING_enable << "\n";
  out << "ONLINE_DMP_UPDATE_enable: " << ONLINE_DMP_UPDATE_enable << "\n";
  out << "RLWR_lambda: " << RLWR_lambda << "\n";
  out << "RLWR_P: " << RLWR_P << "\n";
  out << "USE_GOAL_FILT: " << USE_GOAL_FILT << "\n";
  out << "a_g: " << a_g << "\n";
  out << "USE_PHASE_STOP: " << USE_PHASE_STOP << "\n";
  out << "a_px: " << a_px << "\n";
  out << "a_py: " << a_py << "\n";
  out << "add_points_percent: " << add_points_percent << "\n";
  out << "smooth_points_percent: " << smooth_points_percent << "\n";
  out << "Kd: " << Kd << "\n";
  out << "Dd: " << Dd << "\n";
  out << "APPLY_DISTURBANCE: " << APPLY_DISTURBANCE << "\n";
  out << "Fdist_min: " << Fdist_min << "\n";
  out << "Fdist_max: " << Fdist_max << "\n";
  out << "t1_fdist: " << t1_fdist << "\n";
  out << "t2_fdist: " << t2_fdist << "\n";
  out << "sim_time_step: " << sim_time_step << "\n";
  out << "sim_tol_stop: " << sim_tol_stop << "\n";
  out << "sim_max_iters: " << sim_max_iters << "\n";
  out << "tau_sim_scale: " << tau_sim_scale << "\n";
  out << "goal_sim_scale: " << goal_sim_scale << "\n";
  out << "binary: " << binary << "\n";
  out << "data_input_path: " << data_input_path << "\n";
  out << "data_output_path: " << data_output_path << "\n";
  out << "in_data_filename: " << in_data_filename << "\n";
  out << "out_data_filename: " << out_data_filename << "\n";

}

