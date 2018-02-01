#include <cmd_args.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args()
{
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("a_z", a_z)) a_z = 20.0;
  if (!nh_.getParam("b_z", b_z)) b_z = a_z/4;
  if (!nh_.getParam("DMP_TYPE", DMP_TYPE)) DMP_TYPE = "DMP";
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 80;
  if (!nh_.getParam("kernelStdScaling", kernelStdScaling)) kernelStdScaling = 1.0;
  if (!nh_.getParam("trainMethod", trainMethod)) trainMethod = "LWR";
  if (!nh_.getParam("CAN_CLOCK_TYPE", CAN_CLOCK_TYPE)) CAN_CLOCK_TYPE = "lin";
  if (!nh_.getParam("SHAPE_ATTR_GATTING_TYPE", SHAPE_ATTR_GATTING_TYPE)) SHAPE_ATTR_GATTING_TYPE = "lin";
  if (!nh_.getParam("SHAPE_ATTR_GATTING_u0", SHAPE_ATTR_GATTING_u0)) SHAPE_ATTR_GATTING_u0 = 1.0;
  if (!nh_.getParam("SHAPE_ATTR_GATTING_u_end", SHAPE_ATTR_GATTING_u_end)) SHAPE_ATTR_GATTING_u_end = 0.005;
  if (!nh_.getParam("GOAL_ATTR_GATTING_TYPE", GOAL_ATTR_GATTING_TYPE)) GOAL_ATTR_GATTING_TYPE = "lin";
  if (!nh_.getParam("GOAL_ATTR_GATTING_u0", GOAL_ATTR_GATTING_u0)) GOAL_ATTR_GATTING_u0 = 0.005;
  if (!nh_.getParam("GOAL_ATTR_GATTING_u_end", GOAL_ATTR_GATTING_u_end)) GOAL_ATTR_GATTING_u_end = 1.0;
  if (!nh_.getParam("sigmoid_a_u", sigmoid_a_u)) sigmoid_a_u = 280.0;
  if (!nh_.getParam("OFFLINE_DMP_TRAINING_enable", OFFLINE_DMP_TRAINING_enable)) OFFLINE_DMP_TRAINING_enable = true;
  if (!nh_.getParam("ONLINE_DMP_UPDATE_enable", ONLINE_DMP_UPDATE_enable)) ONLINE_DMP_UPDATE_enable = false;
  if (!nh_.getParam("lambda", lambda)) lambda = 0.99;
  if (!nh_.getParam("P_cov", P_cov)) P_cov = 1000000.0;
  if (!nh_.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = true;
  if (!nh_.getParam("a_g", a_g)) a_g = 20.0;
  if (!nh_.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = true;
  if (!nh_.getParam("a_px", a_px)) a_px = 50.0;
  if (!nh_.getParam("a_py", a_py)) a_py = 40.0;
  if (!nh_.getParam("k_trunc_kernel", k_trunc_kernel)) k_trunc_kernel = 3;
  if (!nh_.getParam("Wmin", Wmin)) Wmin = 0.9999;
  if (!nh_.getParam("Freq_min", Freq_min)) Freq_min = 60.0;
  if (!nh_.getParam("Freq_max", Freq_max)) Freq_max = 150.0;
  if (!nh_.getParam("P1_min", P1_min)) P1_min = 0.008;
  if (!nh_.getParam("Md", Md)) Md = 1.0;
  if (!nh_.getParam("Kd", Kd)) Kd = 50.0;
  if (!nh_.getParam("Dd", Dd)) Dd = 2*std::sqrt(Md*Kd);
  if (!nh_.getParam("F_dead_zone", F_dead_zone)) F_dead_zone = 0.0;
  if (!nh_.getParam("F_retrain_thres", F_retrain_thres)) F_retrain_thres = 100.0;
  if (!nh_.getParam("dt", dt)) dt = 0.002;
  if (!nh_.getParam("tol_stop", tol_stop)) tol_stop = 0.01;
  if (!nh_.getParam("orient_tol_stop", orient_tol_stop)) orient_tol_stop = 0.005;
  if (!nh_.getParam("max_iters", max_iters)) max_iters = 3000;
  if (!nh_.getParam("tau_sim_scale", tau_sim_scale)) tau_sim_scale = 1.0;
  if (!nh_.getParam("goal_scale", goal_scale)) goal_scale = 1.0;
  if (!nh_.getParam("APPLY_DISTURBANCE", APPLY_DISTURBANCE)) APPLY_DISTURBANCE = false;
  if (!nh_.getParam("Fdist_min", Fdist_min)) Fdist_min = 2.0;
  if (!nh_.getParam("Fdist_max", Fdist_max)) Fdist_max = 20.0;
  if (!nh_.getParam("t1_fdist", t1_fdist)) t1_fdist = 0.4;
  if (!nh_.getParam("t2_fdist", t2_fdist)) t2_fdist = 2.2;
  if (!nh_.getParam("binary", binary)) binary = true;
  if (!nh_.getParam("data_input_path", data_input_path)) data_input_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!nh_.getParam("data_output_path", data_output_path)) data_output_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!nh_.getParam("in_data_filename", in_data_filename)) in_data_filename = "";
  if (!nh_.getParam("out_data_filename", out_data_filename)) out_data_filename = "";

  if (!nh_.getParam("in_CartPos_data_filename", in_CartPos_data_filename)) in_CartPos_data_filename = "";
  if (!nh_.getParam("out_CartPos_data_filename", out_CartPos_data_filename)) out_CartPos_data_filename = "";

  if (!nh_.getParam("in_orient_data_filename", in_orient_data_filename)) in_orient_data_filename = "";
  if (!nh_.getParam("out_orient_data_filename", out_orient_data_filename)) out_orient_data_filename = "";

  in_data_filename = data_input_path + "/" + in_data_filename;
  out_data_filename = data_output_path + out_data_filename;

  in_CartPos_data_filename = data_input_path + "/" + in_CartPos_data_filename;
  out_CartPos_data_filename = data_output_path + out_CartPos_data_filename;

  in_orient_data_filename = data_input_path + "/" + in_orient_data_filename;
  out_orient_data_filename = data_output_path + out_orient_data_filename;

  if (USE_PHASE_STOP == false) a_py = 0.0;
}

void CMD_ARGS::print(std::ostream &out) const
{
  out << "a_z: " << a_z << "\n";
  out << "b_z: " << b_z << "\n";
  out << "DMP_TYPE: " << DMP_TYPE << "\n";
  out << "N_kernels: " << N_kernels << "\n";
  out << "kernelStdScaling: " << kernelStdScaling << "\n";
  out << "trainMethod: " << trainMethod << "\n";
  out << "CAN_CLOCK_TYPE: " << CAN_CLOCK_TYPE << "\n";
  out << "SHAPE_ATTR_GATTING_TYPE: " << SHAPE_ATTR_GATTING_TYPE << "\n";
  out << "SHAPE_ATTR_GATTING_u0: " << SHAPE_ATTR_GATTING_u0 << "\n";
  out << "SHAPE_ATTR_GATTING_u_end: " << SHAPE_ATTR_GATTING_u_end << "\n";
  out << "GOAL_ATTR_GATTING_TYPE: " << GOAL_ATTR_GATTING_TYPE << "\n";
  out << "GOAL_ATTR_GATTING_u0: " << GOAL_ATTR_GATTING_u0 << "\n";
  out << "GOAL_ATTR_GATTING_u_end: " << GOAL_ATTR_GATTING_u_end << "\n";
  out << "sigmoid_a_u: " << sigmoid_a_u << "\n";
  out << "OFFLINE_DMP_TRAINING_enable: " << OFFLINE_DMP_TRAINING_enable << "\n";
  out << "ONLINE_DMP_UPDATE_enable: " << ONLINE_DMP_UPDATE_enable << "\n";
  out << "lambda: " << lambda << "\n";
  out << "P_cov: " << P_cov << "\n";
  out << "USE_GOAL_FILT: " << USE_GOAL_FILT << "\n";
  out << "a_g: " << a_g << "\n";
  out << "USE_PHASE_STOP: " << USE_PHASE_STOP << "\n";
  out << "a_px: " << a_px << "\n";
  out << "a_py: " << a_py << "\n";
  out << "k_trunc_kernel: " << k_trunc_kernel << "\n";
  out << "Wmin: " << Wmin << "\n";
  out << "Freq_min: " << Freq_min << "\n";
  out << "Freq_max: " << Freq_max << "\n";
  out << "P1_min: " << P1_min << "\n";
  out << "Md: " << Md << "\n";
  out << "Kd: " << Kd << "\n";
  out << "Dd: " << Dd << "\n";
  out << "F_dead_zone: " << F_dead_zone << "\n";
  out << "F_retrain_thres: " << F_retrain_thres << "\n";
  out << "dt: " << dt << "\n";
  out << "tol_stop: " << tol_stop << "\n";
  out << "orient_tol_stop: " << orient_tol_stop << "\n";
  out << "max_iters: " << max_iters << "\n";
  out << "tau_sim_scale: " << tau_sim_scale << "\n";
  out << "goal_scale: " << goal_scale << "\n";
  out << "APPLY_DISTURBANCE: " << APPLY_DISTURBANCE << "\n";
  out << "Fdist_min: " << Fdist_min << "\n";
  out << "Fdist_max: " << Fdist_max << "\n";
  out << "t1_fdist: " << t1_fdist << "\n";
  out << "t2_fdist: " << t2_fdist << "\n";
  out << "binary: " << binary << "\n";
  out << "data_input_path: " << data_input_path << "\n";
  out << "data_output_path: " << data_output_path << "\n";
  out << "in_data_filename: " << in_data_filename << "\n";
  out << "out_data_filename: " << out_data_filename << "\n";

  out << "in_CartPos_data_filename: " << in_CartPos_data_filename << "\n";
  out << "out_CartPos_data_filename: " << out_CartPos_data_filename << "\n";

  out << "in_orient_data_filename: " << in_orient_data_filename << "\n";
  out << "out_orient_data_filename: " << out_orient_data_filename << "\n";
}
