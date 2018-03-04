#include <cmd_args.h>
#include <ros/package.h>
#include <param_lib/param_lib.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args(const char *config_file)
{
  std::string path_to_config_file;
  if (config_file != NULL) path_to_config_file = *config_file;
  else path_to_config_file = ros::package::getPath("dmp_ur10_controller")+ "/config/DMP_UR10_controller_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  if (!parser.getParam("a_z", a_z)) a_z = 20.0;
  if (!parser.getParam("b_z", b_z)) b_z = a_z/4;
  if (!parser.getParam("DMP_TYPE", DMP_TYPE)) DMP_TYPE = "DMP";
  if (!parser.getParam("N_kernels", N_kernels))
  {
    N_kernels.resize(6);
    for (int i=0;i<6;i++) N_kernels[i] = 80;
  }
  if (!parser.getParam("kernelStdScaling", kernelStdScaling)) kernelStdScaling = 1.0;
  if (!parser.getParam("trainMethod", trainMethod)) trainMethod = "LWR";
  if (!parser.getParam("CAN_CLOCK_TYPE", CAN_CLOCK_TYPE)) CAN_CLOCK_TYPE = "lin";
  if (!parser.getParam("SHAPE_ATTR_GATTING_TYPE", SHAPE_ATTR_GATTING_TYPE)) SHAPE_ATTR_GATTING_TYPE = "lin";
  if (!parser.getParam("SHAPE_ATTR_GATTING_u0", SHAPE_ATTR_GATTING_u0)) SHAPE_ATTR_GATTING_u0 = 1.0;
  if (!parser.getParam("SHAPE_ATTR_GATTING_u_end", SHAPE_ATTR_GATTING_u_end)) SHAPE_ATTR_GATTING_u_end = 0.005;
  if (!parser.getParam("GOAL_ATTR_GATTING_TYPE", GOAL_ATTR_GATTING_TYPE)) GOAL_ATTR_GATTING_TYPE = "lin";
  if (!parser.getParam("GOAL_ATTR_GATTING_u0", GOAL_ATTR_GATTING_u0)) GOAL_ATTR_GATTING_u0 = 0.005;
  if (!parser.getParam("GOAL_ATTR_GATTING_u_end", GOAL_ATTR_GATTING_u_end)) GOAL_ATTR_GATTING_u_end = 1.0;
  if (!parser.getParam("sigmoid_a_u", sigmoid_a_u)) sigmoid_a_u = 280.0;
  if (!parser.getParam("OFFLINE_DMP_TRAINING_enable", OFFLINE_DMP_TRAINING_enable)) OFFLINE_DMP_TRAINING_enable = true;
  if (!parser.getParam("ONLINE_DMP_UPDATE_enable", ONLINE_DMP_UPDATE_enable)) ONLINE_DMP_UPDATE_enable = false;
  if (!parser.getParam("lambda", lambda)) lambda = 0.99;
  if (!parser.getParam("P_cov", P_cov)) P_cov = 1000000.0;
  if (!parser.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = true;
  if (!parser.getParam("a_g", a_g)) a_g = 20.0;
  if (!parser.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = true;
  if (!parser.getParam("a_px", a_px)) a_px = 50.0;
  if (!parser.getParam("a_py", a_py)) a_py = 40.0;
  if (!parser.getParam("k_trunc_kernel", k_trunc_kernel)) k_trunc_kernel = 3;
  if (!parser.getParam("Wmin", Wmin)) Wmin = 0.9999;
  if (!parser.getParam("Freq_min", Freq_min)) Freq_min = 60.0;
  if (!parser.getParam("Freq_max", Freq_max)) Freq_max = 150.0;
  if (!parser.getParam("P1_min", P1_min)) P1_min = 0.008;
  if (!parser.getParam("Md_p", Md_p)) Md_p = 2.0;
  if (!parser.getParam("Kd_p", Kd_p)) Kd_p = 500.0;
  if (!parser.getParam("Dd_p", Dd_p)) Dd_p = 2*std::sqrt(Md_p*Kd_p);
  if (!parser.getParam("Md_o", Md_o)) Md_o = 2.0;
  if (!parser.getParam("Kd_o", Kd_o)) Kd_o = 500.0;
  if (!parser.getParam("Dd_o", Dd_o)) Dd_o = 2*std::sqrt(Md_o*Kd_o);
  if (!parser.getParam("Fp_dead_zone", Fp_dead_zone)) Fp_dead_zone = 0.0;
  if (!parser.getParam("Fo_dead_zone", Fo_dead_zone)) Fo_dead_zone = 0.0;
  if (!parser.getParam("F_norm_retrain_thres", F_norm_retrain_thres)) F_norm_retrain_thres = 100.0;
  if (!parser.getParam("dt", dt)) dt = 0.002;
  if (!parser.getParam("tol_stop", tol_stop)) tol_stop = 0.01;
  if (!parser.getParam("orient_tol_stop", orient_tol_stop)) orient_tol_stop = 0.005;
  if (!parser.getParam("max_iters", max_iters)) max_iters = 3000;
  if (!parser.getParam("tau_sim_scale", tau_sim_scale)) tau_sim_scale = 1.0;
  if (!parser.getParam("goal_scale", goal_scale)) goal_scale = 1.0;
  if (!parser.getParam("APPLY_DISTURBANCE", APPLY_DISTURBANCE)) APPLY_DISTURBANCE = false;
  if (!parser.getParam("Fdist_min", Fdist_min)) Fdist_min = 2.0;
  if (!parser.getParam("Fdist_max", Fdist_max)) Fdist_max = 20.0;
  if (!parser.getParam("t1_fdist", t1_fdist)) t1_fdist = 0.4;
  if (!parser.getParam("t2_fdist", t2_fdist)) t2_fdist = 2.2;
  if (!parser.getParam("binary", binary)) binary = true;
  if (!parser.getParam("data_input_path", data_input_path)) data_input_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!parser.getParam("data_output_path", data_output_path)) data_output_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!parser.getParam("in_data_filename", in_data_filename)) in_data_filename = "";
  if (!parser.getParam("out_data_filename", out_data_filename)) out_data_filename = "";

  if (!parser.getParam("in_CartPos_data_filename", in_CartPos_data_filename)) in_CartPos_data_filename = "";
  if (!parser.getParam("out_CartPos_data_filename", out_CartPos_data_filename)) out_CartPos_data_filename = "";

  if (!parser.getParam("in_orient_data_filename", in_orient_data_filename)) in_orient_data_filename = "";
  if (!parser.getParam("out_orient_data_filename", out_orient_data_filename)) out_orient_data_filename = "";

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
  for (int i=0;i<N_kernels.size();i++) std::cout << "N_kernels[" << i+1 << "] = " << N_kernels[i] << "\n";
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
  out << "Md_p: " << Md_p << "\n";
  out << "Kd_p: " << Kd_p << "\n";
  out << "Dd_p: " << Dd_p << "\n";
  out << "Fp_dead_zone: " << Fp_dead_zone << "\n";
  out << "Md_o: " << Md_o << "\n";
  out << "Kd_o: " << Kd_o << "\n";
  out << "Dd_o: " << Dd_o << "\n";
  out << "Fo_dead_zone: " << Fo_dead_zone << "\n";
  out << "F_norm_retrain_thres: " << F_norm_retrain_thres << "\n";
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