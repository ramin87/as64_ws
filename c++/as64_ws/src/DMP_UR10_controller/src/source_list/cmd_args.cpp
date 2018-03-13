#include <cmd_args.h>
#include <ros/package.h>
#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args(const char *config_file)
{
  std::string path_to_config_file;
  if (config_file != NULL) path_to_config_file = *config_file;
  else path_to_config_file = ros::package::getPath("dmp_ur10_controller")+ "/config/DMP_UR10_controller_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  if (!parser.getParam("a_z", a_z)) a_z = arma::vec().ones(6)*20.0;
  if (!parser.getParam("b_z", b_z)) b_z = a_z/4;
  if (!parser.getParam("DMP_TYPE", DMP_TYPE)) DMP_TYPE = std::vector<std::string>(0);
  if (!parser.getParam("N_kernels", N_kernels)) N_kernels = arma::uvec().ones(6)*40;
  if (!parser.getParam("kernelStdScaling", kernelStdScaling)) kernelStdScaling = 1.0;
  if (!parser.getParam("trainMethod", trainMethod)) trainMethod = "LWR";
  if (!parser.getParam("CAN_CLOCK_TYPE", CAN_CLOCK_TYPE)) CAN_CLOCK_TYPE = "lin";
  if (!parser.getParam("SHAPE_ATTR_GATTING_TYPE", SHAPE_ATTR_GATTING_TYPE)) SHAPE_ATTR_GATTING_TYPE = "lin";
  if (!parser.getParam("SHAPE_ATTR_GATTING_u0", SHAPE_ATTR_GATTING_u0)) SHAPE_ATTR_GATTING_u0 = 1.0;
  if (!parser.getParam("SHAPE_ATTR_GATTING_u_end", SHAPE_ATTR_GATTING_u_end)) SHAPE_ATTR_GATTING_u_end = 0.005;
  if (!parser.getParam("GOAL_ATTR_GATTING_TYPE", GOAL_ATTR_GATTING_TYPE)) GOAL_ATTR_GATTING_TYPE = "lin";
  if (!parser.getParam("GOAL_ATTR_GATTING_u0", GOAL_ATTR_GATTING_u0)) GOAL_ATTR_GATTING_u0 = 0.005;
  if (!parser.getParam("GOAL_ATTR_GATTING_u_end", GOAL_ATTR_GATTING_u_end)) GOAL_ATTR_GATTING_u_end = 1.0;
  if (!parser.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = true;
  if (!parser.getParam("a_g", a_g)) a_g = 20.0;
  if (!parser.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = true;
  if (!parser.getParam("a_px", a_px)) a_px = 50.0;
  if (!parser.getParam("a_py", a_py)) a_py = 40.0;
  if (!parser.getParam("phase_stop_err", phase_stop_err)) phase_stop_err = 0.015;
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

  if (!parser.getParam("lin_vel_lim", lin_vel_lim)) lin_vel_lim = 0.4;
  if (!parser.getParam("rot_vel_lim", rot_vel_lim)) rot_vel_lim = 0.8;

  if (!parser.getParam("Fee_dead_zone", Fee_dead_zone)) Fee_dead_zone = arma::vec().zeros(6);
  if (!parser.getParam("F_norm_retrain_thres", F_norm_retrain_thres)) F_norm_retrain_thres = 100.0;

  if (!parser.getParam("pos_tol_stop", pos_tol_stop)) pos_tol_stop = 0.01;
  if (!parser.getParam("orient_tol_stop", orient_tol_stop)) orient_tol_stop = 0.005;

  if (!parser.getParam("tau_sim_scale", tau_sim_scale)) tau_sim_scale = 1.0;
  if (!parser.getParam("goal_scale", goal_scale)) goal_scale = 1.0;

  if (!parser.getParam("binary", binary)) binary = true;
  if (!parser.getParam("data_input_path", data_input_path)) data_input_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!parser.getParam("data_output_path", data_output_path)) data_output_path = "/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/test/DMP_test/DMP/data/";
  if (!parser.getParam("in_data_filename", in_data_filename)) in_data_filename = "";
  if (!parser.getParam("out_data_filename", out_data_filename)) out_data_filename = "";
  if (!parser.getParam("demo_data_filename", demo_data_filename)) demo_data_filename = "";

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
  out << "a_z: " << a_z.t() << "\n";
  out << "b_z: " << b_z.t() << "\n";
  out << "DMP_TYPE = ";
  as64_::io_::print_vectorString(DMP_TYPE, out);
  out << "\n";
  out << "N_kernels: " << N_kernels.t() << "\n";
  out << "kernelStdScaling: " << kernelStdScaling << "\n";
  out << "trainMethod: " << trainMethod << "\n";
  out << "CAN_CLOCK_TYPE: " << CAN_CLOCK_TYPE << "\n";
  out << "SHAPE_ATTR_GATTING_TYPE: " << SHAPE_ATTR_GATTING_TYPE << "\n";
  out << "SHAPE_ATTR_GATTING_u0: " << SHAPE_ATTR_GATTING_u0 << "\n";
  out << "SHAPE_ATTR_GATTING_u_end: " << SHAPE_ATTR_GATTING_u_end << "\n";
  out << "GOAL_ATTR_GATTING_TYPE: " << GOAL_ATTR_GATTING_TYPE << "\n";
  out << "GOAL_ATTR_GATTING_u0: " << GOAL_ATTR_GATTING_u0 << "\n";
  out << "GOAL_ATTR_GATTING_u_end: " << GOAL_ATTR_GATTING_u_end << "\n";

  out << "USE_GOAL_FILT: " << USE_GOAL_FILT << "\n";
  out << "a_g: " << a_g << "\n";
  out << "USE_PHASE_STOP: " << USE_PHASE_STOP << "\n";
  out << "a_px: " << a_px << "\n";
  out << "a_py: " << a_py << "\n";
  out << "phase_stop_err: " << phase_stop_err << "\n";
  out << "k_trunc_kernel: " << k_trunc_kernel << "\n";
  out << "Wmin: " << Wmin << "\n";
  out << "Freq_min: " << Freq_min << "\n";
  out << "Freq_max: " << Freq_max << "\n";
  out << "P1_min: " << P1_min << "\n";

  out << "Md_p: " << Md_p << "\n";
  out << "Kd_p: " << Kd_p << "\n";
  out << "Dd_p: " << Dd_p << "\n";
  out << "Md_o: " << Md_o << "\n";
  out << "Kd_o: " << Kd_o << "\n";
  out << "Dd_o: " << Dd_o << "\n";

  out << "lin_vel_lim: " << lin_vel_lim << "\n";
  out << "rot_vel_lim: " << rot_vel_lim << "\n";

  out << "Fee_dead_zone: " << Fee_dead_zone.t() << "\n";
  out << "F_norm_retrain_thres: " << F_norm_retrain_thres << "\n";

  out << "pos_tol_stop: " << pos_tol_stop << "\n";
  out << "orient_tol_stop: " << orient_tol_stop << "\n";
  out << "tau_sim_scale: " << tau_sim_scale << "\n";
  out << "goal_scale: " << goal_scale << "\n";

  out << "binary: " << binary << "\n";
  out << "data_input_path: " << data_input_path << "\n";
  out << "data_output_path: " << data_output_path << "\n";
  out << "in_data_filename: " << in_data_filename << "\n";
  out << "out_data_filename: " << out_data_filename << "\n";
  out << "demo_data_filename: " << demo_data_filename << "\n";

  out << "in_CartPos_data_filename: " << in_CartPos_data_filename << "\n";
  out << "out_CartPos_data_filename: " << out_CartPos_data_filename << "\n";

  out << "in_orient_data_filename: " << in_orient_data_filename << "\n";
  out << "out_orient_data_filename: " << out_orient_data_filename << "\n";
}
