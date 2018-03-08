#ifndef DMP_TEST_CMD_ARGS_H
#define DMP_TEST_CMD_ARGS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <ros/ros.h>

#include <armadillo>

struct CMD_ARGS
{
  double a_z;
  double b_z;
  std::string DMP_TYPE; // "DMP", "DMP-bio", "DMP-plus", "DMP-Shannon"
  arma::vec N_kernels;
  double kernelStdScaling; // scaling factor for the kernels std
  std::string trainMethod; // "LWR", "LS", "RLS" , "RLWR"
  std::string CAN_CLOCK_TYPE;
  std::string SHAPE_ATTR_GATTING_TYPE; // "lin", "exp", "spring-damper", "sigmoid", "constant"
  double SHAPE_ATTR_GATTING_u0; // starting value of the shape attractor gating
  double SHAPE_ATTR_GATTING_u_end; // ending value of the shape attractor gating
  std::string GOAL_ATTR_GATTING_TYPE; // "lin", "exp", "spring-damper", "sigmoid", "constant"
  double GOAL_ATTR_GATTING_u0; // starting value of the goal attractor gating
  double GOAL_ATTR_GATTING_u_end; // ending value of the goal attractor gating
  double sigmoid_a_u; // steepness of the sigmoid gating function (optional)
  bool USE_GOAL_FILT;
  double a_g;
  bool USE_PHASE_STOP;
  double a_px;
  double a_py;
  int k_trunc_kernel; // number of stds beyond which the kernel is truncated
  double Wmin;
  double Freq_min;
  double Freq_max;
  double P1_min;
  double Md_p; // translational inertia
  double Kd_p; // translational stiffness
  double Dd_p; // translational damping
  double Md_o; // rotational inertia
  double Kd_o; // rotational stiffness
  double Dd_o; // rotational damping
  arma::vec Fee_dead_zone;
  double F_norm_retrain_thres;
  double dt; // simulation time_step
  double pos_tol_stop; // position error tolerance to stop the simulation
  double orient_tol_stop; // orientation error tolerance to stop the simulation
  double tau_sim_scale; // scaling factor for the time of the DMP simulation
  double goal_scale; // scaling factor for the goal in the DMP simulation

  bool binary;

  std::string data_input_path;
  std::string data_output_path;

  std::string in_data_filename;
  std::string out_data_filename;
  std::string demo_data_filename;

  std::string  in_CartPos_data_filename;
  std::string  out_CartPos_data_filename;

  std::string  in_orient_data_filename;
  std::string  out_orient_data_filename;

  CMD_ARGS();
  bool parse_cmd_args(const char *config_file=NULL);
  void print(std::ostream &out=std::cout) const;
};

#endif // DMP_TEST_CMD_ARGS_H
