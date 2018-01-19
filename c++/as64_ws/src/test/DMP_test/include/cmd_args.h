#ifndef CMD_ARGS_H
#define CMD_ARGS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <ros/ros.h>

struct CMD_ARGS
{
  double a_z;
  double b_z;
  std::string DMP_TYPE; // "DMP", "DMP-bio", "DMP-plus", "DMP-Shannon"
  int N_kernels; // number of kernels used in the DMP
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
  bool OFFLINE_DMP_TRAINING_enable;
  bool ONLINE_DMP_UPDATE_enable;
  double lambda; // forgetting factor for recursive training methods
  double P_cov; // initial value of covariance matrix for recursive training methods
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
  double Md; // translational inertia
  double Kd; // translational stiffness
  double Dd; // translational damping
  double dt; // simulation time_step
  double tol_stop; // position error tolerance to stop the simulation
  double orient_tol_stop; // orientation error tolerance to stop the simulation
  int max_iters; // maximum iteration steps
  double tau_sim_scale; // scaling factor for the time of the DMP simulation
  double goal_scale; // scaling factor for the goal in the DMP simulation
  bool APPLY_DISTURBANCE; // Flag enabling/disabling the introduction of a disturbance in the robot system
  double Fdist_min; // Minimum disturbance value
  double Fdist_max; // Maximum disturbance value
  double t1_fdist; // Start of Fdist_max
  double t2_fdist; // End of Fdist_max
  bool binary;
  std::string data_input_path;
  std::string data_output_path;
  std::string in_data_filename;
  std::string out_data_filename;

  CMD_ARGS();
  bool parse_cmd_args();
  void print(std::ostream &out=std::cout) const;
};

#endif // CMD_ARGS_H
