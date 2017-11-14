#ifndef CMD_ARGS_H
#define CMD_ARGS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <ros/ros.h>

struct CMD_ARGS
{
  int a_z; 
  int b_z; // default: set to a_z/4
  int x0; 
  double x_end; 
  int N_kernels; 
  double std_K; 
  std::string DMP_TYPE; // "DMP", "DMP-bio", "DMP-plus"
  std::string train_method; // "LWR", "LS", "RLS" , "RLWR"
  std::string CAN_SYS_TYPE; // "lin", "exp", "spring-damper"
  bool OFFLINE_DMP_TRAINING_enable; 
  bool ONLINE_DMP_UPDATE_enable; 
  double RLWR_lambda; 
  int RLWR_P; 
  bool USE_GOAL_FILT; 
  int a_g; 
  bool USE_PHASE_STOP; 
  int a_px; 
  int a_py; // default: set to 0 if (USE_PHASE_STOP==false)
  double add_points_percent; 
  double smooth_points_percent; 
  int Kd; 
  int Dd; 
  bool APPLY_DISTURBANCE; 
  int Fdist_min; 
  int Fdist_max; 
  double t1_fdist; 
  double t2_fdist; 
  double sim_time_step; 
  double sim_tol_stop; 
  int sim_max_iters; 
  int tau_sim_scale; 
  int goal_sim_scale; 
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
