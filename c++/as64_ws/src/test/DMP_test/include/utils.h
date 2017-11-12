#ifndef DMP_TEST_UTILS_64_H
#define DMP_TEST_UTILS_64_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>
#include <boost/concept_check.hpp>
#include <armadillo>

#include <ros/ros.h>


struct CMD_ARGS{
  
  // ====  Set up DMP params  ====
  double a_z;
  b_z;
  x0;
  x_end;
  N_kernels;
  std_K;
  DMP_TYPE;
  train_method;
  CAN_SYS_TYPE;

  OFFLINE_DMP_TRAINING_enable;
  ONLINE_DMP_UPDATE_enable;

  RLWR_lambda;
  RLWR_P;

  USE_GOAL_FILT;
  a_g;
  USE_PHASE_STOP;
  a_px: 100;
  a_py: -80;

  // ====  demos preprocess params ====
  add_points_percent;
  smooth_points_percent: 0.03

  // ====  Robot controller params  ====
  Kd: 100
  Dd: 10

  // ====  Set up DMP params  ====
  APPLY_DISTURBANCE: true
  Fdist_min: 5
  Fdist_max: 80
  t1_fdist: 0.4
  t2_fdist: 2.2
      
  // ====  Simulation params  ====
  sim_time_step: 0.002
  sim_tol_stop: 0.001
  sim_max_iters: 5000
  tau_sim_scale: 1
  goal_sim_scale: 1

  // ====  Apply disturbance force  ====
  APPLY_DISTURBANCE = false;
  Fdist_min = 0;
  Fdist_max = 200;
  t1_fdist = 0.4;
  t2_fdist = 2.2;

  // ====  Path to input-output data  ====
  binary: false
  data_input_path: "/home/slifer/Dropbox/64631466/lib/matlab/DMP/DMP_temp/"
  data_output_path: "/home/slifer/Dropbox/64631466/lib/matlab/DMP/DMP_temp/"

  in_data_filename: "data.txt"
  out_data_filename: "data_out.txt"

  
  double a_z;
  double b_z;
  double x0, x_end;
  double N_kernels;
  double std_K;
  std::string train_method;
  std::string CAN_SYS_TYPE;
  
  double add_points_percent;
  double smooth_points_percent;
  
  bool USE_GOAL_FILT;
  double a_g;
  bool USE_PHASE_STOP;
  double a_px;
  double a_py;

  double Kd;
  double Dd;
  
  bool APPLY_DISTURBANCE;
  double Fdist_min;
  double Fdist_max;
  double t1_fdist;
  double t2_fdist;
       
  double sim_time_step;
  double sim_tol_stop;
  int sim_max_iters;
  double tau_sim_scale;
  
  std::string data_input_path;
  std::string data_output_path;
  
  std::string in_data_filename;
  std::string out_data_filename;
  

  CMD_ARGS();
  bool parse_cmd_args();
  void print(std::ostream &out=std::cout) const;
};


struct LogData
{
  std::ofstream out;
  
  CMD_ARGS cmd_args;
  
  arma::rowvec Time_train;
  arma::mat F_train_data;
  arma::mat Fd_train_data;
  
  int D;
  double Ts;
  
  arma::rowvec Time_demo;
  arma::mat yd_data;
  arma::mat dyd_data;
  arma::mat ddyd_data;
  
  std::vector<double> Time;
  arma::mat dy_data;
  arma::mat y_data;
  arma::mat dy_robot_data;
  arma::mat y_robot_data;
  arma::mat z_data;
  arma::mat dz_data;
  std::vector<double> x_data;
  std::vector<double> u_data;
  std::vector<double> Fdist_data;
  arma::mat Force_term_data;
  arma::mat g_data;
  arma::vec g0;
  std::vector<arma::mat> Psi_data;
  arma::mat shape_attr_data;
  arma::mat goal_attr_data;
  
  LogData(int D);
  
  void print_mat(const arma::mat &m);
  
  void print_vec(const arma::vec &v);
  void print_vec(const std::vector<double> &v);
  
  void print_rowVec(const std::vector<double> &v);
  void print_rowVec(const arma::rowvec &v);
  
  void print_vec_vec(const std::vector<arma::rowvec> &m);
  void print_vec_mat(const std::vector<arma::mat> &m);
  
  
  
  void save(const std::string &filename);
};


void movingAverageFilter(const arma::rowvec &y, arma::rowvec &y_filt, int win_n);

void process_demos(const arma::mat &data, double Ts, arma::mat &yd_data, arma::mat &dyd_data, arma::mat &ddyd_data, double add_points_percent=0.01, double smooth_points_percent=0.02);

void load_data(const std::string &data_file_name, arma::mat &data, double &Ts);

double Fdist_fun(double t, const CMD_ARGS &cmd_args)
{
  double Fmax = cmd_args.Fdist_max;
  double Fmin = cmd_args.Fdist_min;
  double t1 = cmd_args.t1_fdist;
  double t2 = cmd_args.t2_fdist;

  double tf = t2 - t1;

  double tb = tf*0.15;
  double a = (Fmax - Fmin) / tb;

  double Fdist = Fmin;

  if (t>t1 && t<t2){
      if (t < t1+tb) Fdist = a*(t-t1) + Fmin;
      else if (t < t2-tb) Fdist = Fmax;
      else Fdist = -a*(t-t2) + Fmin;
  }
  
  return Fdist;

}

#endif
