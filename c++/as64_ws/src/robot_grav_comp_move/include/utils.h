#ifndef DMP_TEST_UTILS_64_H
#define DMP_TEST_UTILS_64_H

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>
#include <boost/concept_check.hpp>
#include <armadillo>

#include <ros/ros.h>

void load_data(const std::string &data_file_name, arma::mat &data, double &Ts);

struct CMD_ARGS{
  
  double a_z;
  double b_z;
  bool USE_GOAL_FILT;
  double a_g;
  double x_end;
  double N_kernels;
  double ts_scale;
  double std_K;
  bool USE_PHASE_STOP;
  double a_px;
  double a_py;
  
  std::string can_sys_type;
  std::string train_method;
  
  double sim_time_step;
  int sim_max_iters;
  
  std::string data_input_path;
  std::string data_output_path;
  
  std::string in_data_filename;
  std::string out_data_filename;
  

  CMD_ARGS();
  bool parse_cmd_args(ros::NodeHandle &nh_);
  void print(std::ostream &out=std::cout) const;
};

struct LogData
{
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
  
  LogData() {}
  LogData(int D);
  
  void print_mat(const arma::mat &m, std::ostream &out = std::cout);
  
  void print_vec(const arma::vec &v, std::ostream &out = std::cout);
  void print_vec(const std::vector<double> &v, std::ostream &out = std::cout);
  
  void print_rowVec(const std::vector<double> &v, std::ostream &out = std::cout);
  void print_rowVec(const arma::rowvec &v, std::ostream &out = std::cout);
  
  void print_vec_vec(const std::vector<arma::rowvec> &m, std::ostream &out = std::cout);
  void print_vec_mat(const std::vector<arma::mat> &m, std::ostream &out = std::cout);
  
  void save_train_sim_data(const std::string &filename);
  void save_demo_data(const std::string &filename);
};

#endif
