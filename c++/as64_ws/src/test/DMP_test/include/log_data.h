#ifndef LOG_DATA_H
#define LOG_DATA_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <memory>

#include <DMP_lib/DMP/DMP_.h>
#include <IO_lib/io_utils.h>
#include <cmd_args.h>

struct LogData
{
  arma::rowvec Time_demo;
  arma::mat yd_data;
  arma::mat dyd_data;
  arma::mat ddyd_data;
  
  int D;
  double Ts;
  arma::vec g0;
  
  arma::rowvec Time_offline_train;
  arma::mat F_offline_train_data;
  arma::mat Fd_offline_train_data;
  
  arma::rowvec Time_online_train;
  arma::mat F_online_train_data;
  arma::mat Fd_online_train_data;
  
  arma::rowvec Time;
  arma::mat y_data;
  arma::mat dy_data;
  arma::mat z_data;
  arma::mat dz_data;
  arma::rowvec x_data;
  arma::rowvec u_data;
  
  arma::mat y_robot_data;
  arma::mat dy_robot_data;
  
  arma::rowvec Fdist_data;
  
  arma::mat Force_term_data;
  arma::mat g_data;
  
  std::vector<arma::mat> Psi_data;
  arma::mat shape_attr_data;
  arma::mat goal_attr_data;
  
  std::vector<std::shared_ptr<as64::DMP_>> dmp;
  std::vector<arma::mat> P_lwr;
  std::vector<arma::mat> DMP_w;
  
  CMD_ARGS cmd_args;
  
  LogData();
  
  void save(const std::string &filename, bool binary=false, int precision = 6);
};


#endif // LOG_DATA_H
