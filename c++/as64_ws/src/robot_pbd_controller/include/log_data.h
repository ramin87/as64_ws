#ifndef DMP_TEST_LOG_DATA_H
#define DMP_TEST_LOG_DATA_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <memory>

// #include <dmp_lib/dmp_lib.h>
#include <io_lib/io_lib.h>

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
  std::vector<arma::mat> Psi_data_train;

  arma::rowvec Time;
  arma::mat y_data;
  arma::mat dy_data;
  arma::mat ddy_data;
  arma::mat z_data;
  arma::mat dz_data;
  arma::rowvec x_data;
  arma::rowvec goalAttr_data;
  arma::rowvec shapeAttr_data;
  std::vector<arma::mat> Psi_data;

  arma::mat y_robot_data;
  arma::mat dy_robot_data;
  arma::mat ddy_robot_data;

  arma::mat Fdist_data;
  arma::mat Force_term_data;
  arma::mat g_data;

  std::vector<arma::mat> DMP_w;
  std::vector<arma::mat> DMP_c;
  std::vector<arma::mat> DMP_h;

  bool poseDataFlag;

  LogData();
  void save(std::string filename, bool binary=false, int precision = 6);
  void clear();
};

#endif // LOG_DATA_H
