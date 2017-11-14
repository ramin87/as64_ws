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

#include <cmd_args.h>

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

double Fdist_fun(double t, const CMD_ARGS &cmd_args);

#endif
