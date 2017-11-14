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
#include <log_data.h>


void movingAverageFilter(const arma::rowvec &y, arma::rowvec &y_filt, int win_n);

void process_demos(const arma::mat &data, double Ts, arma::mat &yd_data, arma::mat &dyd_data, arma::mat &ddyd_data, double add_points_percent=0.01, double smooth_points_percent=0.02);

void load_data(const std::string &data_file_name, arma::mat &data, double &Ts, bool binary);

double Fdist_fun(double t, const CMD_ARGS &cmd_args);

#endif
