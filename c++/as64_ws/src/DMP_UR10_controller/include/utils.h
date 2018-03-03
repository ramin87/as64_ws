#ifndef DMP_TEST_UTILS_64_H
#define DMP_TEST_UTILS_64_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>
#include <boost/concept_check.hpp>
#include <memory>
#include <armadillo>

#include <ros/ros.h>

#include <dmp_lib/dmp_lib.h>
#include <param_lib/param_lib.h>

#include <cmd_args.h>
#include <log_data.h>

void get_canClock_gatingFuns_DMP(const CMD_ARGS &cmd_args, int D, double tau,
  std::shared_ptr<as64_::CanonicalClock> &canClockPtr,
  std::shared_ptr<as64_::GatingFunction> &shapeAttrGatingPtr,
  std::shared_ptr<as64_::GatingFunction> &goalAttrGatingPtr,
  std::vector<std::shared_ptr<as64_::DMP_>> &dmp);


void load_data(const std::string &data_file_name, arma::mat &yd_data, arma::mat &dyd_data,
               arma::mat &ddyd_data, arma::rowvec &Time_demo, bool binary);

double Fdist_fun(double t, const CMD_ARGS &cmd_args);

arma::rowvec join_horiz(const arma::rowvec &v, double a);
arma::mat join_horiz(const arma::mat &v, const arma::mat &v2);

#endif
