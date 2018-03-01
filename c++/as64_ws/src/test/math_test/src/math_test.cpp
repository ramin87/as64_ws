/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <math_lib/math_lib.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "math_test_node");
  ros::NodeHandle nh_("~");

  arma::wall_clock timer;
  double elaps_time;
  long iters = 5000;
  Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  Eigen::Vector4d q;

  timer.tic();
  for (int i=0;i<iters;i++)
  {
    math_::rotm2quat(R, false);
  }
  elaps_time = timer.toc();
  std::cout << "======   rotm2quat   =========\n";
  std::cout << "Elapsed time = " << elaps_time << " sec\n";
  std::cout << "Average = " << elaps_time/iters << " sec\n";
  std::cout << "==============================\n";


  timer.tic();
  for (int i=0;i<iters;i++)
  {
    math_::mat2quat(R);
  }
  elaps_time = timer.toc();
  std::cout << "======   mat2quat   =========\n";
  std::cout << "Elapsed time = " << elaps_time << " sec\n";
  std::cout << "Average = " << elaps_time/iters << " sec\n";
  std::cout << "==============================\n";


  timer.tic();
  for (int i=0;i<iters;i++)
  {
    math_::mat2quat_matlab(R);
  }
  elaps_time = timer.toc();
  std::cout << "======   mat2quat_matlab   =========\n";
  std::cout << "Elapsed time = " << elaps_time << " sec\n";
  std::cout << "Average = " << elaps_time/iters << " sec\n";
  std::cout << "==============================\n";


  arma::mat rotm = arma::mat().randu(3,3)*arma::mat().randu(3,3);
  arma::vec quat = math_::rotm2quat(rotm, false);
  arma::mat rotm2 = math_::quat2rotm(quat);

  std::cout << "rotm = \n" << rotm << "\n";
  std::cout << "rotm2 = \n" << rotm2 << "\n";


  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
