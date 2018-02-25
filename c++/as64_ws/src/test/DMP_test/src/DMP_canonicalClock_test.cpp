/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <fstream>

#include <dmp_lib/dmp_lib.h>


int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_canonicalClock_test");
  ros::NodeHandle nh("~");

  arma::wall_clock timer;
  double elapsed_time;

  double tau = 4.0;
  std::shared_ptr<as64_::CanonicalClock> canClockPtr(new as64_::LinCanonicalClock());
  canClockPtr->init(tau);

  double t = 1.8;
  arma::rowvec T = arma::linspace<arma::rowvec>(0,tau,10);

  double x = canClockPtr->getPhase(t);
  arma::rowvec X = canClockPtr->getPhase(T);

  double dx = canClockPtr->getPhaseDot(x);
  arma::rowvec dX = canClockPtr->getPhaseDot(X);

  std::cout << "tau = " << canClockPtr->getTau() << "\n";
  std::cout << "t = " << t << "\n";
  std::cout << "x = " << x << "\n";
  std::cout << "dx = " << dx << "\n";
  std::cout << "T = " << T << "\n";
  std::cout << "X = " << X << "\n";
  std::cout << "dX = " << dX << "\n";

  ros::shutdown();

  return 0;
}
