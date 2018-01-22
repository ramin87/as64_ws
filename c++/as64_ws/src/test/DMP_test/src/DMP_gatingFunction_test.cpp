/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>

#include <DMP_lib/DMP_lib.h>

int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_gatingFunction_test");
  ros::NodeHandle nh("~");

  double u0 = 1.0;
  double u_end = 0.01;

  const int Ntypes = 5;
  char *gatingFunTypes[Ntypes] = {"lin", "exp", "sigmoid", "spring-damper", "constant"};
  std::string gatingFunType;
  if (argc < 2)
  {
    std::cout << "Usage is: rosrun <package_name> <node_name> <gating function type> <u0> <u_end>\n";
    std::cout << "Type of gating function types:\n";
    for (int i=0; i<Ntypes; i++) std::cout << gatingFunTypes[i] << "\n";
    std::cout << "No gating function type was provided.\n Using default: \"lin\"\n";
    gatingFunType = "lin";
  }else{
    gatingFunType = argv[1];
  }

  if (argc >= 3) u0 = std::atof(argv[2]);
  if (argc >= 4) u_end = std::atof(argv[3]);

  std::shared_ptr<as64_::GatingFunction> gatingFunPtr;

  if (gatingFunType.compare("lin")==0)
  {
    gatingFunPtr.reset(new as64_::LinGatingFunction());
  }
  else if (gatingFunType.compare("constant")==0)
  {
    gatingFunPtr.reset(new as64_::ConstGatingFunction());
  }
  else if (gatingFunType.compare("exp")==0)
  {
    gatingFunPtr.reset(new as64_::ExpGatingFunction());
  }
  else if (gatingFunType.compare("sigmoid")==0)
  {
    gatingFunPtr.reset(new as64_::SigmoidGatingFunction());
  }
  else if (gatingFunType.compare("spring-damper")==0)
  {
    gatingFunPtr.reset(new as64_::SpringDamperGatingFunction());
  }
  else
  {
    throw std::runtime_error(std::string("Unsupported gating function type: \"") + gatingFunType + "\"\n");
  }

  gatingFunPtr->init(u0, u_end);

  double x = 0.4;
  arma::rowvec X = arma::linspace<arma::rowvec>(0,1,10);

  double u = gatingFunPtr->getOutput(x);
  double du = gatingFunPtr->getOutputDot(x);

  arma::rowvec U = gatingFunPtr->getOutput(X);
  arma::rowvec dU = gatingFunPtr->getOutputDot(X);

  std::cout << "==========================\n";
  std::cout << "Gating function type: " << gatingFunType << "\n";
  std::cout << "u0 = " << u0 << "\n";
  std::cout << "u_end = " << u_end << "\n";

  std::cout << "x = " << x << "\n";
  std::cout << "u = " << u << "\n";
  std::cout << "du = " << du << "\n";

  std::cout << "X = " << X << "\n";
  std::cout << "U = " << U << "\n";
  std::cout << "dU = " << dU << "\n";

  ros::shutdown();

  return 0;
}
