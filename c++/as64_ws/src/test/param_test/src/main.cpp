/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <vector>
#include <exception>

#include <armadillo>

#include <param_lib/param_lib.h>

using namespace arma;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "param_test_node");
  ros::NodeHandle nh_("~");

  std::string params_path = ros::package::getPath("param_test") + "/config/config.yml";

  // Set defaults
  rowvec def_vec      = randu<rowvec>(10);
  cx_vec def_vec_cx   = randu<cx_vec>(10);
  mat def_mat         = randu<mat>(5,5);
  cx_mat def_matx     = randu<cx_mat>(5,5);
  Col<int> def_vec_int;
  def_vec_int << 1 << 2 << 3 << 4;

  // Create parser
  as64_::param_::Parser testpar(params_path);

  std::string my_text;
  if (!testpar.getParam("my_text", my_text))
  {
    std::cerr << "Couldn't find key \"my_text\"... Setting to default.\n";
    my_text = "DEFAULT";
  }

  int N;
  if (!testpar.getParam("N", N))
  {
    std::cerr << "Couldn't find key \"N\"... Setting to default.\n";
    N = -1;
  }

  double pi_val;
  if (!testpar.getParam("pi_val",pi_val))
  {
    std::cerr << "Couldn't find key \"pi_val\"... Setting to default.\n";
    pi_val = 0.0;
  }

  rowvec x;
  if (!testpar.getParam("x",x))
  {
    std::cerr << "Couldn't find key \"x\"... Setting to default.\n";
    x = rowvec().zeros(2);
  }

  cx_vec y;
  if (!testpar.getParam("y",y))
  {
    std::cerr << "Couldn't find key \"y\"... Setting to default.\n";
    y = cx_vec(2);
  }

  mat A;
  if (!testpar.getParam("A",A))
  {
    std::cerr << "Couldn't find key \"A\"... Setting to default.\n";
    A = mat().zeros(2,2);
  }

  cx_mat B;
  if (!testpar.getParam("B",B))
  {
    std::cerr << "Couldn't find key \"B\"... Setting to default.\n";
    B = cx_mat(2,2);
  }

  Col<int> Z;
  if (!testpar.getParam("Z",Z))
  {
    std::cerr << "Couldn't find key \"Z\"... Setting to default.\n";
    Z = Col<int>().zeros(2);
  }

  std::cout << "my_text= " << my_text << std::endl;
  std::cout << "N= " << N << std::endl;
  std::cout << "pi_val= " << pi_val << std::endl;
  std::cout << "x= \n" << x << std::endl;
  std::cout << "y= \n" << y << std::endl;
  std::cout << "A= \n" << A << std::endl;
  std::cout << "B= \n" << B << std::endl;
  std::cout << "Z= \n" << Z << std::endl;

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
