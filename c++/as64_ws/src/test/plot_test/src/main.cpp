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

#include <plot_lib/plot_lib.h>
#include <io_lib/io_lib.h>

using namespace as64_::io_;
using namespace as64_::plot_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "plot_test_node");
  ros::NodeHandle nh_("~");


  // ===========  create data  ==================
  int n_data = 200;
  arma::rowvec t = arma::linspace<arma::rowvec>(0.0, 2.0, n_data);
  arma::vec x = arma::pow(t.t(),2);
  arma::vec y = arma::sin(t.t());
  arma::vec z = arma::sqrt(t.t());

  // ===========  plot data  ==================
  std::cout << "*** Create a graphical object ***\n\n";
  GraphObj gObj;
  wait_for_key();

  std::cout << "*** Set line style ***\n\n";
  gObj.setLineStyle("lines");
  wait_for_key();

  std::cout << "*** Grid on ***\n\n";
  gObj.grid_on();
  wait_for_key();

  std::cout << "*** Plot (t,x) ***\n\n";
  gObj.plot(t,x);
  wait_for_key();

  std::cout << "*** Plot (t,y) ***\n This will overwrite the previous plot\n\n";
  gObj.plot(t,y);
  wait_for_key();

  std::cout << "*** Hold on ***\n\n";
  gObj.hold_on();
  wait_for_key();

  std::cout << "*** Plot (t,x) ***\nNow the previous plot remains due to \"hold on\"\n\n";
  gObj.plot(t,x);
  wait_for_key();

  std::cout << "*** Grid off ***\n\n";
  gObj.grid_off();
  wait_for_key();

  std::cout << "*** Hold off ***\n\n";
  gObj.hold_off();
  wait_for_key();

  std::cout << "*** Plot (t,z) ***\nNow the previous plot are erased due to \"hold off\"\n\n";
  gObj.plot(t,z);
  wait_for_key();

  std::cout << "*** Hold on ***\n\n";
  gObj.hold_on();
  wait_for_key();

  std::cout << "*** Plot (t,x), (t,y), (t,z) ***\nAll plots remain due to \"hold on\"\n\n";
  gObj.plot(t,x);
  gObj.plot(t,y);
  gObj.plot(t,z);
  wait_for_key();


  std::cout << "*** Create a 2nd graphical object ***\n\n";
  GraphObj gObj2;
  wait_for_key();

  std::cout << "*** Plot3 (x,y,z) ***\n\n";
  gObj2.plot3(x, y, z);
  gObj2.title("3D plot");
  gObj2.xlabel("x [m]");
  gObj2.ylabel("y [m]");
  gObj2.zlabel("z [m]");
  wait_for_key();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
