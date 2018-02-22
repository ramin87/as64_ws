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


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "IO_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("ur10_test");


  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
