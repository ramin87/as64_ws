/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <armadillo>

#include <std_msgs/String.h>

#include <cmd_args.h>
#include <ur10_robot.h>

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "ur10_test_node");
  ros::NodeHandle nh_("~");
  std::string path = ros::package::getPath("ur10_test");

  CMD_ARGS cmd_args;
  cmd_args.parse_cmd_args();
  cmd_args.print();

  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  ros::Rate loop_rate(cmd_args.pub_rate); // loop at 125 Hz

  arma::vec q = arma::vec().randn(6);
  q << -0.4167 << -1.427 << -1.9 << -0.9 << 1.4 << -0.2;

  robot->movej(q, 1.2, 0.8);

  int count = 0;
  while (ros::ok())
  {
    robot->waitNextCycle();
    robot->print_robot_state();

    robot->movej(q, 1.8, 1.2);

    loop_rate.sleep();
    // ros::Duration(0.1).sleep();
    ++count;
  }

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
