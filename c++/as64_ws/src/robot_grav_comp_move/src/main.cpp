/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <lwr_robot/lwr_model.h>
#include <lwr_robot/lwr_robot.h>
#include <robot_grav_comp_controller.h>
#include <ros/ros.h>
#include <memory>
#include <iostream>


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "deformation_node");
  // Create generic robot model
  std::shared_ptr<arl::robot::Model> model;
  // Initialize generic robot model with kuka-lwr model
  model.reset(new lwr::robot::Model());
  // Create generic robot
  std::shared_ptr<arl::robot::Robot> robot;
  // Initialize generic robot with the kuka-lwr model
  robot.reset(new lwr::robot::Robot(model, "Kuka Robot"));
  ROS_INFO_STREAM("Robot created successfully.");
  // // Create controller instance for the kuka-lwr robot
  std::unique_ptr<RobotGravCompController> controller(new RobotGravCompController(robot));
  controller->run();

  return 0;
}
