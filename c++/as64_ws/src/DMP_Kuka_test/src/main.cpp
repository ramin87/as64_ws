/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <DMP_Kuka_controller.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "DMP_Kuka_test");
  ros::NodeHandle nh("~");
  ros::Rate *loop_rate;
  int use_sim = 0;
  nh.getParam("use_sim", use_sim);
  // Create generic robot model
  std::shared_ptr<arl::robot::Model> model;
  // Initialize generic robot model with kuka-lwr model
  model.reset(new lwr::robot::Model());
  // Create generic robot
  std::shared_ptr<arl::robot::Robot> robot;
  if (use_sim == 0) {
    // Initialize generic robot with the kuka-lwr model
    robot.reset(new lwr::robot::Robot(model, "Kuka Robot"));
    ROS_INFO_STREAM("Robot created successfully.");
  } else {
    // Initialize generic robot with the kuka-lwr model
    robot.reset(new arl::robot::RobotSim(model, 0.01));
    loop_rate = new ros::Rate(1/robot->cycle);
    ROS_INFO_STREAM("Simulation robot created successfully.");
  }
  // =========  Create controller instance for the kuka-lwr robot  =========
  std::unique_ptr<DMP_Kuka_controller> controller(new DMP_Kuka_controller(robot));

  // =========  Main loop running the controller  =========

  controller->execute();

  std::cout << "[MAIN]: Exited loop...\n";

  controller->finalize();

  ROS_INFO_STREAM("Kuka_handover node is going down.");
  // Stop robot using FRILibrary
  robot->stop();
  // Shutdown ROS node
  ros::shutdown();

  return 0;
}
