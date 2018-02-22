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

#include <DMP_UR10_controller.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "DMP_UR10_controller");
  ros::NodeHandle nh("~");

  std::shared_ptr<ur10_::Model> model;
  // Initialize generic robot model with kuka-lwr model
  model.reset(new ur10_::Model());
  // Create generic robot
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot(model, "UR10"));

  ROS_INFO_STREAM("Simulation robot created successfully.");

  // =========  Create controller instance for the kuka-lwr robot  =========
  std::unique_ptr<DMP_UR10_controller> controller(new DMP_UR10_controller(robot));

  // =========  Main loop running the controller  =========

  controller->execute();

  std::cout << "[MAIN]: Exited loop...\n";

  controller->finalize();

  ROS_INFO_STREAM("DMP_UR10_controller node is going down.");

  // Stop robot using FRILibrary
  robot->stop();
  // Shutdown ROS node
  ros::shutdown();

  return 0;
}
