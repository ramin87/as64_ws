/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <lwr_robot/lwr_model.h>
#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/robot_sim.h>
#include <ros/ros.h>
#include <memory>
#include <iostream>

#include <handover_controller.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "handover_node");
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
  // Create controller instance for the kuka-lwr robot
  std::unique_ptr<HandoverController> controller(new HandoverController(robot));
  // Main loop running the controller

  controller->execute();

  // while(ros::ok() && controller->start) {
  //
  //   //std::cout << "[MAIN]: Entered loop...\n";
  //
  //   // Controller execution
  //   controller->execute();
  //   if (use_sim == 1) {
  //     loop_rate->sleep();
  //   }
  //   // Process ROS callbacks on each loop
  //   ros::spinOnce();
  // }
   std::cout << "[MAIN]: Exited loop...\n";

  controller->finalize();

  //controller->goToStartPose(6);
  ROS_INFO_STREAM("Kuka_handover node is going down.");
  // Stop robot using FRILibrary
  robot->stop();
  // Shutdown ROS node
  ros::shutdown();

  return 0;
}
