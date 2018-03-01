/**
 * Copyright (C) 2017 as64_
 */


#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <armadillo>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmd_args.h>
#include <io_lib/io_lib.h>
#include <ur10_robot/ur10_robot.h>


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "ur10_test_node");
  ros::NodeHandle nh_("~");

  // ===========  Read params from yml file  ==================
  CMD_ARGS cmd_args;
  cmd_args.parse_cmd_args();
  cmd_args.print();

  // ===========  Create robot  ==================
  std::shared_ptr<as64_::ur10_::Robot> robot;
  robot.reset(new as64_::ur10_::Robot());

  // ===========  Define control cycle  ==================
  ros::Rate loop_rate(cmd_args.ctr_cycle_rate); // loop at 125 Hz
  double Ts = 1/cmd_args.ctr_cycle_rate;

  // ===========  Move to initial/target configuration  ==================
  arma::vec q_init = cmd_args.init_joint_pos * arma::datum::pi/180;
  arma::vec q_goal = cmd_args.goal_joint_pos * arma::datum::pi/180;

  if (cmd_args.move_to_init_config_flag)
  {
    std::cout << "Movej to initial configuration...\n";
    robot->movej(q_init, cmd_args.joint_vel,  cmd_args.joint_accel);
    ros::Duration(4.0).sleep();
  }

  if (cmd_args.move_to_goal_config_flag)
  {
    std::cout << "Movej to goal configuration...\n";
    robot->movej(q_goal, cmd_args.joint_vel,  cmd_args.joint_accel);
    ros::Duration(4.0).sleep();
  }

  // ===========  Launch thread for printing  ==================
  // robot->launch_printRobotStateThread(cmd_args.print_robotState_rate);

  // ===========  Define the ref model's params  ==================
  double Mr_p = 1.0;
  double Kr_p = 100.0;
  double Dr_p = 2*std::sqrt(Mr_p*Kr_p); // critically damped

  double Mr_o = 1.0;
  double Kr_o = 20.0;
  double Dr_o = 2*std::sqrt(Mr_o*Kr_o); // critically damped

  // ===========  Initialize variables  ==================
  arma::vec p0 = robot->getTaskPosition();
  arma::vec g;
  arma::vec pr = p0;
  arma::vec dpr = arma::vec().zeros(3);
  arma::vec ddpr = arma::vec().zeros(3);

  arma::vec Q0 = robot->getTaskOrientation();
  arma::vec Qg;
  arma::vec Qr = Q0;
  arma::vec v_rot_r = arma::vec().zeros(3);
  arma::vec dv_rot_r = arma::vec().zeros(3);

  arma::mat T1 = robot->getTaskPose();
  arma::vec q1 = robot->getJointPosition();

  arma::mat T2 = robot->forwardKinematic(q1);
  arma::vec q2 = robot->inverseKinematic(T2);

  std::cout << "T1 = \n" << T1 << "\n";
  std::cout << "T2 = \n" << T2 << "\n";

  std::cout << "q1 = \n" << q1.t() << "\n";
  std::cout << "q2 = \n" << q2.t() << "\n";

  exit(-1);

  while (ros::ok())
  {
    robot->waitNextCycle();

    arma::vec ddpr = (1/Mr_p) * ( Kr_p*(g-pr) - Dr_p*dpr);

    loop_rate.sleep();
  }

  robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
