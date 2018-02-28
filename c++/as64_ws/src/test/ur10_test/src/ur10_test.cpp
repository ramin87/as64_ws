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

  arma::vec q_init = arma::vec(cmd_args.init_joint_pos);
  q_init = q_init*arma::datum::pi/180;

  arma::vec q_goal = arma::vec(cmd_args.goal_joint_pos);
  q_goal = q_goal*arma::datum::pi/180;

  std::cout << "Movej to initial configuration...\n";
  robot->movej(q_init, cmd_args.joint_vel,  cmd_args.joint_accel);
  ros::Duration(4.0).sleep();
  // std::cout << "Movej to goal configuration...\n";
  // robot->movej(q_goal, cmd_args.joint_vel,  cmd_args.joint_accel);
  // ros::Duration(4.0).sleep();

  arma::vec task_frame(6), selection_vector(6), wrench(6), limits(6);
  int fmode_type = 2;

  task_frame << 0 << 0 << 0 << 0 << 0 << 0;
  selection_vector << 1 << 1 << 1 << 1 << 1 << 1;
  wrench.fill(0.0);
  limits << 0.25 << 0.25 << 0.25 << 0.1 << 0.1 << 0.1;

  // std::cout << "Entering force mode...\n";
  // robot->force_mode(task_frame, selection_vector, wrench, fmode_type, limits);
  // robot->freedrive_mode();
  // ros::Duration(3.0).sleep();

  arma::vec ddEp = arma::vec().zeros(3);
	arma::vec dEp = arma::vec().zeros(3);
  arma::vec Ep = arma::vec().zeros(3);
	arma::vec ddEo = arma::vec().zeros(3);
	arma::vec dEo = arma::vec().zeros(3);
  arma::vec Fdist_p = arma::vec().zeros(3);
  arma::vec Fdist_o = arma::vec().zeros(3);

  double Ts = 1/cmd_args.pub_rate;
  while (ros::ok())
  {
    robot->waitNextCycle();
    robot->print_robot_state();

    // robot->freedrive_mode();
    // robot->sleep(1.0);
    // ros::Duration(1.0).sleep();

    // ddEp = (1.0 / cmd_args.Md) * (- cmd_args.Dd * dEp + Fdist_p);
    // ddEo = (1.0 / cmd_args.Md) * (- cmd_args.Dd * dEo + Fdist_o);
    //
    // dEp = dEp + ddEp * Ts;
    // dEo = dEo + ddEo * Ts;
    // Ep = Ep + dEp * Ts;
    //
    // arma::vec Vd(6);
    // Vd.subvec(0, 2) = dEp; //(dEp + dY - 4.0*(Y_robot - (Y + Ep)));
    // Vd.subvec(3, 5) = arma::vec().zeros(3); //(dEo + v_rot - 4.0*quatLog( quatProd( Q_robot, quatInv(Q) ) ) );
    //
    // robot->speedl(Vd, cmd_args.Cart_accel, Ts);

    loop_rate.sleep();
    // ros::Duration(0.1).sleep();
  }


  exit(-1);

  // std::cout << "Execute movej...\n";
  // robot->movej(q, 1.2, 10.0);
  // std::cout << "Waiting 10 sec...\n";
  // ros::Duration(1.0).sleep();


  arma::vec V(6);
  V << 0.08 << 0.08 << 0.08 << 0.01 << 0.01 << 0.01;

  int count = 0;
  while (ros::ok())
  {
    robot->waitNextCycle();
    robot->print_robot_state();

    // robot->movej(q, 1.8, 1.2);
    robot->speedl(V, 1.0, 0.01);

    loop_rate.sleep();
    // ros::Duration(0.1).sleep();
    ++count;
  }

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
