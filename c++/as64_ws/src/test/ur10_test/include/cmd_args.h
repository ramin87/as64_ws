#ifndef UR10_TEST_CMD_ARGS_H
#define UR10_TEST_CMD_ARGS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <ros/ros.h>

struct CMD_ARGS
{
  std::string command_ur10_topic;
  double pub_rate;
  std::vector<double> init_joint_pos;
  std::vector<double> goal_joint_pos;
  double joint_vel;
  double joint_accel;
  double Cart_vel;
  double Cart_accel;

  double Md;
  double Kd;
  double Dd;

  CMD_ARGS();
  bool parse_cmd_args();
  void print(std::ostream &out=std::cout) const;
};

#endif // UR10_TEST_CMD_ARGS_H
