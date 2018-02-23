#ifndef UR10_ROBOT_H
#define UR10_ROBOT_H

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <armadillo>

namespace ur10_
{

class Robot
{
  struct RobotState
  {
    arma::vec q, dq;
    arma::mat pose;
    arma::mat Jrobot;
    arma::vec pos, Q;
    arma::vec v_lin, v_rot;
    arma::vec wrench;
    arma::vec jTorques;

    RobotState()
    {
      q.resize(6);
      dq.resize(6);
      pose.resize(3,4);
      pos.resize(3);
      Q.resize(4);
      v_lin.resize(3);
      v_rot.resize(3);
      wrench.resize(6);
      jTorques.resize(6);
    }
  };

public:
  Robot();

  void freedrive_mode() const;
  void end_freedrive_mode() const;

  void teach_mode() const;
  void end_teach_mode() const;

  void force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits)  const;
  void end_force_mode() const;
  void force_mode_set_damping(double damping)  const;

  void movej(const arma::vec &q, double a=1.4, double v=1.05, double t=0, double r=0) const;
  void movel(const arma::vec &p, double a=1.2, double v=0.25, double t=0, double r=0) const;

  void speedj(arma::vec dq, double a, double t=-1) const;
  void speedl(arma::vec dp, double a, double t=-1) const;

  void stopj(double a) const;
  void stopl(double a) const;
  //
  // void position_deviation_warning(enabled, threshold=0.8)  const;

  void set_gravity(const arma::vec &g) const;
  void set_payload(double m, const arma::vec &CoG) const;
  void set_payload_cog(const arma::vec &CoG) const;
  void set_payload_mass(double m) const;
  void set_tcp(const arma::vec &pose) const;

  void sleep(double t) const;
  void powerdown() const;

  void waitNextCycle();

  arma::vec getJointPosition() const { return rSt.q; }
  arma::vec getJointVelocity() const { return rSt.dq; }
  arma::mat getTaskPose() const { return rSt.pose; }
  arma::mat getTaskPosition() const { return rSt.pos; }
  arma::mat getTaskOrientation() const { return rSt.Q; }

  arma::vec getTwist() const { return arma::join_vert(rSt.v_lin, rSt.v_rot); }
  arma::vec getExternalWrench() const { return rSt.wrench; }
  arma::vec getJointTorque() const { return rSt.jTorques; }
  arma::mat getJacobian() const { return rSt.Jrobot; }


  // void setJointVelocity(const arma::vec &input);
  // void setJointTorque(const arma::vec &input);
  // setWrench
  // setTaskPose

  bool isOk() const;

  void print_robot_state(std::ostream &out=std::cout) const;

private:

  RobotState rSt;

  ros::NodeHandle n;

  std::string command_ur10_topic;
  std::string read_wrench_topic;
  std::string read_toolVel_topic;
  std::string read_jointState_topic;
  std::string base_frame;
  std::string tool_frame;

  double pub_rate;

  ros::Publisher pub2ur10;
  ros::Subscriber wrench_sub;
  ros::Subscriber toolVel_sub;
  ros::Subscriber jointState_sub;

  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  geometry_msgs::TransformStamped transformStamped;

  std::string print_vector(const arma::vec &v) const
  {
    std::ostringstream out;
    out << "[" << v(0);
    for (int i=1;i<v.size();i++) out << "," << v(i);
    out << "]";

    return out.str();
  }

  void urScript_command(const std::string &cmd) const
  {
    std_msgs::String cmd_str;
    cmd_str.data = cmd;
    pub2ur10.publish(cmd_str);
  }

  // Callbacks
  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void readToolVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void readJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void readTaskPoseCallback();

};

} // namespace ur10_

#endif
