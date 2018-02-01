/**
 * Copyright (C) 2017 as64_
 */

#ifndef DMP_KUKA_CONTROLLER_H
#define DMP_KUKA_CONTROLLER_H

#define _USE_MATH_DEFINES

#define ROBOT_ARM_INDEX 0
#define CART_DOF_SIZE 6
#define JOINT_SIZE 7

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <random>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <mutex>

#include <armadillo>
#include <Eigen/Dense>

#include <lwr_robot/lwr_model.h>
#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/robot_sim.h>
#include <autharl_core/robot/controller.h>


#include <utils.h>
#include <cmd_args.h>

#include <autharl_core/utils/kdl_arma.h>
#include <autharl_core/utils/kdl_eigen.h>
#include <autharl_core/math/orientation.h>
#include <autharl_core/math/skew_symmetric.h>

#include <DMP_lib/DMP_lib.h>
#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>
#include <signalProcessing_lib/signalProcessing_lib.h>
#include <filter_lib/filter_lib.h>
//#include <plot_lib/plot_lib.h>
#include <math_lib/math_lib.h>
// #include <robotics_lib/jlav.h>

using namespace as64_;

class DMP_Kuka_controller : public arl::robot::Controller
{
public:

  DMP_Kuka_controller(std::shared_ptr<arl::robot::Robot> robot);

  bool start;
  void finalize();
  void execute();
  void update();
  void command();

  void read_train_data();

  void train();

  void init_simulation();
  void run_simulation();
  void log_online();
  void log_offline();
  void log_demo();
  void save_logged_data();
  void calc_simulation_mse();

  /**
   * Reads the current joints, position and orientation and saves them
   * in 'q_robot', 'Y_robot', 'Q_robot'.
   * Updates also the 'T_robot_ee' and 'J_robot'
  */
  void read_current_robot_state();

  ros::NodeHandle n;
	ros::Publisher n_pub;
	ros::Subscriber n_sub;
	std_msgs::Float64MultiArray msg;

	bool enteredListenCallback;
 	void listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  void keyboard_ctrl_function();
private:

  bool log_reset;
  bool log_on;
  bool run_dmp;
  bool train_dmp;
  bool goto_start;
  bool pause_robot;
  bool stop_robot;
  bool start_demo;
  bool end_demo;

  std::shared_ptr<std::thread> keyboard_ctrl_thread;

  arma::wall_clock timer;

  CMD_ARGS cmd_args;
  LogData log_data;

  std::shared_ptr<as64_::CanonicalClock> canClockPtr;
  std::shared_ptr<as64_::GatingFunction> shapeAttrGatingPtr;
  std::shared_ptr<as64_::GatingFunction> goalAttrGatingPtr;
  std::vector<std::shared_ptr<as64_::DMP_>> dmp;
  std::shared_ptr<as64_::DMP_CartPos> dmpCartPos;
  std::shared_ptr<as64_::DMP_orient> dmpOrient;

  int n_data;
  arma::rowvec Time_demo;
  arma::mat Yd_data, dYd_data, ddYd_data;
  arma::mat Qd_data, v_rot_d_data, dv_rot_d_data;

  double Ts;
  double tau;
  int Dp;
  int Do;
  int D;

  arma::rowvec Time_offline_train;
  arma::mat F_p_offline_train_data;
  arma::mat Fd_p_offline_train_data;
  arma::vec offline_train_p_mse;
  arma::mat F_o_offline_train_data;
  arma::mat Fd_o_offline_train_data;
  arma::vec offline_train_o_mse;

  int iters;
  double t;
  double dt;
  double x, dx;

  arma::vec Yg0, Yg, Yg2, dg_p;
  arma::vec Y0, Y, dY, ddY;
  arma::vec Y_robot, dY_robot, ddY_robot;
  arma::vec V_robot;
  arma::vec Z, dZ;
  arma::vec Fdist_p;

  arma::vec Qg0, Qg, Qg2, dg_o;
  arma::vec Q0, Q, dQ, v_rot, dv_rot;
  arma::vec Q_robot, v_rot_robot, dv_rot_robot;
  arma::vec eta, deta;
  arma::vec Fdist_o;

  arma::vec F, Fd;

  arma::vec sim_mse;

	arma::vec q0_robot, q_prev_robot, q_robot, dq_robot, qd, dqd;
  arma::mat T_robot_ee;
  arma::mat J_robot;

	//as64_::JLAv jlav;
	//as64_::SingularValueFilter svf;

	std::shared_ptr<arl::robot::Robot> robot_;

	Eigen::Matrix4d robotArm_fkine() const;
	Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> robotArm_jacob() const;
};

#endif
