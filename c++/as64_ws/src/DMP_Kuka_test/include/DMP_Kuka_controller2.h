/**
 * Copyright (C) 2016 AUTH-ARL
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
#include <plot_lib/plot_lib.h>
#include <math_lib/math_lib.h>
#include <robotics_lib/jlav.h>

class DMP_Kuka_controller : public arl::robot::Controller
{
public:
	void finalize();


	DMP_Kuka_controller(std::shared_ptr<arl::robot::Robot> robot);
	void execute();
	void update();
	bool run();
	bool start;

	ros::NodeHandle n;
	ros::Publisher n_pub;
	ros::Subscriber n_sub;
	std_msgs::Float64MultiArray msg;

	bool enteredListenCallback;
 	void listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	as64_::MovingWinMeanFilter obj_weight_filter;



private:
	Eigen::Matrix<double, JOINT_SIZE, 1> q_prev;
	Eigen::Matrix4d T_robot_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,1> V_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,1> Vd_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,1> Vd_endEffector_prev;
	Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> J_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> J_robot;
	Eigen::Matrix<double,JOINT_SIZE,1> q_dot;
	Eigen::Matrix<double,JOINT_SIZE,1> q;
	Eigen::Matrix<double,JOINT_SIZE,1> qd_dot;
	Eigen::Matrix<double,JOINT_SIZE,1> qd;
	Eigen::Matrix<double,JOINT_SIZE,1> q0;

	KDL::JntArray q_kdl;
	arma::vec q_arma;

	double Ts;

	as64_::JLAv jlav;
	as64_::SingularValueFilter svf;

	std::shared_ptr<arl::robot::Robot> robot_;

	Eigen::Matrix4d robotArm_fkine() const;
	Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> robotArm_jacob() const;
};

#endif  // DMP_KUKA_CONTROLLER_H
