/**
 * Copyright (C) 2016 AUTH-ARL
 */

#ifndef HANDOVER_CONTROLLER_H
#define HANDOVER_CONTROLLER_H

#define _USE_MATH_DEFINES

#define ROBOT_ARM_INDEX 0

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
#include <DS_lib/DS_GMR/DS_GMR.h>

#include <utils.h>
//#include <arm_handover_controller.h>
//#include <hand_handover_controller.h>

#include <autharl_core/utils/kdl_arma.h>
#include <autharl_core/utils/kdl_eigen.h>
#include <autharl_core/math/orientation.h>
#include <autharl_core/math/skew_symmetric.h>

#include <robotics_lib/control/RobotController.h>
#include <math_lib/math.h>
#include <filter_lib/filter.h>
#include <robotics_lib/jlav.h>
#include <time_lib/time.h>

class HandoverController : public arl::robot::Controller
{
public:
	std::ofstream out_err;

	std::ofstream out_log;

	double global_t;

	int debug_iter;

	double v_scale;

	void finalize();

	// *******************************
	// *********    TIMER   **********
	// *******************************
	as64_::TIMER timer;
	double average_cycle_time;
	double min_cycle_time;
	double max_cycle_time;
	int cycles_count;

	double w_est;
	double w_est_filt;
	enum BarrettHandAction{OPEN_HAND, CLOSE_HAND, STOP_HAND, TERMINATE_HAND};
	BarrettHandAction bHandAction;

	// **********************************************
	// **********************************************

 	double Fee_z_init, Fee_z;
 	double Fee_z_max;


	HandoverController(std::shared_ptr<arl::robot::Robot> robot);
	void execute();
	void update();
	bool run();
	bool start;

	ros::NodeHandle n;
	ros::Publisher n_pub;
	ros::Subscriber sub_camTargTrans;
	std_msgs::Float64MultiArray msg;

	bool received_target;
 	void listenCameraTargetTransformCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	/**
	 * @brief JointPosController::goToJointsTrajectory, moves the robot to target joints
	 * ATTENTION this function is blocking ... untill the motion ends
	 * @param qT, the vector of target joints
	 * @param totalTime, total duration of motion
	 */
	void goToJointsTrajectory(arma::vec qT, arma::vec &q0, double totalTime);

	void goToStartPose(double t);

	/**
	 * @brief JointPosController::get5thOrder, designs a 5th order trajectory
	 * @param t, for the current time instance
	 * @param p0, from this point (initial)
	 * @param pT, to the target
	 * @param totalTime, if the total duaration of motion is this
	 * @return the exact position, velocity and acceleration for that time instance, as columns [p][pdot][pddot]
	 */
	arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

	LogData logData;
	CMD_ARGS cmd_args;

	as64_::MovingWinMeanFilter obj_weight_filter;

	Eigen::Matrix<double, JOINT_SIZE, 1> q_prev;

private:
	boost::shared_ptr<DS::DS_abstract> ds;

	Eigen::Matrix4d T_robot_cam; // initialized once
	Eigen::Matrix4d T_robot_endEffector;
	Eigen::Matrix4d T_cam_robot;  // initialized once
	Eigen::Matrix4d T_cam_obj;
	Eigen::Matrix4d T_cam_hand;
	Eigen::Matrix4d T_endEffector_obj;  // initialized once
	Eigen::Matrix4d T_hand_endEffector;
	Eigen::Matrix4d T_hand_obj;
	Eigen::Matrix4d T_cam_endEffector;
	Eigen::Matrix4d T_obj_endEffector;
	Eigen::Matrix4d T_cam_endEffectorTarget;
	Eigen::Matrix4d T_cam_objTarget;
	Eigen::Matrix4d T_robot_objTarget;
	Eigen::Matrix4d T_endEffectorTarget_endEffector;
	Eigen::Matrix4d T_objTarget_obj;
	Eigen::Matrix<double, 6, 6> Twist_endEffector_obj;

	Eigen::Matrix4d T_hand_obj_dot;
	Eigen::Matrix<double,CART_DOF_SIZE,1> V_obj;
	Eigen::Matrix<double,CART_DOF_SIZE,CART_DOF_SIZE> Torsion_endEffector_obj;
	Eigen::Matrix<double,CART_DOF_SIZE,1> V_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,1> Vd_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,1> Vd_endEffector_prev;
	Eigen::Matrix<double, CART_DOF_SIZE, 1> V_objTarget_obj;
	Eigen::Matrix<double,CART_DOF_SIZE,1> V_objTarget_obj_prev;
	Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> J_endEffector;
	Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> J_robot;
	Eigen::Matrix<double,JOINT_SIZE,1> q_dot;
	Eigen::Matrix<double,JOINT_SIZE,1> q;

	Eigen::Matrix<double,JOINT_SIZE,1> qd_dot;
	Eigen::Matrix<double,JOINT_SIZE,1> qd;

	Eigen::Matrix<double,JOINT_SIZE,1> q0;

	KDL::JntArray q_kdl;
	arma::vec q_arma;

	double max_r;

	double Ts;

	as64_::JLAv jlav;
	as64_::SingularValueFilter svf;

	ros::NodeHandle node_handle_;
	ros::Subscriber sub;
	std::shared_ptr<arl::robot::Robot> robot_;

	Eigen::Matrix4d robotArm_fkine() const;
	Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> robotArm_jacob() const;

	double get_pose_error(const Eigen::Matrix4d &T) const;

	void calc_DS_input(const Eigen::Matrix4d &T_objTarget_obj, Eigen::Matrix<double,6,1> *X_in) const;

	bool zero_vel, move_back, start_move_back;
};

#endif  // HANDOVER_CONTROLLER_H
