/**
 * Copyright (C) 2017 AUTH-ARL
 */

#ifndef ROBOT_GRAVITY_COMPENSATION_CONTROLLER_H
#define ROBOT_GRAVITY_COMPENSATION_CONTROLLER_H

#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/controller.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <ctime>

#include <IO_lib/io_utils.h>

class RobotGravCompController : public arl::robot::Controller
{
public:
	RobotGravCompController(std::shared_ptr<arl::robot::Robot> robot);
	void init();
	void readParameters();
	void printParameters();
	void reset();
	void setImpedanceParams();
	void measure();
	void CartImpedance();
	void JointImpedance();
	void update();
	void command();

	bool run();
	bool stop();
	void keyboardControl();

private:
	int N_JOINTS;

	double Ts;

	double time, zeta;
	bool set_init_config;
	ros::NodeHandle nh_;
	arma::vec qdot, u, p, p_ref, Q, xdot, Qd, e_p,  e_o,  pdot, quatDiff, forces;
	arma::vec q_init, q, q_ref, friction;
	arma::mat pose, pose_ref, J, Jt, Jr, K_imp, D_imp, R, R_ref;

	//constraints
	arma::vec e_maze, pd_path, v_ref;
	double rate_limit, v_err;

	std::string control_method;
	std::string filename;

	double stiff_transl, stiff_rot, damp_transl, damp_rot; //impedance free-space

	bool record_path;
	bool record_current_pose;
	bool stop_program;
	bool reset_to_init_config;
};

#endif // KUKA_GRAVITY_COMPENSATION_CONTROLLER_H
