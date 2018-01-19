/**
 * Copyright (C) 2017 AUTH-ARL
 */

#ifndef ROBOT_GRAVITY_COMPENSATION_CONTROLLER_H
#define ROBOT_GRAVITY_COMPENSATION_CONTROLLER_H


#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <memory> // for std::shared_ptr
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/controller.h>

#include <time_lib/time.h>
#include <io_lib/io_utils.h>

class RobotGravCompController : public arl::robot::Controller
{
public:
	RobotGravCompController(std::shared_ptr<arl::robot::Robot> robot);
	void init();
	void readParameters();
	void printParameters(std::ostream &out = std::cout);
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

	void write_recorded_poses();

private:

	std::vector<arma::mat> Poses;
	std::vector<arma::vec> Joints;

	//std::string getTimeStamp() const;

	std::string DIR; ///< the path to the \a robot_grav_comp_move package data file
	int N_JOINTS; ///< number robot's joints

	double Ts; ///< sampling time for recording a path

	double time; ///< the time elapsed since the start of the program
	double zeta;
	bool set_init_config;
	ros::NodeHandle nh_;
	arma::vec qdot, u, p, p_ref, Q, xdot, Qd, e_p,  e_o,  pdot, quatDiff, forces;
	arma::vec q_init, q, q_ref, friction;
	arma::mat pose, pose_ref, J, Jt, Jr, K_imp, D_imp, R, R_ref;

	//constraints
	arma::vec v_ref;
	double rate_limit, v_err;

	std::string control_method; ///< control method for the robot
	std::string filename; ///< the name of the file for logging data
	bool  binary; ///< true for binary output format, false for text format

	double stiff_transl; ///< impedance translation stiffness
	double stiff_rot; ///< impedance rotation stiffness
	double damp_transl; ///< impedance translation damping
	double damp_rot; ///< impedance rotation damping

	bool record_path; ///< set to true to start recording a path
	bool record_current_pose; ///< set to true to record the current robot pose-joints
	bool stop_program; ///< stops the programs control loop
	bool reset_to_init_config; ///< set to true to move the robot to its initial configuration \a q_init
};

#endif // KUKA_GRAVITY_COMPENSATION_CONTROLLER_H
