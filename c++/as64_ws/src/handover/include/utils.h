#ifndef ROBOT_ARM_UTILS_64_H
#define ROBOT_ARM_UTILS_64_H

#define CART_DOF_SIZE 6
#define JOINT_SIZE 7

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <pcl/console/parse.h>

/*
 * Pauses the program until the user presses the keyboard
 */ 
void pause_program();

void print_error_message(const std::string &err_msg);

/*
 * Converts a number to string represantation
 */
std::string numToString(double number);

void mirror_Htrans_wrt_yz_plane(Eigen::Matrix4d *T_mirror);
void mirror_Velocity_wrt_yz_plane(Eigen::Matrix<double, 6, 1> *V_mirror);

void transform_Jacob(const Eigen::Matrix4d &T_a_b, const Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> &J_a, Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> &J_b);

struct LogData{
	double Ts;
	std::vector<Eigen::Matrix<double, JOINT_SIZE, 1> > q_data;
	std::vector<Eigen::Matrix<double, JOINT_SIZE, 1> > dq_data;
	std::vector<Eigen::Matrix<double, CART_DOF_SIZE, 1> > V_endEffector_data;
	std::vector<Eigen::Matrix<double, CART_DOF_SIZE, 1> > V_objTarget_obj_data;
	std::vector<Eigen::Matrix<double, 4, 4> > T_cam_obj_data;
	std::vector<Eigen::Matrix<double, 4, 4> > T_cam_objTarget_data;
	std::vector<double> Fee_z_data;
	std::vector<double> v_scale_data;
	std::vector<bool> mirror_occured_data;
	
	void push(const Eigen::Matrix<double, JOINT_SIZE, 1> &q, 
				const Eigen::Matrix<double, JOINT_SIZE, 1> &dq, 
				const Eigen::Matrix<double, CART_DOF_SIZE, 1> &V_endEffector,
				const Eigen::Matrix<double, CART_DOF_SIZE, 1> &V_objTarget_obj,
				const Eigen::Matrix<double, 4, 4> &T_cam_obj,
				const Eigen::Matrix<double, 4, 4> &T_cam_objTarget,
				double Fee_z,
				const double &v_scale,
				const bool &mirror_occured);
	bool save(const std::string &filename);
};

struct CMD_ARGS{
	int p_ch;
	
	bool LOG_DATA;
	std::string output_path;
	
	bool USE_MIRRORING;
	double MIRROR_THRES;
	double MIRROR_SMOOTH_THRES;
	double OLT_THRES;
	int WEIGHT_EST_FILT_N;
	
	double TOL_STOP;
	double START_MOVE_THRES;
	double HIGH_VEL_TOL_STOP;
	double kV;
	double ks;
	double VEL_SCALE;
	bool V_Vd_equal;
	double MAX_R;
	double V_SCALE_C;
	double V_SCALE_EXP;
	bool NORM_VEL;
	
	double POS_ERR_COEFF;
	double ORIENT_ERR_COEFF;
	
	double c;
	
	std::string params_path;
	std::string DS_params_filename;
	std::string init_pose_filename;
	std::string target_pose_filename;
	std::string T_endEffector_obj_filename;
	std::string T_robot_cam_filename;
	
	std::string Kuka_to_Barrett_topic;
	std::string cam_target_transform_topic;
	
	// ====== Joint Limit Avoidance (Jla) ===========
	bool JLAV_enable;
	double JLAV_gain;

	// ====== Singular Value Filtering (SVF) ========
	bool SVF_enable;
	double SVF_sigma_min;
	double SVF_shape_factor;

	CMD_ARGS();
	CMD_ARGS(int program_execution_choice, double vel_scale, double tol_stop, bool log_data, bool use_mirroring);
	bool parse_cmd_args(int argc, char **argv);
	bool parse_cmd_args(const std::string &filename);
	bool parse_cmd_args();
	void print(std::ostream &out=std::cout) const;
};

#endif
