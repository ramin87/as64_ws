#ifndef HAND_HANDOVER_CONTROLLER_H
#define HAND_HANDOVER_CONTROLLER_H

#include <BHand_lib/RobotHand.h>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <exception>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <lwr_robot/lwr_model.h>
#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/controller.h>

const char *ActionName[] = {"OPEN_HAND", "CLOSE_HAND", "STOP_HAND", "TERMINATE_HAND"};

class HandHandoverController : public arl::robot::Controller
{
public:
	enum BarrettHandAction{OPEN_HAND, CLOSE_HAND, STOP_HAND, TERMINATE_HAND};

	BarrettHandAction bHandAction;
	double weight_est;

	double finger_pos[NUM_FINGERS];
	double finger_force[NUM_FINGERS];
	double finger_init_force[NUM_FINGERS];
	double finger_vel[NUM_FINGERS];

	std::string hand_type;
	RobotHand robotHand;

	double grasp_force_thres;

	HandHandoverController();

	void init();

	void listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void update();

	void grasp_object();
	void release_object();

	void terminate();

	void open();

	void close();

	bool stop();

	bool run();
private:
};


#endif
