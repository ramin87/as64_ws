#include "BarrettHand.h"
#include "RobotHand.h"
#include <cstring>
#include <cstdlib>
#include <memory>
#include <exception>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


const char *ActionName[] = {"OPEN_HAND", "CLOSE_HAND", "STOP_HAND", "TERMINATE_HAND"};

class Bhand_controller{
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

	Bhand_controller();

	void init();

	void listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void update_bhand();

	void grasp_object();
	void release_object();

	void shutdown_bhand();

	void open_hand_action();

	void close_hand_action();

	void stop_hand_action();

	void barrett_thread();
private:
};

Bhand_controller::Bhand_controller():robotHand("BH8-282")
{
	hand_type = "BH8-282";
	bHandAction = Bhand_controller::CLOSE_HAND;
	weight_est = 0;

	grasp_force_thres = 400;
}

void Bhand_controller::init()
{
	robotHand.init();

	update_bhand();
	for (int i=0;i<NUM_FINGERS;i++) finger_init_force[i] = finger_force[i];
}

void Bhand_controller::listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	bHandAction = (BarrettHandAction)(msg->data[0]);
	weight_est = msg->data[1];

	std::cout << "I received: " <<  ActionName[(int)(msg->data[0])] << "\n";
}

void Bhand_controller::update_bhand()
{
	//bhand.waitNextCycle();
	for (int i=0;i<NUM_FINGERS;i++){
		finger_force[i] = robotHand.getFingerForce(i+1);
		finger_pos[i] = robotHand.getFingerPos(i+1);
		//std::cout << "void Bhand_controller::update_bhand(): finger_pos = " << finger_pos[i] << "\n";
	}
}

void Bhand_controller::grasp_object()
{
/** Start grasping. When every fingers feels the object stop and hold the grasp */
	bool move_finger[NUM_FINGERS] = {true, true, true};

	while (move_finger[0] | move_finger[1] | move_finger[2]) {
		update_bhand();

		close_hand_action();

		for (int i=0;i<NUM_FINGERS;i++){
			move_finger[i] = finger_vel[i] != 0;
			robotHand.setFingerVelocity(finger_vel[i], i+1);
			//std::cout << "CLOSE: finger_vel #" << i+1 << " = " << finger_vel[i] << "\n";
		}
	}
}

void Bhand_controller::release_object()
{
/** Start grasping. When every fingers feels the object stop and hold the grasp */
	bool move_finger[NUM_FINGERS] = {true, true, true};

	while (move_finger[0] | move_finger[1] | move_finger[2]) {
		update_bhand();

		open_hand_action();

		for (int i=0;i<NUM_FINGERS;i++){
			move_finger[i] = finger_vel[i] != 0;
			robotHand.setFingerVelocity(finger_vel[i], i+1);
			//std::cout << "OPEN: finger_vel #" << i+1 << " = " << finger_vel[i] << "\n";
		}
	}
}


void Bhand_controller::shutdown_bhand()
{
	std::cout << "Bhand: Shutting done...\n";
	robotHand.terminate();
}

void Bhand_controller::open_hand_action()
{
	for (int i=0; i<NUM_FINGERS; i++){
		if (finger_pos[i] < robotHand.max_finger_pos/7){
			finger_vel[i] = 0;
		}else{
			double const_vel = -70;

			double grasp_scale = (finger_force[i] - finger_init_force[i])/grasp_force_thres;
			if (grasp_scale < 0) grasp_scale = 0;
			if (grasp_scale > 1) grasp_scale = 1;
			double grasp_vel = -grasp_scale*50;

			double weight_scale = weight_est/5;
			if (weight_scale < 0) weight_scale = 0;
			if (weight_scale > 1) weight_scale = 1;
			double weight_vel =  -weight_scale * 50;

			finger_vel[i] =  const_vel + grasp_vel + weight_vel;
		}
	}
}

void Bhand_controller::close_hand_action()
{
	for (int i=0; i<NUM_FINGERS; i++){
		if (grasp_force_thres <= finger_force[i] - finger_init_force[i]){
			finger_vel[i] = 0;
		}else{
			double const_vel = 90;

			double grasp_scale = (finger_force[i] - finger_init_force[i])/grasp_force_thres;
			if (grasp_scale < 0) grasp_scale = 0;
			if (grasp_scale > 1) grasp_scale = 1;
			double grasp_vel = -grasp_scale*50;

			double weight_scale = -weight_est/5;
			if (weight_scale < 0) weight_scale = 0;
			if (weight_scale > 1) weight_scale = 1;
			double weight_vel =  weight_scale * 40;

			finger_vel[i] =  const_vel + grasp_vel + weight_vel;
		}
	}
}

void Bhand_controller::stop_hand_action()
{
	for (int i=0;i<NUM_FINGERS;i++) finger_vel[i] = 0;
}


void Bhand_controller::barrett_thread()
{
	while (bHandAction!=Bhand_controller::TERMINATE_HAND && ros::ok()){
		//std::cout << "Spinned once...\n";
		ros::spinOnce();
		update_bhand();
		switch (bHandAction){
			case Bhand_controller::OPEN_HAND:
				open_hand_action();
				break;
			case Bhand_controller::CLOSE_HAND:
				close_hand_action();
				break;
			case Bhand_controller::STOP_HAND:
				stop_hand_action();
				break;
			default:
				break;
		}

		//std::cout << "[BAD] Action: " << ActionName[(int)bHandAction] << "\n";

		for (int i=0;i<NUM_FINGERS;i++){
			robotHand.setFingerVelocity(finger_vel[i], i+1);
			//std::cout << "Finger #" << i+1 << " velocity = " << finger_vel[i] << "\n";
		}
	}
}

int main(int argc, char* argv[])
{
	Bhand_controller bhandController;

	ros::init(argc, argv, "bad");
	ros::NodeHandle n;
	//ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("Barrett_to_Kuka", 1);
	ros::Subscriber sub = n.subscribe("/Kuka_to_Barrett1", 1, &Bhand_controller::listenCallback, &bhandController);


	printf("[BAD] Initializing...\n");
	bhandController.init();
	if (!bhandController.robotHand.initialized()){
		std::cerr << "[BAD ERROR]: Barrett hand not initiallized properly\n";
		exit(-1);
	}

	std::cout << "[BAD] Closing fingers...\n";
	bhandController.grasp_object();
	std::cout << "[BAD] Grasped bject...\n";

	bhandController.bHandAction = Bhand_controller::STOP_HAND;

	bhandController.barrett_thread();

	/*int dummy;
	std::cout << "Enter a dummy value to release object...\n";
	std::cin >> dummy;

	std::cout << "Opening fingers...\n";
	bhandController.release_object();
	std::cout << "Object released...\n";*/

	/*
	bhandController.bHandAction = Bhand_controller::STOP_HAND;
	bhandController.barrett_thread();
	*/

	bhandController.shutdown_bhand();

	return 0;
}
