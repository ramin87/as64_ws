#include <hand_handover_controller.h>

HandHandoverController::HandHandoverController():robotHand("BH8-282")
{
	hand_type = "BH8-282";
	bHandAction = HandHandoverController::CLOSE_HAND;
	weight_est = 0;

	grasp_force_thres = 400;
}

void HandHandoverController::init()
{
	robotHand.init();

	update();
	for (int i=0;i<NUM_FINGERS;i++) finger_init_force[i] = finger_force[i];
}

void HandHandoverController::listenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	bHandAction = (BarrettHandAction)(msg->data[0]);
	weight_est = msg->data[1];

	std::cout << "I received: " <<  ActionName[(int)(msg->data[0])] << "\n";
}

void HandHandoverController::update()
{
	//bhand.waitNextCycle();
	for (int i=0;i<NUM_FINGERS;i++){
		finger_force[i] = robotHand.getFingerForce(i+1);
		finger_pos[i] = robotHand.getFingerPos(i+1);
		//std::cout << "void Bhand_controller::update_bhand(): finger_pos = " << finger_pos[i] << "\n";
	}
}

void HandHandoverController::grasp_object()
{
/** Start grasping. When every fingers feels the object stop and hold the grasp */
	bool move_finger[NUM_FINGERS] = {true, true, true};

	while (move_finger[0] | move_finger[1] | move_finger[2]) {
		update();

		close();

		for (int i=0;i<NUM_FINGERS;i++){
			move_finger[i] = finger_vel[i] != 0;
			robotHand.setFingerVelocity(finger_vel[i], i+1);
			//std::cout << "CLOSE: finger_vel #" << i+1 << " = " << finger_vel[i] << "\n";
		}
	}
}

void HandHandoverController::release_object()
{
/** Start grasping. When every fingers feels the object stop and hold the grasp */
	bool move_finger[NUM_FINGERS] = {true, true, true};

	while (move_finger[0] | move_finger[1] | move_finger[2]) {
		update();

		open();

		for (int i=0;i<NUM_FINGERS;i++){
			move_finger[i] = finger_vel[i] != 0;
			robotHand.setFingerVelocity(finger_vel[i], i+1);
			//std::cout << "OPEN: finger_vel #" << i+1 << " = " << finger_vel[i] << "\n";
		}
	}
}


void HandHandoverController::terminate()
{
	std::cout << "Bhand: Shutting done...\n";
	robotHand.terminate();
}

void HandHandoverController::open()
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

void HandHandoverController::close()
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

bool HandHandoverController::stop()
{
	for (int i=0;i<NUM_FINGERS;i++) finger_vel[i] = 0;

	return true;
}


bool HandHandoverController::run()
{
	while (bHandAction!=HandHandoverController::TERMINATE_HAND && ros::ok()){
		//std::cout << "Spinned once...\n";
		ros::spinOnce();
		update();
		switch (bHandAction){
			case HandHandoverController::OPEN_HAND:
				open();
				break;
			case HandHandoverController::CLOSE_HAND:
				close();
				break;
			case HandHandoverController::STOP_HAND:
				stop();
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

	return true;
}
