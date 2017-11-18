#include <BHControlMain.h>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <deque>

#include <thread>
#include <mutex>


class TIMER
{
public:
	void start() { t1 = std::chrono::high_resolution_clock::now(); }
	void stop() { t2 = std::chrono::high_resolution_clock::now(); }
	double get_elapsed_time() const { return (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1)).count(); }
private:
	std::chrono::high_resolution_clock::time_point t1, t2;
};

TIMER timer;

class HandoverController
{
public:
	// *******************************
	HandoverController();
	
	double finger_pos[3];
	double finger_force[3];
	double grasp_force_thres;
	double finger_vel[3];
	
	bool grasped_object_flag;
	
	double max_finger_pos[3];
	double min_finger_pos[3];
	
	double w_est;
	
	bool is_bhand_okay;
	
	BarrettHand bhand;
	
	enum BarrettHandAction{OPEN_HAND, CLOSE_HAND, STOP_HAND, TERMINATE_HAND};
	
	BarrettHandAction bHandAction;
	
	std::thread bHandThread;
	
	double grasp_force_coeff_o, weight_coeff_o, grasp_force_coeff_c, weight_coeff_c;
	
	void barrett_thread();
	void init_bhand();
	void grasp_object();
	void release_object();
	void update_bhand();
	void shutdown_bhand();
	
	void open_hand_action();
	void close_hand_action();
	void stop_hand_action();

};

void HandoverController::init_bhand()
{
	grasped_object_flag = false;
	
	is_bhand_okay = true;
	
	// **************************
	// ****  parse CMD_ARGS  ****
	// **************************
	//parse_cmd_args();
	//print_cmd_args();
	grasp_force_coeff_o = 10;
	weight_coeff_o = 100;
	grasp_force_coeff_c = 100;
	weight_coeff_c = 20;
	grasp_force_thres = 160;

	// initialize barrett hand
	std::string hand_type = "BH8-280";
	bool RT_control_flag = true;
	
	bhand.initialize(hand_type, RT_control_flag);

	if (!bhand.is_initialized()){
		std::cerr << "Barrett hand initialization failed...\n";
		is_bhand_okay = false;
		return;
	}

	if (!bhand.is_RT_control_enabled()){
		std::cerr << "Barrett hand RT control failed...\n";
		is_bhand_okay = false;
		return;
	}
	
	for (int i=0;i<3;i++){
		max_finger_pos[i] = bhand.get_max_finger_pos(i+1);
		min_finger_pos[i] = bhand.get_min_finger_pos(i+1);
	}
	
	w_est = 0;
	
	update_bhand();
}

void HandoverController::update_bhand()
{
	bhand.waitNextCycle();
	for (int i=0;i<3;i++){
		finger_force[i] = bhand.getFingerForce(i+1);
		finger_pos[i] = bhand.getFingerPosition(i+1);
	}
}

void HandoverController::grasp_object()
{
	int k = 0;
	double vel = 0;
	
	while (k != 7){
		update_bhand();
		
		close_hand_action();
		
		
		for (int i=0; i<3; i++){
			if (grasp_force_thres <= finger_force[i]) k |= (1<<i);
			bhand.setFingerVelocity(finger_vel[i], i+1);
			
			
			std::cout << "k = " << k << "\n";
			std::cout << "Finger #" << i+1 << " velocity = " << finger_vel[i] << "\n";
			/*if (grasp_force_thres <= finger_force[i]){
				vel = 0;
				k |= (1<<i);
			}else{
				vel = 50;
			}
			bhand.setFingerVelocity(vel, i+1);*/
			
		}
	}
	
}

void HandoverController::release_object()
{
	int k = 0;
	double vel = 0;
	
	while (k != 7){
		update_bhand();
		
		open_hand_action();
		
		for (int i=0; i<3; i++){
			if (finger_vel[i] = 0) k |= (1<<i);
			bhand.setFingerVelocity(finger_vel[i], i+1);

			//std::cout << "k = " << k << "\n";
			//std::cout << "Finger #" << i+1 << " velocity = " << finger_vel[i] << "\n";		
		}
	}
	
}
	
void HandoverController::shutdown_bhand()
{
	std::cout << "Bhand: Shutting done...\n";
	bhand.command("123 HOME");
	bhand.terminate();
}

void HandoverController::barrett_thread()
{
	while (bHandAction != HandoverController::TERMINATE_HAND){
		update_bhand();
		switch (bHandAction){
			case HandoverController::OPEN_HAND:
				open_hand_action();
				break;
			case HandoverController::CLOSE_HAND:
				close_hand_action();
				break;
			case HandoverController::STOP_HAND:
				stop_hand_action();
				break;
			default:
				break;
		}
		
		for (int i=0;i<3;i++){
			bhand.setFingerVelocity(finger_vel[i], i+1);
			std::cout << "Finger #" << i+1 << " velocity = " << finger_vel[i] << "\n";
		}
	}
}

void HandoverController::open_hand_action()
{
	for (int i=0; i<3; i++){
		if (finger_pos[i] < max_finger_pos[i]/7){
			finger_vel[i] = 0;
		}else{
			
			finger_vel[i] = -80;
			/*
			double finger_force_diff = (finger_force[i] -grasp_force_thres) / 100;
			if (finger_force_diff > 1) finger_force_diff = 1;
			if (finger_force_diff < 0) finger_force_diff = 0;
			
			double weight_diff = fabs(w_est)/5;
			if (weight_diff > 1) weight_diff = 1;
			
			finger_vel[i] = -grasp_force_coeff_o*finger_force_diff - weight_coeff_o*weight_diff;*/
		}	
	}
}

void HandoverController::close_hand_action()
{
	for (int i=0; i<3; i++){
		if (grasp_force_thres <= finger_force[i]){
			finger_vel[i] = 0;
		}else{
			
			finger_vel[i] = 60;
			
			/*
			double finger_force_diff = (grasp_force_thres - finger_force[i]) / 100;
			if (finger_force_diff > 1) finger_force_diff = 1;
			if (finger_force_diff < -1) finger_force_diff = -1;
			
			double weight_diff = fabs(w_est)/5;
			if (weight_diff > 1) weight_diff = 1;
			
			finger_vel[i] = grasp_force_coeff_c*finger_force_diff + weight_coeff_c*weight_diff;*/
		}
	}
}

void HandoverController::stop_hand_action()
{
	for (int i=0;i<3;i++){
		finger_vel[i] = 0;
	}
}


HandoverController::HandoverController()
{
	std::cout << "Initializing Bhand...\n";
	init_bhand();
	std::cout << "Closing Bhand...\n";
	grasp_object();
	std::cout << "Grasped object!\n";
	
	int wait_time = 3;
	std::cout << "Waiting " << wait_time << " sec...\n";
	timer.start();
	while (timer.get_elapsed_time() < wait_time);
	timer.stop();
	
	std::cout << "Opening Bhand...\n";
	release_object();
	
	std::cout << "Bhand finger forces:\n";
	for (int i=0;i<3;i++) std::cout << "#" << i+1 << " force = " << finger_force[i] << "\n";
	
	bhand.command("123 HOME");
	bhand.terminate();
}

int main(int argc, char* argv[])
{
	HandoverController controller;
	
    /*std::string hand_type = "BH8-280";
    bool RT_control_flag = true;
	BarrettHand bhand;

	bhand.initialize(hand_type, RT_control_flag);

	if (!bhand.is_initialized()){
        std::cerr << "Barrett hand initialization failed...\n";
        return -1;
	}

	if (!bhand.is_RT_control_enabled()){
        std::cerr << "Barrett hand RT control failed...\n";
        return -1;
	}

    double finger_pos[3];
    double finger_vel[3];
    double finger_force[3];

    double max_finger_pos[3];
    double min_finger_pos[3];

    for (int i=0;i<3;i++){
        max_finger_pos[i] = bhand.get_max_finger_pos(i+1);
        min_finger_pos[i] = bhand.get_min_finger_pos(i+1);
    }
    
    int stop_hand = 0;
    finger_vel[0] = finger_vel[1] = finger_vel[2] = 100;
    while (true){
        bhand.waitNextCycle();
        for (int i=0;i<3;i++){

            finger_pos[i] = bhand.getFingerPosition(i+1);
            //finger_vel[i] = bhand.getFingerVelocity(i+1);
            finger_force[i] = bhand.getFingerForce(i+1);

            if (finger_force[i] > 150){
                std::cout << "#" << i << ": STOP barrett!\n";
                bhand.setFingerVelocity(0, i+1);
                stop_hand |= (1<<i);
            }else{
                std::cout << "#" << i << ": MOVE barrett!\n";
                std::cout << finger_vel[i] << "\n";
                bhand.setFingerVelocity(finger_vel[i], i+1);
            }
        }
        if (stop_hand == 7) break;
    }

	//bhand.command("123 HOME");
	
    bhand.terminate();
    */
	return 0;
}

