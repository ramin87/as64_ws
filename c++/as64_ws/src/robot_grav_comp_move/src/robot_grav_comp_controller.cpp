/**
 * Copyright (C) 201N_JOINTS AUTH-ARL
*/

#include <robot_grav_comp_controller.h>


RobotGravCompController::RobotGravCompController(std::shared_ptr<arl::robot::Robot> robot)
{
	nh_ = ros::NodeHandle("~");
	this->robot = robot;

	this->DIR = ros::package::getPath("robot_grav_comp_move") + "/data/";

	N_JOINTS = robot->model->getNrOfJoints(0);

	// get the parameters from the yaml file
	readParameters();
	printParameters(std::cout);

	init();
}

void RobotGravCompController::init()
{
	time = 0.0;

	rate_limit = 0.5; //Maximum velocity of reference path to avoid large bumps. Limits the maximum execution speed

	u.zeros(N_JOINTS);
	q.zeros(N_JOINTS);
	q_ref.zeros(N_JOINTS);
	qdot.zeros(N_JOINTS);
	pose.zeros(3, 4);
	pose_ref.zeros(3, 4);
	J.resize(6, N_JOINTS);
	Jt.resize(3, N_JOINTS);
	Jr.resize(3, N_JOINTS);
	xdot.zeros(6);
	forces.zeros(6);
	friction.zeros(N_JOINTS);
	p.zeros(3);
	p_ref.zeros(3);
	K_imp.eye(6, 6);
	D_imp.eye(6, 6);
	R.eye(3, 3);
	R_ref.eye(3, 3);

	Q.zeros(4); Q(0)=1;
	Qd.zeros(4); Qd(0)=1;
	quatDiff.zeros(4); quatDiff(0)=1;

	e_p.zeros(3);
	e_o.zeros(3);

	v_ref.zeros(3);

	D_imp(0,0)=D_imp(1,1)=D_imp(2,2)=0;

	record_path = false;
	record_current_pose = false;
	stop_program = false;
	reset_to_init_config = false;
}

void RobotGravCompController::readParameters()
{
	std::cout << "[readParameters] Reading Parameter list ..." << std::endl;
	// load parameters and gains
	if (!nh_.getParam("stiff_transl", stiff_transl)) stiff_transl = 0;
	if (!nh_.getParam("stiff_rot", stiff_rot)) stiff_rot = 0;
	if (!nh_.getParam("damp_transl", damp_transl)) damp_transl = 0;
	if (!nh_.getParam("damp_rot", damp_rot)) damp_rot = 0;
	if (!nh_.getParam("zeta", zeta)) zeta = 0;
	if (zeta<0.0) zeta=0.0;
	else if (zeta>1.0) zeta=1.0;

	if (!nh_.getParam("Ts", Ts)) Ts = 0.001;

	if (!nh_.getParam("filename", filename)) filename = "data";

	if (!nh_.getParam("binary", binary)) binary = false;

	if (!nh_.getParam("set_init_config", set_init_config)) set_init_config = false;

	std::vector<double> vec;
	if (set_init_config) nh_.getParam("init_config", vec);
	q_init = arma::vec(vec);

	//get control method for contact
	nh_.getParam("control_method", control_method);
	if(!control_method.compare("Cartesian")) //compare() returns 0 if equal.
		std::cout << "Using Cartesian Impedance mode" << std::endl;
	else if (!control_method.compare("Joint"))
		std::cout << "Using Joint Impedance" << std::endl;
	else {
		std::cout << "Method: " << control_method << " is unknown. Candidates are: Cartesian, Joint" << std::endl;
		std::cout << "Using Cartesian instead." << std::endl;
		control_method = "Cartesian"; //default method
	}

	// Print messages
	std::cout << "[readParameters] Parameters are successfully loaded. " << std::endl;

}

void RobotGravCompController::printParameters(std::ostream &out)
{
	out << "stiff_transl = " << stiff_transl << std::endl;
	out << "damp_transl = " << damp_transl << std::endl;
	out << "stiff_rot = " << stiff_rot << std::endl;
	out << "damp_rot = " << damp_rot << std::endl;
	out << "zeta = " << zeta << std::endl;
	out << "Ts = " << Ts << std::endl;
	out << "control_method = " << stiff_transl << std::endl;
	out << "set_init_config = " << set_init_config << std::endl;
	out << "init_config = " << q_init.t() << std::endl;
	out << "filename = " << filename << std::endl;
	out << "binary = " << binary << std::endl;
}

void RobotGravCompController::reset()
{
	/*
	v_ref.zeros();
	p_ref = p; //mirror current position
	e_maze.zeros();
	setImpedanceParams();

	//open gripper
	std_msgs::Bool open_gripper;
	open_gripper.data = true;
	gripper_pub.publish(open_gripper);
	gripper_closed = false;

	path->reset();
	cout << "Resetting completed!" << endl;
	*/
}

void RobotGravCompController::setImpedanceParams()
{
	//Impedance parameters
	arma::vec temp_k, temp_c;
	temp_k << stiff_transl << stiff_transl << stiff_transl << stiff_rot << stiff_rot << stiff_rot; //stiffness
	K_imp = diagmat(temp_k);
	temp_c << damp_transl << damp_transl << damp_transl << damp_rot << damp_rot << damp_rot; //damping
	D_imp = diagmat(temp_c);

	if(!control_method.compare("Cartesian")) { //compare() returns 0 if equal.
		robot->setCartStiffness(temp_k);
		temp_c.fill(zeta);
		robot->setCartDamping(temp_c);
	}
}


void RobotGravCompController::measure()
{
	// std::cout <<"start measure()" << std::endl;
	// read pose from robot
	robot->getTaskPose(pose);

	robot->getTwist(xdot); //read Cartesian velocity (J*q_dot) [Does not work well in Cartesian impedance]
	if(!control_method.compare("Cartesian")) //because there is a bug in xdot for this mode
		xdot.subvec(0,2) = (pose.col(3) - p)/0.001; //so calculate it here [only for position so far]

	// update current position & orientation
	p = pose.col(3);
	R = pose.submat(0, 0, 2, 2);

	//update current orientation
	//Q = Orientation::rot2quat(R);
	// update orientation error
	arma::vec te=(Q.t()*Qd); if (te(0)<0.0) Qd=-Qd; //avoid incontinuity
	//quatDiff = Orientation::quatDifference(Q, Qd);
	e_o = -2.0*quatDiff.rows(1, 3);

	//read Jacobian
	robot->getJacobian(J);
	Jt = J.submat(0, 0, 2, 6); //update translat.  Jacobian
	Jr = J.submat(3, 0, 5, 6); //update rotational Jacobian

	robot->getJointPosition(q); //read joint position
	robot->getJointVelocity(qdot); //read joint velocity

	//read forces
	robot->getExternalWrench(forces);


	if (record_current_pose)
	{
		record_current_pose = false; // reset flag

		Poses.push_back(pose);
		Joints.push_back(q);

		std::cout << "Recorded current pose!\n";
		// record_current_pose = false;
		//
		// std::string time_stamp = as64::getTimeStamp();
		//
		// std::string path = this->DIR + "pose_" + this->filename + "_" + time_stamp; // + ".dat";
		//
		// std::ofstream out;
		// if (this->binary) out.open(path, std::ios::binary);
		// else out.open(path);
		//
		// if (!out) throw std::ios_base::failure(std::string("Couldn't create file \"") + path + "\"");
		//
		// as64_::io_::write_mat(pose, out, binary, 6);
		// as64_::io_::write_mat(q, out, binary, 6);
		//
		// out.close();
	}


}


void RobotGravCompController::JointImpedance()
{
	//Ott's (3.18 page 38, Ott) cartesian impedance controller without inertia reshaping
	u = -Jt.t()*( K_imp.submat(0,0,2,2)*(p-p_ref) + D_imp.submat(0,0,2,2)*(xdot.subvec(0,2)-v_ref))
		+Jr.t()*( K_imp.submat(3,3,5,5)*e_o - D_imp.submat(3,3,5,5)*xdot.subvec(3,5) ); //the last term is friction compensation

  u.zeros(N_JOINTS);

	robot->setJointTorque(u);
}

void RobotGravCompController::CartImpedance()
{
	// if (!path->loop_closed) { //loop is still open
	// 	pose_ref.submat(0,3,2,3)  = p; //mirror position values to avoid CP limit
	// 	p_ref = p; //mirror p_ref for the first control loop when we close the task cycle
	//
	// 	//Extra damping in free-space when K_imp=0
	// 	u.subvec(0,2) = D_imp.submat(0,0,2,2)*xdot.subvec(0,2);
	// }
	// else {
	//
	// 	pose_ref.submat(0,3,2,3)  = p_ref;
	// 	robot->setCartStiffness(diagvec(K_imp));
	//
	// 	u.subvec(0,2) = D_imp.submat(0,0,2,2)*xdot.subvec(0,2) - getConstraints(); //enable constraint forces
	// }
	// robot->setTaskPose(pose_ref);
	//
	// // It seems that setWrench() applies XY in the wrong way. This is a crappy way to fix it temporarily
	// double tmp = u(0);
	// u(0) = u(1);
	// u(1) = tmp;
	//
	// robot->setWrench(u.subvec(0,5)); //apply extra damping and constraint forces
}

void RobotGravCompController::update()
{
	// if (reset_flag) { //reset system after a key has been pressed
	// 	reset();
	// 	reset_flag = false;
	// }

	time += robot->cycle;

	//Select which method to use. They should have the same result. "Joint" is preferred because it is thoroughly tested.
	if(!control_method.compare("Cartesian")) {
		CartImpedance();
	}
	else if (!control_method.compare("Joint"))
		JointImpedance();

}

void RobotGravCompController::command()
{
	// //Only used to stop the robot by the user to avoid dangerous movement
	// if (user_stop1) {
	// 	if(!control_method.compare("Joint")) {
	// 		u.fill(0.0);
	// 		robot->setJointTorque(u);
	// 	}
	// 	user_stop2=true; //send terminate
	// }
}

/// Keyboard control in a thread
void RobotGravCompController::keyboardControl()
{
	int key=0;
	while (key!=10) // Enter
	{
		key = as64_::io_::getch();

		//std::cout << "Pressed " << (char)key  << "\n";

		switch (key)
		{
			// case 32: //Spacebar
			case 'z':
				reset_to_init_config = true;
				break;
			case 'r':
				record_path = !record_path;
				break;
			case 'o':
				record_current_pose = true;
				break;
			case 's':
				stop_program = true;
				break;
		}

	}
}

bool RobotGravCompController::run()
{
	std::thread waitInput(&RobotGravCompController::keyboardControl, this);

	//move to initial pose
	if (set_init_config)
	{
		robot->setMode(arl::robot::Mode::POSITION_CONTROL); //position mode
		std::cout << "Moving to start configuration..." << std::endl;
		robot->setJointTrajectory(q_init, 6.0);
	}


	//Generate trajectory to follow
	robot->getTaskPose(pose);
	p = pose.col(3);
	p_ref = p; //mirror current position
	pose_ref = pose;

	Qd.zeros(4);
	//Qd = Orientation::rot2quat(pose.submat(0, 0, 2, 2)); //[keep Q desired on the initial one]

	//Save starting configuration
	robot->getJointPosition(q_ref);

	//Switch to impedance control
	if(!control_method.compare("Cartesian"))
	{
		robot->setMode(arl::robot::Mode::IMPEDANCE_CONTROL);
	}
	else if (!control_method.compare("Joint"))
	{
		robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
	}

	setImpedanceParams();

	// a mutex in robot should be locked to ensure no other controller is running
	// on this robot
	while (!stop() && robot->isOk())
	{
		measure();
		update();
		command();
		robot->waitNextCycle();
		ros::spinOnce();
	}

	//stop when finished
	robot->stop();

	this->write_recorded_poses();

	std::cout << "Press [ENTER] to shutdown controller..." << std::endl;
	waitInput.join();

	return true;
}

void RobotGravCompController::write_recorded_poses()
{
	std::ofstream out(DIR + filename);

	int n_poses = Poses.size();

	out << n_poses << "\n";

	for (int k=0; k<n_poses; k++)
	{
		for (int i=0;i<Joints[k].size(); i++) out << Joints[k](i) << " ";
		out << "\n";
		for (int i=0;i<3;i++){
			for (int j=0;j<4;j++) out << Poses[k](i,j) << " ";
			out << "\n";
		}
		out << "\n";
	}

	out.close();
}


bool RobotGravCompController::stop()
{
	return stop_program;
}
