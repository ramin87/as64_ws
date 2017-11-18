/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <handover_controller.h>

Eigen::Matrix4d HandoverController::robotArm_fkine() const
{
	Eigen::Matrix4d T_robot_endEffector = Eigen::Matrix4d::Identity();

	Eigen::MatrixXd T_robot_endEffector2;

	//arl::utils::kdlToEigen(robot_->getPose(), &T_robot_endEffector);
	robot_->getTaskPose(T_robot_endEffector2,ROBOT_ARM_INDEX);
	//std::cout << "T_robot_endEffector2:\n" << T_robot_endEffector2 << "\n";

	T_robot_endEffector.block(0,0,3,4) = T_robot_endEffector2;

	return T_robot_endEffector;
}

Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> HandoverController::robotArm_jacob() const
{
	Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> Jrobot;
	Eigen::MatrixXd Jrobot2;

	//arl::utils::kdlToEigen(robot_->getEEJacobian(), &JendEffector);
	robot_->getJacobian(Jrobot2, ROBOT_ARM_INDEX);

	Jrobot = Jrobot2;

	return Jrobot;
}

void HandoverController::goToStartPose(double t)
{
	arma::vec qT(7), q_current(7);

	for (int i=0;i<7;i++){
		qT(i) = q0(i);
		q_current(i) = q(i);
	}

	goToJointsTrajectory(qT, q_current, t);
}

void HandoverController::goToJointsTrajectory(arma::vec qT, arma::vec &q0, double totalTime)
{

	arma::vec temp = (arma::abs(qT-q0));
	double max_q_e = temp.max() * 180 / M_PI;
	totalTime = 25*max_q_e/360;

	std::cout << "max_q_e = " << max_q_e << "\n";
	std::cout << "totalTime = " << totalTime << "\n";

	//int dummy;
	//std::cin >> dummy;
	if (totalTime < 4.5) totalTime = 4.5;

    // inital joint position values
    // arma::vec q0 = getJointPositions();
    arma::vec qref = q0;

    //initalize time
    double t = 0;

    // the main while
    while (t < totalTime) {

		//std::cout << "Time: " << t  << " s \n";
		//std::cout << "Ts: " << Ts  << " s \n";

        //ocmpute time now
        t += Ts;

        // update trajectory
        qref = (get5thOrder(t, q0, qT, totalTime)).col(0);

        // set joint positions
        for (int i=0;i<7;i++) q_kdl(i) = qref(i);
        robot_->setJointPosition(q_kdl,ROBOT_ARM_INDEX);
		robot_->waitNextCycle();
    }

}
/*
Eigen::MatrixXd HandoverController::get5thOrder(double t, const Eigen::VectorXd &p0, const Eigen::VectorXd &pT, double totalTime)
{
    Eigen::MatrixXd retTemp = Eigen::MatrixXd::Zero();
    arma::zeros<arma::mat>(p0.n_rows, 3);

    if (t < 0) {
        // before start
        retTemp.col(0) = p0;
    } else if (t > totalTime) {
        // after the end
        retTemp.col(0) = pT;
    } else {
        // somewhere betweeen ...
        //position
        retTemp.col(0) = p0 + (pT - p0) * (10 * pow(t / totalTime, 3) - 15 * pow(t / totalTime, 4) + 6 * pow(t / totalTime, 5));
        //vecolity
        retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) - 60 * pow(t, 3) / pow(totalTime, 4) + 30 * pow(t, 4) / pow(totalTime, 5));
        // acceleration
        retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) - 180 * pow(t, 2) / pow(totalTime, 4) + 120 * pow(t, 3) / pow(totalTime, 5));
    }

    // return vector
    return retTemp;

}*/

arma::mat HandoverController::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
    arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

    if (t < 0) {
        // before start
        retTemp.col(0) = p0;
    } else if (t > totalTime) {
        // after the end
        retTemp.col(0) = pT;
    } else {
        // somewhere betweeen ...
        //position
        retTemp.col(0) = p0 + (pT - p0) * (10 * pow(t / totalTime, 3) - 15 * pow(t / totalTime, 4) + 6 * pow(t / totalTime, 5));
        //vecolity
        retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) - 60 * pow(t, 3) / pow(totalTime, 4) + 30 * pow(t, 4) / pow(totalTime, 5));
        // acceleration
        retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) - 180 * pow(t, 2) / pow(totalTime, 4) + 120 * pow(t, 3) / pow(totalTime, 5));
    }

    // return vector
    return retTemp;

}

void HandoverController::listenCameraTargetTransformCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	if (cmd_args.p_ch==2 || cmd_args.p_ch==5 || cmd_args.p_ch==22){

		received_target = true;

		int k=0;
		for (int i=0;i<4;i++){
			for (int j=0;j<4;j++) T_cam_objTarget(i,j) = msg->data[k++];
		}

		std::cout << "**********   Received transform!!!   ************\n";
	}

}

double HandoverController::get_pose_error(const Eigen::Matrix4d &T) const
{
	Eigen::Vector3d pos_err = T.block(0,3,3,1);

	Eigen::Vector4d quat = as64::rotm2quat(T.block(0,0,3,3));
	Eigen::Vector3d orient_err = quat.segment(1,3);
	double r = cmd_args.POS_ERR_COEFF * pos_err.squaredNorm() + (1-cmd_args.POS_ERR_COEFF) * orient_err.squaredNorm();

	return r;
}

void HandoverController::calc_DS_input(const Eigen::Matrix4d &T_objTarget_obj, Eigen::Matrix<double,6,1> *X_in) const
{
// 	Eigen::Vector3d pos_err = T_objTarget_obj.block(0,3,3,1);
// 	X_in->segment(0,3) = pos_err;
// 	Eigen::Vector4d quat_err = rotm2quat(T_objTarget_obj.block(0,0,3,3));
// 	Eigen::Vector3d orient_err = quat_err.segment(1,3);
// 	X_in->segment(3,3) = orient_err;

	X_in->segment(0,3) = T_objTarget_obj.block(0,3,3,1);
	X_in->segment(3,3) = as64::rotm2quat(T_objTarget_obj.block(0,0,3,3)).segment(1,3);
}

HandoverController::HandoverController(std::shared_ptr<arl::robot::Robot> robot)
{
	robot_ = robot;
	start = true;
	Ts = robot_->cycle;

	zero_vel = false;
	move_back = false;
	start_move_back = false;

	out_err.open(cmd_args.output_path + "err_log.txt",std::ios::out);
	if (!out_err){
		std::cerr << "Could create error_log.txt...\n";
		start = false;
	}

	global_t = 0;
	out_log.open(cmd_args.output_path + "log.txt",std::ios::out);
	if (!out_log){
		std::cerr << "Couldn't create log.txt...\n";
		exit(-1);
	}

	cmd_args.parse_cmd_args();
	cmd_args.print(out_log);
	cmd_args.print();


	// *******************
	// *****  JLAV  ******
	// *******************
	arma::vec qmin({-170, -120, -170, -120, -170, -120, -170});
	arma::vec qmax = -qmin;
	jlav.init(qmin,qmax);
	jlav.changeGains(cmd_args.JLAV_gain);

	// *******************
	// *****  SVF  ******
	// *******************
	svf.set_sigma_min(cmd_args.SVF_sigma_min);
	svf.set_shape_factor(cmd_args.SVF_shape_factor);


	// ************************
	// ***  initialize DS  ***
	// ************************
	ds = boost::shared_ptr<DS::DS_abstract>(new DS::DS_GMR());
	std::cout << "==> Created DS!\n";
	if (!ds->load_ds_params(cmd_args.DS_params_filename.c_str())){
		std::cerr << "Couldn't load \"" << cmd_args.DS_params_filename << "\"...\n";
		start = false;
		return;
	}
	std::cout << "Loaded params successfully!\n";

	// ****************************************
	// ***  subscribe to TargetFrame topic  ***
	// ****************************************
	node_handle_ = ros::NodeHandle("~");

	sub_camTargTrans = n.subscribe(cmd_args.cam_target_transform_topic, 1, &HandoverController::listenCameraTargetTransformCallback, this);

	std::cout << "Subscrivbed to target-frame topic: \"" << cmd_args.cam_target_transform_topic << "\"\n";

	// *****************************************
	// ***  load the robot_camera transform  ***
	// *****************************************
	/*if (!load_robot_camera_transform(T_robot_cam)){
		std::cerr << "ERROR: Failed to load the robot-camera transform...\n";
		exit(-1);
	}
	*/
	T_robot_cam = Eigen::Matrix4d::Identity();
	T_cam_robot = T_robot_cam.inverse();

	// *****************************************
	// ***  get the camera_object transform  ***
	// *****************************************
	T_endEffector_obj = Eigen::Matrix4d::Identity();

	std::ifstream in_T_endEffector_obj(cmd_args.T_endEffector_obj_filename.c_str());
	if (!in_T_endEffector_obj){
		std::cerr << "Couldn't open \"" << cmd_args.T_endEffector_obj_filename << "\"..\n";
		start = false;
		return;
	}
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++) in_T_endEffector_obj >> T_endEffector_obj(i,j);
	}

	T_obj_endEffector = T_endEffector_obj.inverse();

	Eigen::Matrix3d R_endEffector_obj = T_endEffector_obj.block(0,0,3,3);
	Eigen::Vector3d p_endEffector_obj = T_endEffector_obj.block(0,3,3,1);
	Twist_endEffector_obj << R_endEffector_obj, as64::vec2ssMat(p_endEffector_obj)*R_endEffector_obj,
	Eigen::Matrix3d::Zero(), R_endEffector_obj;

	if (cmd_args.LOG_DATA) logData.Ts = Ts;

	if (cmd_args.p_ch == 0){
		std::cout << "Pose:\n";
		std::cout << robotArm_fkine() << std::endl;
		robot_-> getJointPosition(q_kdl,ROBOT_ARM_INDEX);
		std::cout << "Joints:\n";
		for (int i=0;i<7;i++) std::cout << q_kdl(i) << " ";
		std::cout << std::endl;
		start = false;
		return;
	}
	else if (cmd_args.p_ch == 5){
		std::cout << " ========  CALIBRATION MODE: T_robot_cam ==========\n";
		T_robot_endEffector = robotArm_fkine();
		Eigen::Matrix4d T_endEffector_objTarget = Eigen::Matrix4d::Identity();
		received_target = false;
		while (!received_target) ros::spinOnce();
		T_robot_cam = T_robot_endEffector * T_endEffector_objTarget * T_cam_objTarget.inverse();
		T_cam_robot = T_robot_cam.inverse();


		std::ofstream out_robot_cam_trans(cmd_args.T_robot_cam_filename.c_str());
		if (!out_robot_cam_trans){
			std::cerr << "Couldn't open \"" << cmd_args.T_robot_cam_filename << "\"...\n";
			exit(-1);
		}
		for (int i=0;i<4;i++){
			for (int j=0;j<4;j++) out_robot_cam_trans << T_robot_cam(i,j) << " ";
			out_robot_cam_trans << "\n";
		}

		out_robot_cam_trans.close();


		std::cout << "T_robot_cam successfully writen to output file!!!\n";
		start = false;
		return;

	}
	else if (cmd_args.p_ch >= 1){
		std::ifstream in_init_pose(cmd_args.init_pose_filename.c_str(), std::ios::in);
		if (!in_init_pose){
			std::cerr << "Error opening \"" << cmd_args.init_pose_filename << "\"...\n";
			start = false;
			return;
		}
		arma::vec q_start(7), qT(7);
		robot_-> getJointPosition(q_kdl,ROBOT_ARM_INDEX);
		for (int i=0;i<7;i++) q_start(i) = q_kdl(i);
		for (int i=0;i<7;i++) in_init_pose >> qT(i);
		in_init_pose.close();
		std::cout << "Setting initial pose...";

		if (cmd_args.p_ch == 22) qT = q_start;

		goToJointsTrajectory(qT, q_start, 6);

		std::cout << "...[DONE]\n";

		for (int i=0;i<7;i++) this->q0(i) = qT(i);
		q = q0;

		if (cmd_args.p_ch == 1){
			std::ifstream in_target_pose(cmd_args.target_pose_filename.c_str(), std::ios::in);
			if (!in_target_pose){
				std::cerr << "Error opening \"" << cmd_args.target_pose_filename << "\"...\n";
				start = false;
				return;
			}

			T_cam_objTarget = Eigen::Matrix4d::Identity();
			for (int i=0;i<3;i++){
				for (int j=0;j<4;j++) in_target_pose >> T_cam_objTarget(i,j);
			}
			in_target_pose.close();

		}else if (cmd_args.p_ch==2 || cmd_args.p_ch==22){

			std::cout << "Loading T_robot_cam transform...\n";
			std::ifstream in_T_robot_cam(cmd_args.T_robot_cam_filename.c_str(), std::ios::in);
			if (!in_T_robot_cam){
				std::cerr << "Error opening \"" << cmd_args.T_robot_cam_filename << "\"...\n";
				start = false;
				return;
			}
			T_robot_cam = Eigen::Matrix4d::Identity();
			for (int i=0;i<3;i++){
				for (int j=0;j<4;j++) in_T_robot_cam >> T_robot_cam(i,j);
			}
			in_T_robot_cam.close();
			T_cam_robot = T_robot_cam.inverse();
			std::cout << "Loaded T_robot_cam transform!\n";

			std::cout << "Waiting to receive target pose...\n";
			received_target = false;
			while (!received_target) ros::spinOnce();
			std::cout << "Received target pose!!!\n";
		}

		T_robot_endEffector = robotArm_fkine();

		T_cam_obj = T_cam_robot * T_robot_endEffector * T_endEffector_obj;
		T_objTarget_obj = T_cam_objTarget.inverse() * T_cam_obj;

		//max_r = T_objTarget_obj.block(0,3,3,1).squaredNorm();
	}
	else{
		start = false;
		return;
	}

	if (!(robot_->isOk()) || !start){
	    std::cout << "Robot is not okay...\n";
	    start = false;
	    return;
	}

	// ********************************************
	// ******    Init Weight Estimator    *********
	// ********************************************

	qd = q0;
	for (int i=0;i<7;i++) q_kdl(i) = q0(i);

	KDL::Wrench wrench_ee;
	Fee_z_init = 0;
	const int nFee = 30;
	for (int i=0;i<nFee;i++){
		robot_->waitNextCycle();
		robot_->getExternalWrench(wrench_ee, ROBOT_ARM_INDEX);
		robot_->setJointPosition(q_kdl, ROBOT_ARM_INDEX);
		Fee_z_init += -wrench_ee(2);
	}
	Fee_z_init /= nFee;
	Fee_z = Fee_z_init = Fee_z_init;
	std::cout << "Fee_z_init = " << Fee_z_init << "\n";

	Fee_z_max = 0;

	obj_weight_filter.init(cmd_args.WEIGHT_EST_FILT_N, 0);

	robot_-> getJointPosition(q_kdl, ROBOT_ARM_INDEX);
	for (int i=0;i<7;i++) q_prev(i) = q_kdl(i);
	q = q_prev;
	q_dot = (q - q_prev) / Ts;

	average_cycle_time = 0;
	cycles_count = 0;
	max_cycle_time = 0;
	min_cycle_time = 100000000000;

	debug_iter = 0;


	// ******************************************
	// ***  publish to Kuka_to_Barrett topic  ***
	// ******************************************
	n_pub = node_handle_.advertise<std_msgs::Float64MultiArray>(cmd_args.Kuka_to_Barrett_topic, 1);
	msg.data.resize(2);

	bHandAction = STOP_HAND;
	w_est = Fee_z - Fee_z_init;
	msg.data[0] = (int)bHandAction;
	msg.data[1] = w_est;

	out_log << "Fee_z_init = " << Fee_z_init << "\n";

	V_endEffector = Vd_endEffector = Eigen::Matrix<double,6,1>::Zero();
}


void HandoverController::execute()
{
	while (ros::ok() && this->start){
		ros::spinOnce();

		timer.start();

		global_t += Ts;

		out_log << "================================================\n";
		out_log << "t = " << global_t << "\n";


		ros::spinOnce();

		if (!(robot_->isOk()) || !start){
		    std::cout << "Robot is not okay...\n";
		    start = false;
		    return;
		}

		// ***************************************************
		// ***************************************************

		T_robot_endEffector = robotArm_fkine();
		J_robot = robotArm_jacob();
		transform_Jacob(T_robot_endEffector, J_robot, J_endEffector);

		q_dot = (q - q_prev) / Ts;

		if (cmd_args.V_Vd_equal) V_endEffector = Vd_endEffector;
		else V_endEffector = J_endEffector * q_dot;

		T_cam_obj = T_cam_robot * T_robot_endEffector * T_endEffector_obj;

		T_objTarget_obj = T_cam_objTarget.inverse() * T_cam_obj;

		Eigen::Matrix4d T_robot_objTarget = T_robot_cam * T_cam_obj * T_objTarget_obj.inverse();

		out_log << "q = " << q.transpose() << "\n";
		out_log << "q_dot = " << q_dot.transpose() << "\n";
		out_log << "V_endEffector = " << V_endEffector.transpose() << "\n";
		out_log << "T_cam_robot = \n" << T_cam_robot << "\n";
		out_log << "T_robot_endEffector = \n" << T_robot_endEffector << "\n";
		out_log << "T_endEffector_obj = \n" << T_endEffector_obj << "\n";
		out_log << "T_objTarget_obj = \n" << T_objTarget_obj << "\n";
		out_log << "T_cam_obj = \n" << T_cam_obj << "\n";
		out_log << "T_robot_objTarget = \n" << T_robot_objTarget << "\n";

		double r = get_pose_error(T_objTarget_obj);

		double r_p = T_objTarget_obj.block(0,3,3,1).squaredNorm();
		if (r_p > cmd_args.START_MOVE_THRES){
			zero_vel = true;
		}else if (r_p < cmd_args.START_MOVE_THRES && !move_back){
			zero_vel = false;
		}

		out_log << "r = " << r << "\n";
		out_log << "r_p = " << r_p << "\n";

		// ********* find desired endEffector velocity and apply mirroring if necessary  *************

		Eigen::Matrix4d T_objTarget_obj2 = T_objTarget_obj;

		bool mirror_occured = false;

		double x_pos = T_objTarget_obj(0,3);
		if (!zero_vel)
		{
			double robot_x_pos;
			if (cmd_args.USE_MIRRORING)
			{
				robot_x_pos = T_objTarget_obj2(0,3);
				if (robot_x_pos > cmd_args.MIRROR_THRES)
				{
			  	mirror_Htrans_wrt_yz_plane(&T_objTarget_obj2);
			  	mirror_occured = true;
				}
	    }

			Eigen::Matrix<double, 6, 1> X_in;
			calc_DS_input(T_objTarget_obj2, &X_in);
			Eigen::MatrixXd X_out;
			ds->get_ds_output(X_in, &X_out);
			V_objTarget_obj = X_out.block(0,0,6,1);

	    if (mirror_occured)
			{
				mirror_Velocity_wrt_yz_plane(&V_objTarget_obj);

				out_log << "mirror_occured = " << mirror_occured << "\n";

				if (robot_x_pos < cmd_args.MIRROR_SMOOTH_THRES)
				{
			    Eigen::Matrix<double, 6, 1> V2 = V_objTarget_obj;
			    calc_DS_input(T_objTarget_obj, &X_in);
			    ds->get_ds_output(X_in, &X_out);
			    Eigen::Matrix<double, 6, 1> V1 = X_out.block(0,0,6,1);

			    V_objTarget_obj = (V2-V1)*robot_x_pos/cmd_args.MIRROR_SMOOTH_THRES + V1;
				}

				// Eigen::Matrix<double,6,1> v_gains;
				// v_gains << 1, 1, 0.6, 1, 1, 1;
				// V_objTarget_obj = - (v_gains.array() * X_in.array()).matrix();
			}

	    // express the velocity of the object in the object frame
	    Eigen::Matrix<double, 6, 1> V_obj = Eigen::Matrix<double, 6, 1>::Zero();
	    V_obj.segment(0,3) = T_objTarget_obj.block(0,0,3,3).transpose() * V_objTarget_obj.segment(0,3);
	    V_obj.segment(3,3) = T_objTarget_obj.block(0,0,3,3).transpose() * V_objTarget_obj.segment(3,3);

	    Vd_endEffector = Twist_endEffector_obj * V_obj;

	    if (cmd_args.NORM_VEL)
			{
				Vd_endEffector = Vd_endEffector/Vd_endEffector.norm();
				double r2 = (r>1)?1:r;
				v_scale = cmd_args.V_SCALE_C*r2 / (std::pow(1-r2,cmd_args.V_SCALE_EXP)+cmd_args.V_SCALE_C*r2);
				Vd_endEffector *= cmd_args.VEL_SCALE*v_scale;
			}
			else
			{
				Vd_endEffector = cmd_args.VEL_SCALE*Vd_endEffector;
				v_scale = 1;
	    }

	    Eigen::Matrix<double, 6, 1> eV = (Vd_endEffector - V_endEffector);
	    //Eigen::Matrix<double, 6, 1> eV_dot = -cmd_args.ks * eV;
	    //eV += eV_dot*Ts;
	    //Vd_endEffector = Vd_endEffector - eV;
	    Vd_endEffector = cmd_args.ks*Vd_endEffector + (1-cmd_args.ks)*V_endEffector;
		}
		else
		{
			Vd_endEffector = V_endEffector*cmd_args.kV; //+ Ts*(-cmd_args.kV*V_endEffector);
		}

		if (Vd_endEffector.norm() > cmd_args.HIGH_VEL_TOL_STOP)
		{
			out_err << "========================================\n";
			out_err << "*****  WARNING  ******\n";
			out_err << "High velocity:\nVd_endEffector = \n" << Vd_endEffector << "\n";
			out_err << "Vd_endEffector.norm() = " << Vd_endEffector.norm();
			out_err << "Zeroing velocity to avoid accident...\n";
			out_err << "========================================\n";
			zero_vel = true;
			return;
		}

		Eigen::Matrix<double, JOINT_SIZE,CART_DOF_SIZE> J_endEffector_inv;
		if (cmd_args.SVF_enable) J_endEffector_inv = svf.inv(J_endEffector);
		else J_endEffector_inv = as64::inv(J_endEffector);

		qd_dot = J_endEffector_inv * Vd_endEffector;

		if (cmd_args.JLAV_enable)
		{
		  arma::vec q_temp(&(q(0)), JOINT_SIZE, false, true);
		  arma::vec qdot_jval_temp = jlav.getControlSignal(q_temp);
		  Eigen::Map<Eigen::Matrix<double,JOINT_SIZE,1>> qdot_jval(qdot_jval_temp.memptr());
		  //qd_dot += (Eigen::Matrix<double,JOINT_SIZE,JOINT_SIZE>::Identity() - J_endEffector_inv*J_endEffector) * qdot_jval;
		  qd_dot += qdot_jval;
		}

	// 	//qd_dot = J_endEffector.colPivHouseholderQr().solve(Vd_endEffector);

		q_prev = q;
		//q += q_dot * Ts;
		q += qd_dot * Ts;

		for (int i=0;i<7;i++) q_kdl(i) = q(i);
		robot_->setJointPosition(q_kdl,ROBOT_ARM_INDEX);

		robot_->waitNextCycle();

		if (r < cmd_args.TOL_STOP)
		{
			zero_vel = true;
		}else if (!move_back){
			zero_vel = false;
		}

		KDL::Wrench wrench_ee; // = robot_->getExternalCartForces();
		robot_->getExternalWrench(wrench_ee, ROBOT_ARM_INDEX);
		Fee_z = -wrench_ee(2);
		w_est = Fee_z - Fee_z_init;

		obj_weight_filter.update(w_est);
		w_est_filt = obj_weight_filter.get_filtered_output();

		if (w_est_filt > Fee_z_max) Fee_z_max = w_est_filt;

		if (w_est_filt > cmd_args.OLT_THRES)
		{
			//std::cout << "weight_estimate > cmd_args.OLT_THRES = " << Fee_z << " > " << cmd_args.OLT_THRES << "\n";
			zero_vel = true;
			move_back = true;

			bHandAction = HandoverController::OPEN_HAND;
			msg.data[0] = (int)bHandAction;
			msg.data[1] = w_est_filt;
			n_pub.publish(msg);

			std::cout << "Kuka: I published!!!\n";
			//bHandAction = HandoverController::OPEN_HAND;
		}

		start_move_back = move_back && V_endEffector.squaredNorm() < 0.001;
		if (start_move_back)
		{
			std::cout << "V_endEffector.squaredNorm() = " << V_endEffector.squaredNorm() << "\n";
			start=false;
			return;
		}

		if (cmd_args.LOG_DATA){
			logData.push(q, q_dot, V_endEffector, V_objTarget_obj,
				T_cam_obj, T_cam_objTarget, w_est_filt,
				 v_scale, mirror_occured);
		}

		double elapsed_time = 1000*timer.stop();
		//std::cout << "elapsed_time = " << elapsed_time*1000 << " ms\n";

		average_cycle_time += elapsed_time;
		cycles_count++;

		if (max_cycle_time < elapsed_time) max_cycle_time = elapsed_time;
		if (min_cycle_time > elapsed_time) min_cycle_time = elapsed_time;
	}
}

void HandoverController::finalize()
{
	std::cout << "[HANDOVER_CONTROLLER]: Entered finalize...\n";

	if (robot_->isOk()){
	  std::cout << "[HANDOVER_CONTROLLER]: Go to start pose...\n";
	  goToStartPose(6);
	}

	bHandAction = HandoverController::TERMINATE_HAND;
	msg.data[0] = (int)bHandAction;
	msg.data[1] = w_est;
	n_pub.publish(msg);
	//bHandThread.join();



	average_cycle_time /= cycles_count;
	std::cout << "Elapsed cycle time:\n";
	std::cout << "average = " << average_cycle_time << "\n";
	std::cout << "max_cycle_time = " << max_cycle_time << "\n";
	std::cout << "min_cycle_time = " << min_cycle_time << "\n";


	std::cout << "Fee_z_max = " << Fee_z_max << "\n";

	//shutdown_bhand();

	std::cout << "Saving logged data...\n";
	if (cmd_args.LOG_DATA) logData.save(cmd_args.output_path + "exp_results.txt");

	out_err.close();
	out_log.close();

}
