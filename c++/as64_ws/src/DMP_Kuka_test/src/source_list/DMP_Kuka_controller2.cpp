/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <DMP_Kuka_controller.h>

Eigen::Matrix4d DMP_Kuka_controller::robotArm_fkine() const
{
	Eigen::Matrix4d T_robot_endEffector = Eigen::Matrix4d::Identity();

	Eigen::MatrixXd T_robot_endEffector2;

	//arl::utils::kdlToEigen(robot_->getPose(), &T_robot_endEffector);
	robot_->getTaskPose(T_robot_endEffector2,ROBOT_ARM_INDEX);
	//std::cout << "T_robot_endEffector2:\n" << T_robot_endEffector2 << "\n";

	T_robot_endEffector.block(0,0,3,4) = T_robot_endEffector2;

	return T_robot_endEffector;
}

Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> DMP_Kuka_controller::robotArm_jacob() const
{
	Eigen::Matrix<double, CART_DOF_SIZE, JOINT_SIZE> Jrobot;
	Eigen::MatrixXd Jrobot2;

	//arl::utils::kdlToEigen(robot_->getEEJacobian(), &JendEffector);
	robot_->getJacobian(Jrobot2, ROBOT_ARM_INDEX);

	Jrobot = Jrobot2;

	return Jrobot;
}


DMP_Kuka_controller::DMP_Kuka_controller(std::shared_ptr<arl::robot::Robot> robot)
{
	robot_ = robot;
	start = true;
	Ts = robot_->cycle;

	// *******************
	// *****  JLAV  ******
	// *******************
	arma::vec qmin({-170, -120, -170, -120, -170, -120, -170});
	arma::vec qmax = -qmin;
	jlav.init(qmin,qmax);
	jlav.setGains(cmd_args.JLAV_gain);

	// *******************
	// *****  SVF  ******
	// *******************
	svf.set_sigma_min(cmd_args.SVF_sigma_min);
	svf.set_shape_factor(cmd_args.SVF_shape_factor);


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

	robot_-> getJointPosition(q_kdl, ROBOT_ARM_INDEX);
	for (int i=0;i<7;i++) q_prev(i) = q_kdl(i);
	q = q_prev;
	q_dot = (q - q_prev) / Ts;

	V_endEffector = Vd_endEffector = Eigen::Matrix<double,6,1>::Zero();
}


void DMP_Kuka_controller::execute()
{
	while (ros::ok() && this->start){
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
		//transform_Jacob(T_robot_endEffector, J_robot, J_endEffector);

		q_dot = (q - q_prev) / Ts;

		// Eigen::Matrix<double, JOINT_SIZE,CART_DOF_SIZE> J_endEffector_inv;
		// if (cmd_args.SVF_enable) J_endEffector_inv = svf.inv(J_endEffector);
		// else J_endEffector_inv = as64_::inv(J_endEffector);
    //
		// qd_dot = J_endEffector_inv * Vd_endEffector;
    //
		// if (cmd_args.JLAV_enable)
		// {
		//   arma::vec q_temp(&(q(0)), JOINT_SIZE, false, true);
		//   arma::vec qdot_jval_temp = jlav.getControlSignal(q_temp);
		//   Eigen::Map<Eigen::Matrix<double,JOINT_SIZE,1>> qdot_jval(qdot_jval_temp.memptr());
		//   //qd_dot += (Eigen::Matrix<double,JOINT_SIZE,JOINT_SIZE>::Identity() - J_endEffector_inv*J_endEffector) * qdot_jval;
		//   qd_dot += qdot_jval;
		// }

	// 	//qd_dot = J_endEffector.colPivHouseholderQr().solve(Vd_endEffector);

		q_prev = q;
		//q += q_dot * Ts;
		q += qd_dot * Ts;

		for (int i=0;i<7;i++) q_kdl(i) = q(i);
		robot_->setJointPosition(q_kdl,ROBOT_ARM_INDEX);

		robot_->waitNextCycle();


		KDL::Wrench wrench_ee; // = robot_->getExternalCartForces();
		robot_->getExternalWrench(wrench_ee, ROBOT_ARM_INDEX);
		//Fee_z = -wrench_ee(2);
	}
}

void DMP_Kuka_controller::update()
{

}

bool DMP_Kuka_controller::run()
{
	return true;
}

void DMP_Kuka_controller::finalize()
{
	std::cout << "[HANDOVER_CONTROLLER]: Entered finalize...\n";
}
