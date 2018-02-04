#include <DMP_Kuka_controller.h>

DMP_Kuka_controller::DMP_Kuka_controller(std::shared_ptr<arl::robot::Robot> robot)
{
	robot_ = robot;
	start = true;
	Ts = robot_->cycle;

	std::cout << as64_::io_::bold << as64_::io_::green << "Reading params from yaml file..." << as64_::io_::reset << "\n";
  cmd_args.parse_cmd_args();
  cmd_args.print();

	keyboard_ctrl_thread.reset(new std::thread(&DMP_Kuka_controller::keyboard_ctrl_function, this));
}

void DMP_Kuka_controller::init_controller()
{
	log_on = false;
  run_dmp = false;
  train_dmp = false;
  goto_start = false;
  stop_robot = false;
	start_demo = false;
  end_demo = false;
	clear_restart_demo = false;
	save_restart_demo = false;

	demo_save_counter = 0;

	robot_->getJointPosition(q0_robot, ROBOT_ARM_INDEX);
	q_robot = q_prev_robot = q0_robot;
	dq_robot = (q_robot - q_prev_robot) / Ts;

	dY_robot_prev = arma::vec().zeros(3);
	v_rot_robot_prev = arma::vec().zeros(3);

	t = 0;
}

void DMP_Kuka_controller::keyboard_ctrl_function()
{
  // run_dmp = false;
  // train_dmp = false;
  // goto_start = false;

	int key=0;
	while (!stop_robot) // Enter
	{
		key = as64_::io_::getch();

		//std::cout << "Pressed " << (char)key  << "\n";

		switch (key)
		{
			// case 32: //Spacebar
			case 'c':
				clear_restart_demo = true;
				break;
			case 'r':
				save_restart_demo = true;
				break;
			case 'l':
				log_on = true;
				break;
			case 's':
				stop_robot = true;
				break;
		}

	}
}

void DMP_Kuka_controller::record_demo()
{
	run_dmp = false; // stop running the DMP
	while (start_demo == false)
	{
		command();
	}

	update();
	q0_robot = q_robot;
	while (end_demo == false)
	{
		update();
		log_demo_step();
	}

	start_demo = false;
	end_demo = false;
	train_dmp = true; // the DMP must be trained now with the demo data

  n_data = Time_demo.size();
  Dp = dYd_data.n_rows;
  Do = v_rot_d_data.n_rows;
  D = Dp+Do;
  tau = Time_demo(n_data-1);

	goto_start_pose();
}

void DMP_Kuka_controller::goto_start_pose()
{
	robot_->setJointTrajectory(q0_robot, 6.0, ROBOT_ARM_INDEX);
	goto_start = false;
}


void DMP_Kuka_controller::train_DMP()
{
  // as64_::param_::ParamList trainParamList;
  // trainParamList.setParam("lambda", cmd_args.lambda);
  // trainParamList.setParam("P_cov", cmd_args.P_cov);

  //std::cout << as64_::io_::bold << as64_::io_::green << "DMP CartPos training..." << as64_::io_::reset << "\n";
	n_data = Time_demo.size();
  arma::vec y0 = Yd_data.col(0);
  arma::vec g = Yd_data.col(n_data-1);
	tau = Time_demo(n_data-1);
	canClockPtr->setTau(tau);
  // dmpCartPos->setTrainingParams(&trainParamList);
  //timer.tic();
  offline_train_p_mse= dmpCartPos->train(Time_demo, Yd_data, dYd_data, ddYd_data, y0, g, cmd_args.trainMethod);
  //std::cout << "Elapsed time is " << timer.toc() << "\n";

  //std::cout << as64_::io_::bold << as64_::io_::green << "DMP orient training..." << as64_::io_::reset << "\n";
  arma::vec Q0 = Qd_data.col(0);
  arma::vec Qg = Qd_data.col(n_data-1);
  // dmpOrient->setTrainingParams(&trainParamList);
  //timer.tic();
  offline_train_o_mse= dmpOrient->train(Time_demo, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg, cmd_args.trainMethod);
  //std::cout << "Elapsed time is " << timer.toc() << "\n";

	train_dmp = false;
	run_dmp = true; // now the DMP can run again
}

void DMP_Kuka_controller::init_execution()
{
  x = 0.0;
  dx = 0.0;

  Y0 = Yd_data.col(0);
  Yg0 = cmd_args.goal_scale*Yd_data.col(n_data-1);
  Yg = Yg0;
  Yg2 = Yg0;
  dg_p = arma::vec().zeros(Dp);
  Y = Y0;
  dY = arma::vec().zeros(Dp);
  ddY = arma::vec().zeros(Dp);
  dZ = arma::vec().zeros(Dp);
  Z = arma::vec().zeros(Dp);

  Q0 = Qd_data.col(0);
  Qg0 = cmd_args.goal_scale*Qd_data.col(n_data-1);
  Qg = Qg0;
  Qg2 = Qg0;
  dg_o = arma::vec().zeros(Do);
  Q = Q0;
  v_rot = arma::vec().zeros(Do);
  dv_rot = arma::vec().zeros(Do);
  deta = arma::vec().zeros(Do);
  eta = arma::vec().zeros(Do);

	ddEp = ddY_robot - ddY;
	dEp = dY_robot - dY;
	ddEo = dv_rot_robot - dv_rot;
	dEo = v_rot_robot - v_rot;

  // double tau0 = canClockPtr->getTau();
  // tau = cmd_args.tau_sim_scale*tau;
  // canClockPtr->setTau(tau);

  // iters = 0;
  // dt = cmd_args.dt;
}

void DMP_Kuka_controller::execute_DMP()
{
	// DMP CartPos simulation
	arma::vec Y_c = cmd_args.a_py*(Y_robot-Y);
	arma::vec Z_c = arma::vec(Dp).zeros();

	arma::vec X = arma::vec(Dp).fill(x);
	arma::vec dX = arma::vec(Dp).zeros();

	arma::mat statesDot;
	statesDot = dmpCartPos->getStatesDot(X, Y, Z, Y0, Yg, Y_c, Z_c);
	dZ = statesDot.col(0);
	dY = statesDot.col(1);
	// dX = statesDot.col(2);

	ddY = dZ/dmpCartPos->get_v_scale();
	// ddY_robot = ddY + (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot-Y) + Fdist_p );

	// DMP orient simulation
	arma::vec Q_c = cmd_args.a_py*quatLog(quatProd(Q_robot,quatInv(Q)));
	arma::vec eta_c = arma::vec(Do).zeros();

	X = arma::vec(Do).fill(x);
	dX = arma::vec(Do).zeros();

	std::vector<arma::vec> statesDot_o;
	statesDot_o = dmpOrient->getStatesDot(X, Q, eta, Q0, Qg, Q_c, eta_c);
	deta = statesDot_o[0];
	dQ = statesDot_o[1];
	// dX = statesDot_o[2];

	arma::vec v_rot_temp = 2*quatProd(dQ,quatInv(Q));
	v_rot = v_rot_temp.subvec(1,3);

	dv_rot = deta/dmpOrient->get_v_scale();
	// dv_rot_robot = dv_rot + (1/cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - cmd_args.Kd_o*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist_o );

	// Goal filtering
	if (cmd_args.USE_GOAL_FILT)
	{
		dg_p = cmd_args.a_g*(Yg2-Yg)/canClockPtr->getTau();
		dg_o = cmd_args.a_g*quatLog(quatProd(Qg2,quatInv(Qg)))/canClockPtr->getTau();
	}
	else
	{
		Yg = Yg2;
		dg_p.fill(0.0);
		Qg = Qg2;
		dg_o.fill(0.0);
	}

	// Update phase variable

	dx = canClockPtr->getPhaseDot(x);

	// Phase stopping
	if (cmd_args.USE_PHASE_STOP)
	{
			double stop_coeff = 1/(1+0.5*cmd_args.a_px*arma::norm(Y_robot - Y) + 0.5*cmd_args.a_px*arma::norm(quatLog(quatProd(Q_robot,quatInv(Q)))));

			dx = dx*stop_coeff;
			dg_p = dg_p*stop_coeff;
			dg_o = dg_o*stop_coeff;
	}

	// Numerical integration
	x = x + dx*Ts;

	Y = Y + dY*Ts;
	Z = Z + dZ*Ts;
	Yg = Yg + dg_p*Ts;

	Q = quatProd(quatExp(v_rot*Ts), Q);
	eta = eta + deta*Ts;
	Qg = quatProd(quatExp(dg_o*Ts),Qg);
}



void DMP_Kuka_controller::log_demo_step()
{
	Time_demo = join_horiz(Time_demo, t);
	Yd_data = join_horiz(Yd_data, Y_robot);
	dYd_data = join_horiz(dYd_data, dY_robot);
	ddYd_data = join_horiz(ddYd_data, ddY_robot);
	Qd_data = join_horiz(Qd_data, Q_robot);
	v_rot_d_data = join_horiz(v_rot_d_data, v_rot_robot);
	dv_rot_d_data = join_horiz(dv_rot_d_data, dv_rot_robot);
}

void DMP_Kuka_controller::log_online()
{
  log_data.Time = join_horiz(log_data.Time, t);

  log_data.y_data = join_horiz( log_data.y_data, arma::join_vert(Y, as64_::quat2qpos(Q)) );

  log_data.dy_data = join_horiz( log_data.dy_data, arma::join_vert(dY, v_rot) );
  log_data.z_data = join_horiz( log_data.z_data, arma::join_vert(Z, eta) );
  log_data.dz_data = join_horiz( log_data.dz_data, arma::join_vert(dZ, deta) );

  log_data.x_data = join_horiz(log_data.x_data, x);

  log_data.y_robot_data = join_horiz( log_data.y_robot_data, arma::join_vert(Y_robot, as64_::quat2qpos(Q_robot)) );
  log_data.dy_robot_data = join_horiz( log_data.dy_robot_data, arma::join_vert(dY_robot, v_rot_robot) );
  log_data.ddy_robot_data = join_horiz( log_data.ddy_robot_data, arma::join_vert(ddY_robot, dv_rot_robot) );

  log_data.Fdist_data = join_horiz( log_data.Fdist_data, arma::join_vert(Fdist_p, Fdist_o) );
  log_data.g_data = join_horiz( log_data.g_data, arma::join_vert(Yg, as64_::quat2qpos(Qg)) );
}

void DMP_Kuka_controller::log_offline()
{
  arma::vec y0 = Yd_data.col(0);
  arma::vec g = Yd_data.col(n_data-1);
  F_p_offline_train_data.resize(Dp, n_data);
  Fd_p_offline_train_data.resize(Dp, n_data);
  F_o_offline_train_data.resize(Do, n_data);
  Fd_o_offline_train_data.resize(Do, n_data);
  for (int j=0; j<Time_demo.size(); j++)
  {
    arma::vec X = dmpCartPos->phase(Time_demo(j));
    F_p_offline_train_data.col(j) = dmpCartPos->learnedForcingTerm(X, y0, g);
    Fd_p_offline_train_data.col(j) = dmpCartPos->calcFd(X, Yd_data.col(j), dYd_data.col(j), ddYd_data.col(j), y0, g);

    X = dmpOrient->phase(Time_demo(j));
    F_o_offline_train_data.col(j) = dmpOrient->learnedForcingTerm(X, Q0, Qg);
    Fd_o_offline_train_data.col(j) = dmpOrient->calcFd(X, Qd_data.col(j), v_rot_d_data.col(j), dv_rot_d_data.col(j), Q0, Qg);
  }
  Time_offline_train = Time_demo;

  log_data.poseDataFlag = true;

  log_data.DMP_w.resize(D);
  log_data.DMP_c.resize(D);
  log_data.DMP_h.resize(D);
  for (int i=0;i<D;i++)
  {
    log_data.DMP_w[i] = dmp[i]->w;
    log_data.DMP_c[i] = dmp[i]->c;
    log_data.DMP_h[i] = dmp[i]->h;
  }

  log_data.Time_demo = Time_demo;
  arma::mat yd_data(Do,n_data);
  for (int i=0;i<n_data;i++)
  {
    yd_data.col(i) = quat2qpos(Qd_data.col(i));
  }
  log_data.yd_data = arma::join_vert(Yd_data, yd_data);
  log_data.dyd_data = arma::join_vert(dYd_data, v_rot_d_data);
  log_data.ddyd_data = arma::join_vert(ddYd_data, dv_rot_d_data);

  log_data.D = D;
  log_data.Ts = Ts;
  log_data.g0 = arma::join_vert(Yg0, quat2qpos(Q0));

  log_data.Time_offline_train = Time_offline_train;
  log_data.F_offline_train_data = arma::join_vert(F_p_offline_train_data, F_o_offline_train_data);
  log_data.Fd_offline_train_data = arma::join_vert(Fd_p_offline_train_data, Fd_o_offline_train_data);
  log_data.Psi_data_train.resize(D);

  for (int i=0;i<D;i++)
  {
    int n_data = Time_offline_train.size();
    log_data.Psi_data_train[i].resize(dmp[i]->N_kernels,n_data);
    for (int j=0;j<n_data;j++)
    {
      double x = canClockPtr->getPhase(Time_offline_train(j));
      log_data.Psi_data_train[i].col(j) = dmp[i]->kernelFunction(x);
    }
  }

  log_data.Psi_data.resize(D);
  y0 = arma::join_vert(Y0, -quatLog(quatProd(Qg0,quatInv(Q0))) );
  arma::vec g0 = arma::join_vert(Yg0, arma::vec(Do).zeros());
  for (int i=0;i<D;i++)
  {
    int n_data = log_data.Time.size();
    log_data.Psi_data[i].resize(dmp[i]->N_kernels,n_data);
    log_data.Force_term_data.resize(D,n_data);
    for (int j=0;j<n_data;j++)
    {
      double x = log_data.x_data(j);
      log_data.Psi_data[i].col(j) = dmp[i]->kernelFunction(x);
      log_data.Force_term_data(i,j) = dmp[i]->learnedForcingTerm(x, y0(i), g0(i));
    }
  }

  log_data.goalAttr_data.resize(log_data.Time.size());
  log_data.shapeAttr_data.resize(log_data.Time.size());
  for (int j=0;j<log_data.Time.size();j++)
  {
    double x = log_data.x_data(j);
    log_data.goalAttr_data(j) = dmp[0]->goalAttrGating(x);
    log_data.shapeAttr_data(j) = dmp[0]->shapeAttrGating(x);
  }
}


void DMP_Kuka_controller::clear_logged_data()
{
	log_data.clear();
}

void DMP_Kuka_controller::save_and_restart_demo()
{
	save_logged_data();
	clear_and_restart_demo();
}

void DMP_Kuka_controller::clear_and_restart_demo()
{
	t = 0.0;
	Time_demo.clear();
  Yd_data.clear();
	dYd_data.clear();
	ddYd_data.clear();
  Qd_data.clear();
	v_rot_d_data.clear();
	dv_rot_d_data.clear();

	clear_logged_data();

	init_controller();
}

void DMP_Kuka_controller::save_logged_data()
{
	log_offline();

	std::ostringstream o_str;
	o_str << "";
	if (demo_save_counter > 0) o_str << demo_save_counter+1;

  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_data_filename + o_str.str(), true);
  log_data.save(cmd_args.out_data_filename + o_str.str(), false, 10);
  std::cout << "[DONE]!\n";

	demo_save_counter++;
}

void DMP_Kuka_controller::calc_simulation_mse()
{
  sim_mse.resize(D);
  for (int i=0;i<D;i++)
  {
    arma::rowvec y_robot_data2;
    arma::rowvec yd_data2;
    as64_::spl_::makeSignalsEqualLength(log_data.Time, log_data.y_robot_data.row(i),
            log_data.Time_demo, log_data.yd_data.row(i), log_data.Time,
            y_robot_data2, yd_data2);
    sim_mse(i) = arma::norm(y_robot_data2-yd_data2);
  }
}



void DMP_Kuka_controller::execute()
{
	execute_function:
  // ======  Read training data  =======
  record_demo();

  // ======  Get DMP  =======
  get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  std::vector<std::shared_ptr<as64_::DMP_>> dmpVec1(Dp);
  for (int i=0;i<Dp;i++) dmpVec1[i] = dmp[i];
  dmpCartPos.reset(new as64_::DMP_CartPos());
  dmpCartPos->init(dmpVec1);

  std::vector<std::shared_ptr<as64_::DMP_>> dmpVec2(Do);
  for (int i=0;i<Do;i++) dmpVec2[i] = dmp[Dp+i];
  dmpOrient.reset(new as64_::DMP_orient());
  dmpOrient->init(dmpVec2);

  // ======  Train the DMP  =======
  train_DMP();

  // for (int i=0;i<Dp;i++) std::cout << "DMP CartPos " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";
  // for (int i=0;i<Do;i++) std::cout << "DMP orient " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";
  // std::cout << "n_data = " << n_data << "\n";

  // Set initial conditions
  init_execution();

	update();

  // Start simulation
  while (ros::ok() && robot_->isOk() && !stop_robot)
  {
		//ros::spinOnce();

    // ========= data logging =============
		if (log_on) log_online();

		if (clear_restart_demo)
		{
			clear_and_restart_demo();
			goto execute_function;
		}

		if (save_restart_demo)
		{
			save_and_restart_demo();
			goto execute_function;
		}

		if (run_dmp) execute_DMP();

		if (train_dmp) train_DMP();

		if (goto_start) goto_start_pose();

		update();
		command();

  }
}

void DMP_Kuka_controller::update()
{
	t = t + Ts;

	robot_->getJointPosition(q_robot, ROBOT_ARM_INDEX);
	robot_->getTaskPose(T_robot_ee, ROBOT_ARM_INDEX);
	robot_->getJacobian(J_robot, ROBOT_ARM_INDEX);

	arma::vec Fee(CART_DOF_SIZE);
	robot_->getExternalWrench(Fee, ROBOT_ARM_INDEX);
	Fee.subvec(0,2) -= cmd_args.Fp_dead_zone;
	Fee.subvec(3,5) -=  cmd_args.Fo_dead_zone;
	for (int i=0; i<6; i++)
	{
		if (Fee(i) < 0) Fee(i) = 0.0;
	}

	Fdist_p = Fee.subvec(0,2);
	Fdist_o = Fee.subvec(3,5);

	if (arma::norm(Fee) > cmd_args.F_norm_retrain_thres) train_dmp = true;

	dq_robot = (q_robot - q_prev_robot) / Ts;
	V_robot = J_robot*dq_robot;

	Y_robot = T_robot_ee.submat(0,3,2,3);
	dY_robot = V_robot.subvec(0,2);
	ddY_robot = (dY_robot - dY_robot_prev) / Ts;

	Q_robot = as64_::rotm2quat(T_robot_ee.submat(0,0,2,2));
	v_rot_robot = V_robot.subvec(3,5);
	dv_rot_robot = (v_rot_robot - v_rot_robot_prev) / Ts;

	q_prev_robot = q_robot;
	dY_robot_prev = dY_robot;
	v_rot_robot_prev = v_rot_robot;
}

void DMP_Kuka_controller::command()
{
	double Kp = cmd_args.Kd_p;
	double Ko = cmd_args.Kd_o;

  // set the stiffness to a low value when the dmp is inactive so that the
	// user can move the robot freely
	if (run_dmp = false)
	{
		Kp = 1.0;
		Ko = 1.0;
	}

	ddEp = (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - Kp*(Y_robot-Y) + Fdist_p );
	ddEo = (1/cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - Ko*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist_o );

	dEp = dEp + ddEp*Ts;
	dEo = dEo + ddEo*Ts;

	arma::vec Vd(6);
	Vd.subvec(0,2) = (dEp + dY);
	Vd.subvec(3,5) = (dEo + v_rot);
	arma::vec qd = arma::pinv(J_robot)*Vd;

	robot_->setJointPosition(qd, ROBOT_ARM_INDEX);
	robot_->waitNextCycle();

	// Stopping criteria
	double err_p = arma::max(arma::abs(Yg2-Y_robot));
	double err_o = arma::norm(quatLog(quatProd(Qg,quatInv(Q_robot))));
	if (err_p <= cmd_args.tol_stop && err_o <= cmd_args.orient_tol_stop && t>=tau) goto_start = true;

}

void DMP_Kuka_controller::finalize()
{
	save_logged_data();
	keyboard_ctrl_thread->join();

	// calc_simulation_mse();
  // std::cout << "offline_train_p_mse = \n" << offline_train_p_mse << "\n";
  // std::cout << "offline_train_o_mse = \n" << offline_train_o_mse << "\n";
  // std::cout << "sim_mse = \n" << sim_mse << "\n";
}
