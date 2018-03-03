#include <Robot_PbD_controller.h>

#include <ros/package.h>

#include <io_lib/io_lib.h>
#include <time_lib/time.h>

using namespace as64_::math_;
using namespace as64_;

Robot_PbD_controller::Robot_PbD_controller()
{
  Dp = 3;
  Do = 3;
  D = Dp + Do;

  // robot_ = robot;
  Ts = 0.008;// robot_->cycle;

  std::cout << io_::bold << io_::green << "Reading params from yaml file..." << io_::reset << "\n";
  // cmd_args.parse_cmd_args();
  // cmd_args.print();

  demo_save_counter = 0;

  initController();

  keyboard_ctrl_thread.reset(new std::thread(&Robot_PbD_controller::keyboardCtrlThreadFun, this));
}

Robot_PbD_controller::~Robot_PbD_controller()
{
  this->finalize();
}

void Robot_PbD_controller::parseConfigFile(const char *config_file)
{
  std::string filename;
  if (config_file != NULL) filename = *config_file;
  else filename = ros::package::getPath("robot_pbd_controller")+ "/config/Robot_PbD_controller_config.yml";

  as64_::param_::Parser parser(filename);

  // if (!parser.getParam("a",a)) a = 0.0;
}

void Robot_PbD_controller::initControlFlags()
{
  pause_robot = false;
  train_from_file = false;
  save_exec_results = false;
  log_on = false;
  run_model = false;
  train_model = false;
  goto_start = false;
  stop_robot = false;
  start_demo = false;
  end_demo = false;
  clear_restart_demo = false;
  save_restart_demo = false;
}

void Robot_PbD_controller::initProgramVariables()
{
  getJointPosition(q_robot);
  getJointVelocity(dq_robot);
  // q_prev_robot = q_robot;
  // dq_robot = (q_robot - q_prev_robot) / Ts;

  getTaskPose(T_robot_ee);
  Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  Q_robot = rotm2quat(T_robot_ee.submat(0, 0, 2, 2));

  t = 0.0;

  Y = Y_robot;
  dY = arma::vec().zeros(Dp);
  ddY = arma::vec().zeros(Dp);
  dY_robot = arma::vec().zeros(Dp);
  ddY_robot = arma::vec().zeros(Dp);
  Fpos = arma::vec().zeros(Dp);

  Q = Q_robot;
  v_rot = arma::vec().zeros(Do);
  dv_rot = arma::vec().zeros(Do);
  v_rot_robot = arma::vec().zeros(Do);
  dv_rot_robot = arma::vec().zeros(Do);
  Forient = arma::vec().zeros(Do);

  Fee = arma::vec().zeros(D);

  dY_robot_prev = arma::vec().zeros(Dp);
  v_rot_robot_prev = arma::vec().zeros(Do);

  J_robot = arma::zeros<arma::mat>(DOFs,N_JOINTS);

  ddEp = arma::zeros<arma::vec>(3);
  dEp = arma::zeros<arma::vec>(3);
  Ep =  arma::zeros<arma::vec>(3);

  ddEo =  arma::zeros<arma::vec>(3);
  dEo =  arma::zeros<arma::vec>(3);
}

void Robot_PbD_controller::initController()
{
    initControlFlags();
    initProgramVariables();
}

void Robot_PbD_controller::keyboardCtrlThreadFun()
{
  using namespace as64_::io_;
  int key = 0;
  while (!stop_robot) { // Enter
    key = io_::getch();
    std::cout << green << bold  << "[KEY PRESSED]: ";

    switch (key) {
      case 32: //Spacebar to toggle DMP execution (run/stop)
        run_model = !run_model;
        std::cout << (run_model?"Run DMP\n":"Stop DMP\n");
        break;
      case 'c':
        clear_restart_demo = true;
        std::cout << "Clear and restart demo\n";
        break;
      case 'r':
        save_restart_demo = true;
        std::cout << "Save and restart demo\n";
        break;
      case 'l':
        log_on = true;
        std::cout << "Log on enabled\n";
        break;
      case 's':
        stop_robot = true;
        std::cout << "Stop robot\n";
        break;
      case 'p':
        pause_robot = !pause_robot;
        std::cout << (pause_robot?"Pause":"Start") << " robot\n";
        break;
      case 'n':
        start_demo = true;
        std::cout << "Start demo\n";
        break;
      case 'm':
        end_demo = true;
        std::cout << "End demo\n";
        break;
      case 'v':
        std::cout << "Saving execution results\n";
        save_exec_results_thread.reset(new std::thread(&Robot_PbD_controller::saveExecutionResults, this));
        save_exec_results_thread->detach();
        break;
      case 'f':
        train_from_file = !train_from_file;
        std::cout << "Training: use training data from " << (train_from_file?"file":"demo recording") << "\n";
        break;
    }
    std::cout << reset;
  }
}

void Robot_PbD_controller::saveExecutionResults()
{
  using namespace io_;
  if (log_on)
  {
    std::cout << red << bold << "Cannot save execution results!\n";
    std::cout << "Reason: \"log on\" is enabled...\n" << reset;
    return;
  }
  save_exec_results = true;
  saveLoggedData();
  save_exec_results = false;
}

void Robot_PbD_controller::robotWait()
{
  update();
  command();
}

void Robot_PbD_controller::recordDemo()
{
    run_model = false; // stop running the DMP
    train_model = false;
    initProgramVariables(); // to zero all velocities and accelerations and set all poses/joints to the current ones
    clearTrainData();

    pause_robot = true;

    using namespace io_;
    std::cout << blue << bold << "Robot paused ! Please press 'p' to continue ..." << std::endl << reset;

    while (pause_robot) robotWait();

    if(!train_from_file){
      while (!start_demo) {
          robotWait();
      }

      t = 0.0;

    //update();

      q0_robot = q_robot; // save intial joint positions
      while (!end_demo) {
          logDemoStep();
          command();
          update();
      }
      // trainData.q0 = q0_robot;
      // trainData.getNumData() = trainData.Time.size();
    }
    else
    {
      loadDemoData();
    }

    start_demo = false;
    end_demo = false;
    train_model = true; // the DMP must be trained now with the demo data

    Dp = trainData.dY_data.n_rows;
    Do = trainData.v_rot_data.n_rows;
    D = Dp + Do;

    // get the time duration
    tau = trainData.Time(trainData.getNumData() - 1);

    // save initial pose
    Y0 = trainData.Y_data.col(0);
    Q0 = trainData.Q_data.col(0);

    // save goal-target pose
    Yg0 = trainData.Y_data.col(trainData.getNumData() - 1);
    Qg0 = trainData.Q_data.col(trainData.getNumData() - 1);

    // trainData0 = trainData;

    saveDemoData();

     //std::cout << "trainData.getNumData() = " << trainData.getNumData() << "\n";
     //std::cout << "trainData.Time: " << trainData.Time.size() << "\n";
     //std::cout << "trainData.Y_data: " << trainData.Y_data.n_rows << " x " << trainData.Y_data.n_cols << "\n";
     //std::cout << "trainData.dY_data: " << trainData.dY_data.n_rows << " x " << trainData.dY_data.n_cols << "\n";
     //std::cout << "trainData.ddY_data: " << trainData.ddY_data.n_rows << " x " << trainData.ddY_data.n_cols << "\n";
}


void Robot_PbD_controller::saveDemoData()
{
  std::string train_data_save_file = cmd_args.data_output_path + "/kuka_demo_data.bin";
  bool binary = true;

  std::ofstream out;
  out.open(train_data_save_file, std::ios::binary);

  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: \"") + train_data_save_file + "\"");

  io_::write_mat(trainData.Time, out, binary);

  io_::write_mat(trainData.Y_data, out, binary);
  io_::write_mat(trainData.dY_data, out, binary);
  io_::write_mat(trainData.ddY_data, out, binary);

  io_::write_mat(trainData.Q_data, out, binary);
  io_::write_mat(trainData.v_rot_data, out, binary);
  io_::write_mat(trainData.dv_rot_data, out, binary);

  // io_::write_mat(trainData.q0, out, binary);

  out.close();
}


void Robot_PbD_controller::loadDemoData()
{
  std::string train_data_load_file = cmd_args.data_output_path + "/kuka_demo_data.bin";
  bool binary = true;

  std::ifstream in;
  in.open(train_data_load_file, std::ios::binary);

  if (!in) throw std::ios_base::failure(std::string("Couldn't open file: \"") + train_data_load_file + "\"");

  io_::read_mat(trainData.Time, in, binary);

  io_::read_mat(trainData.Y_data, in, binary);
  io_::read_mat(trainData.dY_data, in, binary);
  io_::read_mat(trainData.ddY_data, in, binary);

  io_::read_mat(trainData.Q_data, in, binary);
  io_::read_mat(trainData.v_rot_data, in, binary);
  io_::read_mat(trainData.dv_rot_data, in, binary);

  // io_::read_mat(trainData.q0, in, binary);

  in.close();
}

void Robot_PbD_controller::gotoStartPose()
{
    using namespace io_;

    std::cout << bold << cyan << "Moving to start pose...\n" << reset;

    //if (!)

    //robot_->setJointTrajectory(trainData.q0, tau + 1.5);
    initProgramVariables();
    goto_start = false;

    std::cout << bold << cyan << "Reached start pose!\n" << reset;
}

void Robot_PbD_controller::logDemoStep()
{
  //  std::cout << "Log Demo step\n";
  trainData.Time = join_horiz(trainData.Time, arma::mat({t}));

  trainData.q_data = join_horiz(trainData.q_data, q_robot);
  trainData.dq_data = join_horiz(trainData.dq_data, dq_robot);

  trainData.Y_data = join_horiz(trainData.Y_data, Y_robot);
  trainData.dY_data = join_horiz(trainData.dY_data, dY_robot);
  trainData.ddY_data = join_horiz(trainData.ddY_data, ddY_robot);

  trainData.Q_data = join_horiz(trainData.Q_data, Q_robot);
  trainData.v_rot_data = join_horiz(trainData.v_rot_data, v_rot_robot);
  trainData.dv_rot_data = join_horiz(trainData.dv_rot_data, dv_rot_robot);

  trainData.wrench_data = join_horiz(trainData.wrench_data, Fee);
}

void Robot_PbD_controller::logOnline()
{
  // //  std::cout << "Log online";
  //   // logDemoStep();
  //
  //   log_data.Time = join_horiz(log_data.Time, t);
  //
  //   log_data.y_data = join_horiz(log_data.y_data, arma::join_vert(Y, quat2qpos(Q)));
  //
  //   log_data.dy_data = join_horiz(log_data.dy_data, arma::join_vert(dY, v_rot));
  //   log_data.z_data = join_horiz(log_data.z_data, arma::join_vert(Z, eta));
  //   log_data.dz_data = join_horiz(log_data.dz_data, arma::join_vert(dZ, deta));
  //
  //   log_data.x_data = join_horiz(log_data.x_data, x);
  //
  //   log_data.y_robot_data = join_horiz(log_data.y_robot_data, arma::join_vert(Y_robot, quat2qpos(Q_robot)));
  //   log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, arma::join_vert(dY_robot, v_rot_robot));
  //   log_data.ddy_robot_data = join_horiz(log_data.ddy_robot_data, arma::join_vert(ddY_robot, dv_rot_robot));
  //
  //   log_data.Fdist_data = join_horiz(log_data.Fdist_data, arma::join_vert(Fpos, Forient));
  //   log_data.g_data = join_horiz(log_data.g_data, arma::join_vert(Yg, quat2qpos(Qg)));
}

void Robot_PbD_controller::logOffline()
{
  // arma::rowvec Time_offline_train;
  // arma::mat F_p_offline_train_data;
  // arma::mat Fd_p_offline_train_data;
  // arma::mat F_o_offline_train_data;
  // arma::mat Fd_o_offline_train_data;
  //
  // arma::vec y0 = trainData.Y_data.col(0);
  // arma::vec g = trainData.Y_data.col(trainData.getNumData() - 1);
  // F_p_offline_train_data.resize(Dp, trainData.getNumData());
  // Fd_p_offline_train_data.resize(Dp, trainData.getNumData());
  // F_o_offline_train_data.resize(Do, trainData.getNumData());
  // Fd_o_offline_train_data.resize(Do, trainData.getNumData());
  //
  // for (int j = 0; j < trainData.Time.size(); j++) {
  //     arma::vec X = dmpCartPos->phase(trainData.Time(j));
  //     F_p_offline_train_data.col(j) = dmpCartPos->learnedForcingTerm(X, y0, g);
  //     Fd_p_offline_train_data.col(j) = dmpCartPos->calcFd(X, trainData.Y_data.col(j), trainData.dY_data.col(j), trainData.ddY_data.col(j), y0, g);
  //
  //     X = dmpOrient->phase(trainData.Time(j));
  //     F_o_offline_train_data.col(j) = dmpOrient->learnedForcingTerm(X, Q0, Qg);
  //     Fd_o_offline_train_data.col(j) = dmpOrient->calcFd(X, trainData.Q_data.col(j), trainData.v_rot_data.col(j), trainData.dv_rot_data.col(j), Q0, Qg);
  // }
  // Time_offline_train = trainData.Time;
  //
  // log_data.poseDataFlag = true;
  //
  // log_data.DMP_w.resize(D);
  // log_data.DMP_c.resize(D);
  // log_data.DMP_h.resize(D);
  // for (int i = 0; i < D; i++) {
  //     log_data.DMP_w[i] = dmp[i]->w;
  //     log_data.DMP_c[i] = dmp[i]->c;
  //     log_data.DMP_h[i] = dmp[i]->h;
  // }
  //
  // log_data.Time_demo = trainData.Time;
  // arma::mat yd_data(Do, trainData.getNumData());
  // for (int i = 0; i < trainData.getNumData(); i++) {
  //     yd_data.col(i) = quat2qpos(trainData.Q_data.col(i));
  // }
  // log_data.yd_data = arma::join_vert(trainData.Y_data, yd_data);
  // log_data.dyd_data = arma::join_vert(trainData.dY_data, trainData.v_rot_data);
  // log_data.ddyd_data = arma::join_vert(trainData.ddY_data, trainData.dv_rot_data);
  //
  // log_data.D = D;
  // log_data.Ts = Ts;
  // log_data.g0 = arma::join_vert(Yg0, quat2qpos(Q0));
  //
  // log_data.Time_offline_train = Time_offline_train;
  // log_data.F_offline_train_data = arma::join_vert(F_p_offline_train_data, F_o_offline_train_data);
  // log_data.Fd_offline_train_data = arma::join_vert(Fd_p_offline_train_data, Fd_o_offline_train_data);
  // log_data.Psi_data_train.resize(D);
  //
  // for (int i = 0; i < D; i++) {
  //     int n_data = Time_offline_train.size();
  //     log_data.Psi_data_train[i].resize(dmp[i]->N_kernels, n_data);
  //     for (int j = 0; j < n_data; j++) {
  //         double x = canClockPtr->getPhase(Time_offline_train(j));
  //         log_data.Psi_data_train[i].col(j) = dmp[i]->kernelFunction(x);
  //     }
  // }

}

void Robot_PbD_controller::clearTrainData()
{
  trainData.Time.clear();

  trainData.q_data.clear();
  trainData.dq_data.clear();

  trainData.Y_data.clear();
  trainData.dY_data.clear();
  trainData.ddY_data.clear();

  trainData.Q_data.clear();
  trainData.v_rot_data.clear();
  trainData.dv_rot_data.clear();

  trainData.wrench_data.clear();
}

void Robot_PbD_controller::clearLoggedData()
{
  log_data.clear();
}

void Robot_PbD_controller::saveAndRestartDemo()
{
    saveLoggedData();
    clearAndRestartDemo();
}

void Robot_PbD_controller::clearAndRestartDemo()
{
    clearTrainData();
    clearLoggedData();
    initController();
}

void Robot_PbD_controller::saveLoggedData()
{
    // logOffline();
    //
    // std::ostringstream o_str;
    // o_str << "";
    // if (demo_save_counter > 0) o_str << demo_save_counter + 1;
    //
    // std::string suffix = cmd_args.DMP_TYPE + "_" + tm_::getTimeStamp();
    //
    // std::cout << "Saving results...";
    // log_data.cmd_args = cmd_args;
    // std::cout << "Saving to \"" << cmd_args.out_data_filename + o_str.str() <<"\" ...\n";
    // log_data.save(cmd_args.out_data_filename + o_str.str(), true);
    // std::cout << "Saving to \"" << cmd_args.out_data_filename + suffix <<"\" ...\n";
    // log_data.save(cmd_args.out_data_filename + suffix, true);
    // //log_data.save(cmd_args.out_data_filename + o_str.str(), false, 10);
    // std::cout << "[DONE]!\n";
    //
    // demo_save_counter++;
}


void Robot_PbD_controller::execute()
{
  restart_demo_label:
  // ======  Read training data  =======
  recordDemo();

  // // ======  Get DMP  =======
  // get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);
  //
  // std::vector<std::shared_ptr<DMP_>> dmpVec1(Dp);
  // for (int i = 0; i < Dp; i++) dmpVec1[i] = dmp[i];
  // dmpCartPos.reset(new DMP_CartPos());
  // dmpCartPos->init(dmpVec1);
  //
  // std::vector<std::shared_ptr<DMP_>> dmpVec2(Do);
  // for (int i = 0; i < Do; i++) dmpVec2[i] = dmp[Dp + i];
  // dmpOrient.reset(new DMP_orient());
  // dmpOrient->init(dmpVec2);

  restart_dmp_execution_label:

  run_model = false;
  log_on = false;

  gotoStartPose();

    if (train_model)
    {
      train_model_thread.reset(new std::thread(&Robot_PbD_controller::trainModel, this));
      train_model_thread->detach();
    }

    while (train_model) robotWait();


    while (!run_model) robotWait();

    while (save_exec_results) robotWait();

    clearLoggedData();
    //clearTrainData();

    initProgramVariables();

    // Start simulation
    while (ros::ok() && !stop_robot) {
      //ros::spinOnce();

      // ========= data logging =============
      if (log_on) logOnline();

      if (clear_restart_demo)
      {
        clearAndRestartDemo();
        goto restart_demo_label;
      }

      if (save_restart_demo)
      {
        saveAndRestartDemo();
        goto restart_demo_label;
      }

      if (run_model)
      {
        executeModel();
      }

      if (goto_start) goto restart_dmp_execution_label;

      //if (arma::norm(Fee) > cmd_args.F_norm_retrain_thres) train_model = true;

      update();
      command();

      if (run_model)
      {
        // // Stopping criteria
        // double err_p = arma::max(arma::abs(Yg2 - Y_robot));
        // double err_o = 0*arma::norm(quatLog(quatProd(Qg, quatInv(Q_robot))));
        // if (err_p <= cmd_args.pos_tol_stop && err_o <= cmd_args.orient_tol_stop && t >= tau)
        // {
        //   goto_start = true;
        //   std::cout << io_::cyan << "Target reached. Moving to start pose...\n" << io_::reset;
        // }
      }

    }
}

void Robot_PbD_controller::update()
{
  t = t + Ts;

  getJointPosition(q_robot);
  getTaskPose(T_robot_ee);
  getJacobian(J_robot);
  getExternalWrench(Fee); // Need to invert the sign of Fee ??????

  arma::vec Fee_sign = arma::sign(Fee);
  for(int i=0;i<6;i++)
  {
    Fee(i) = (Fee_sign(i)) * fmax(0.0,fabs(Fee(i)) - cmd_args.F_dead_zone(i));
  }
  // std::cout<<"Fee in update() 2: "<< Fee.t();
  // Fee = Fee - Fee_sign % cmd_args.F_dead_zone;
  // Fee = 0.5 * (arma::sign(Fee) + Fee_sign) % Fee;

  // Fpos =  Fee.rows(0,2);
  // Forient =  Fee.rows(3,5);
  //
  // dq_robot = (q_robot - q_prev_robot) / Ts;
  // V_robot = J_robot * dq_robot;
  //
  // Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  // dY_robot = V_robot.subvec(0, 2);
  // ddY_robot = (dY_robot - dY_robot_prev) / Ts;
  //
  // Q_robot = rotm2quat(T_robot_ee.submat(0, 0, 2, 2));
  // v_rot_robot = V_robot.subvec(3, 5);
  // dv_rot_robot = (v_rot_robot - v_rot_robot_prev) / Ts;
  //
  // q_prev_robot = q_robot;
  // dY_robot_prev = dY_robot;
  // v_rot_robot_prev = v_rot_robot;
}

void Robot_PbD_controller::command()
{
  // double Kp = cmd_args.Kd_p;
  // double Ko = cmd_args.Kd_o;
  // double  factor_force = 0;
  //
  // // set the stiffness to a low value when the dmp is inactive so that the
  // // user can move the robot freely
  // if (!run_model) {
  //     Kp = 0.0;
  //     Ko = 0.0;
  //     factor_force = 1.0;
  // }
  //
  // //ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * (dY_robot - dY) - Kp * (Y_robot - Y) + factor_force*Fpos);
  // ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * dEp - Kp * Ep + factor_force*Fpos);
  //
  // //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot - v_rot) - Ko * quatLog(quatProd(Q_robot, quatInv(Q))) + factor_force*Forient);
  // //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot) + factor_force*Forient);
  // ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * dEo + factor_force*Forient);
  //
  // dEp = dEp + ddEp * Ts;
  // dEo = dEo + ddEo * Ts;
  // Ep = Ep + dEp * Ts;
  //
  // arma::vec Vd(6);
  // Vd.subvec(0, 2) = (dEp + dY - 4.0*(Y_robot - (Y + Ep)));
  // Vd.subvec(3, 5) = (dEo + v_rot - 4.0*quatLog( quatProd( Q_robot, quatInv(Q) ) ) );
  //
  //
  // if(run_model || goto_start){
  //   if(robot_->getMode() != ur10_::Mode::VELOCITY_CONTROL){
  //     robot_->setMode(ur10_::Mode::VELOCITY_CONTROL);
  //   }
  //   Vd.rows(3,5) = arma::zeros<arma::vec>(3);
  //   arma::vec qd = arma::pinv(J_robot) * Vd;
  //   robot_->setJointVelocity(qd);
  // }else{
  //   if(robot_->getMode() != ur10_::Mode::TORQUE_CONTROL){
  //     robot_->setMode(ur10_::Mode::TORQUE_CONTROL);
  //   }
  //   arma::vec torque_for_rotation = arma::zeros<arma::vec>(7);
  //     //  torque_for_rotation = - (J_robot.rows(3,5)).t() * 5.0* quatLog(quatProd(Q_robot, quatInv(trainData.Q_data.col(0))));
  //   robot_->setJointTorque(torque_for_rotation);
  //   //std::cout<<"torque_for_rotation: " << torque_for_rotation.t() <<std::endl;
  // }
  // robot_->waitNextCycle();

}

void Robot_PbD_controller::finalize()
{
    //saveLoggedData();
    if (keyboard_ctrl_thread->joinable()) keyboard_ctrl_thread->join();

}
