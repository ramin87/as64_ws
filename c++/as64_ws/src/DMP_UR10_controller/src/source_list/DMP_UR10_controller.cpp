#include <DMP_UR10_controller.h>
#include <io_lib/io_lib.h>
#include <time_lib/time.h>
#include <plot_lib/plot_lib.h>

using namespace as64_::math_;
using namespace as64_;

DMP_UR10_controller::DMP_UR10_controller(std::shared_ptr<ur10_::Robot> robot)
{
  Dp = 3;
  Do = 3;
  D = Dp + Do;

  robot_ = robot;
  Ts = robot_->cycle;

  std::cout << io_::bold << io_::green << "Reading params from yaml file..." << io_::reset << "\n";
  cmd_args.parse_cmd_args();
  cmd_args.print();

  F_dead_zone.resize(6);
  F_dead_zone.subvec(0, 2).fill(cmd_args.Fp_dead_zone);
  F_dead_zone.subvec(3, 5).fill(cmd_args.Fo_dead_zone);

  demo_save_counter = 0;

  initController();

  keyboard_ctrl_thread.reset(new std::thread(&DMP_UR10_controller::keyboardCtrlThreadFun, this));
}

void DMP_UR10_controller::initControlFlags()
{
  pause_robot = false;
  train_from_file = false;
  save_exec_results = false;
  log_on = false;
  run_dmp = false;
  train_dmp = false;
  goto_start = false;
  stop_robot = false;
  start_demo = false;
  end_demo = false;
  clear_restart_demo = false;
  save_restart_demo = false;
}

void DMP_UR10_controller::initProgramVariables()
{
  q_robot = robot_->getJointPosition();
  q_prev_robot = q_robot;

  dq_robot = robot_->getJointVelocity();
  // dq_robot
  // dq_robot = (q_robot - q_prev_robot) / Ts;

  T_robot_ee = robot_->getTaskPose();
  Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  Q_robot = math_::rotm2quat(T_robot_ee.submat(0, 0, 2, 2));

  t = 0.0;
  x = 0.0;
  dx = 0.0;

  Yg = cmd_args.goal_scale * Yg0; // Yg0 is initialized only after "recordDemo"
  Yg2 = Yg;
  dg_p = arma::vec().zeros(Dp);
  Y = Y_robot; // Y0
  dY = arma::vec().zeros(Dp);
  ddY = arma::vec().zeros(Dp);
  dZ = arma::vec().zeros(Dp);
  Z = arma::vec().zeros(Dp);
  ddY_robot = arma::vec().zeros(Dp);
  dY_robot = arma::vec().zeros(Dp);
  Fdist_p = arma::vec().zeros(Dp);

  // Qg = cmd_args.goal_scale * Qg0; // Qg0 is initialized only after "recordDemo"
  Qg = Qg0;
  Qg2 = Qg;
  dg_o = arma::vec().zeros(Do);
  Q = Q_robot; // Q0
  v_rot = arma::vec().zeros(Do);
  dv_rot = arma::vec().zeros(Do);

  v_rot_robot = arma::vec().zeros(Do);
  dv_rot_robot = arma::vec().zeros(Do);

  Fdist_p = arma::vec().zeros(Do);

  Fee = arma::vec().zeros(D);

  deta = arma::vec().zeros(Do);
  eta = arma::vec().zeros(Do);

  dY_robot_prev = arma::vec().zeros(Dp);
  v_rot_robot_prev = arma::vec().zeros(Do);

  J_robot = arma::zeros<arma::mat>(D,7);
 // robot_->getJacobian(J_robot);

  //ddEp = ddY_robot - ddY;
  ddEp = arma::zeros<arma::vec>(3);
  //dEp = dY_robot - dY;
  dEp = arma::zeros<arma::vec>(3);
  //ddEo = dv_rot_robot - dv_rot;
  ddEo =  arma::zeros<arma::vec>(3);
  //dEo = v_rot_robot - v_rot;
  dEo =  arma::zeros<arma::vec>(3);

  Ep =  arma::zeros<arma::vec>(3);

}

void DMP_UR10_controller::initController()
{
  initControlFlags();
  initProgramVariables();
}

void DMP_UR10_controller::keyboardCtrlThreadFun()
{
  using namespace io_;
  // run_dmp = false;
  // train_dmp = false;
  // goto_start = false;

  int key = 0;
  while (!stop_robot) { // Enter
    key = io_::getch();

    //std::cout << "Pressed " << (char)key  << "\n";
    std::cout << green << bold  << "[KEY PRESSED]: ";

    switch (key) {
      case 32: //Spacebar to toggle DMP execution (run/stop)
        run_dmp = !run_dmp;
        std::cout << (run_dmp?"Run DMP\n":"Stop DMP\n");
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
        save_exec_results_thread.reset(new std::thread(&DMP_UR10_controller::saveExecutionResults, this));
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

void DMP_UR10_controller::saveExecutionResults()
{
  using namespace io_;
  if (log_on)
  {
    std::cout << red << bold << "Cannot save execution results!\n";
    std::cout << "Reason: \"log on\" is enabled...\n" << reset;
    return;
  }
  save_exec_results = true;
  save_logged_data();
  save_exec_results = false;
}

void DMP_UR10_controller::robotWait()
{
  update();
  command();
}

void DMP_UR10_controller::recordDemo()
{
  run_dmp = false; // stop running the DMP
  train_dmp = false;
  initProgramVariables(); // to zero all velocities and accelerations and set all poses/joints to the current ones
  clearTrainData();

  pause_robot = true;

  using namespace io_;
  std::cout << blue << bold << "Robot paused ! Please press 'p' to continue ..." << std::endl << reset;

  while (pause_robot) robotWait();

  if(!train_from_file)
  {
    while (!start_demo)
    {
      robotWait();
    }

    t = 0.0;

    //update();

    q0_robot = q_robot; // save intial joint positions
    while (!end_demo)
    {
      logDemoStep();
      command();
      update();
    }
    trainData.q0 = q0_robot;
    trainData.n_data = trainData.Time.size();
  }
  else
  {
  loadDemoData();
  }

  start_demo = false;
  end_demo = false;
  train_dmp = true; // the DMP must be trained now with the demo data

  Dp = trainData.dY_data.n_rows;
  Do = trainData.v_rot_data.n_rows;
  D = Dp + Do;

  // get the time duration
  tau = trainData.Time(trainData.n_data - 1);

  // save initial pose
  Y0 = trainData.Y_data.col(0);
  Q0 = trainData.Q_data.col(0);

  // save goal-target pose
  Yg0 = trainData.Y_data.col(trainData.n_data - 1);
  Qg0 = trainData.Q_data.col(trainData.n_data - 1);

  trainData0 = trainData;

  saveDemoData();

  std::cout << "trainData.n_data = " << trainData.n_data << "\n";
  std::cout << "trainData.Time: " << trainData.Time.size() << "\n";
  std::cout << "trainData.Y_data: " << trainData.Y_data.n_rows << " x " << trainData.Y_data.n_cols << "\n";
  std::cout << "trainData.dY_data: " << trainData.dY_data.n_rows << " x " << trainData.dY_data.n_cols << "\n";
  std::cout << "trainData.ddY_data: " << trainData.ddY_data.n_rows << " x " << trainData.ddY_data.n_cols << "\n";
}


void DMP_UR10_controller::saveDemoData()
{
  std::string train_data_save_file = cmd_args.data_output_path + "/kuka_demo_data.bin";
  bool binary = true;

  std::ofstream out;
  out.open(train_data_save_file, std::ios::binary);

  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: \"") + train_data_save_file + "\"");

  io_::write_mat(trainData0.Time, out, binary);

  io_::write_mat(trainData0.Y_data, out, binary);
  io_::write_mat(trainData0.dY_data, out, binary);
  io_::write_mat(trainData0.ddY_data, out, binary);

  io_::write_mat(trainData0.Q_data, out, binary);
  io_::write_mat(trainData0.v_rot_data, out, binary);
  io_::write_mat(trainData0.dv_rot_data, out, binary);

  io_::write_mat(trainData0.q0, out, binary);

  out.close();
}


void DMP_UR10_controller::loadDemoData()
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

  io_::read_mat(trainData.q0, in, binary);

  trainData.n_data = trainData.Time.size();

  in.close();
}

void DMP_UR10_controller::gotoStartPose()
{
  using namespace io_;

  //robot_->setMode(ur10_::Mode::POSITION_CONTROL);

  std::cout << bold << cyan << "Moving to start pose...\n" << reset;

  robot_->setJointTrajectory(trainData0.q0, tau + 1.5);
  initProgramVariables();
  goto_start = false;

  std::cout << bold << cyan << "Reached start pose!\n" << reset;
}


void DMP_UR10_controller::trainDMP()
{
  // param_::ParamList trainParamList;
  // trainParamList.setParam("lambda", cmd_args.lambda);
  // trainParamList.setParam("P_cov", cmd_args.P_cov);

  using namespace io_;
  std::cout << yellow << bold << "Training DMP...\n" << reset;

  //std::cout << io_::bold << io_::green << "DMP CartPos training..." << io_::reset << "\n";
  trainData.n_data = trainData.Time.size();
  arma::vec y0 = trainData.Y_data.col(0);
  arma::vec g = trainData.Y_data.col(trainData.n_data - 1);
  tau = trainData.Time(trainData.n_data - 1);
  canClockPtr->setTau(tau);
  // dmpCartPos->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_p_mse = dmpCartPos->train(trainData.Time, trainData.Y_data, trainData.dY_data, trainData.ddY_data, y0, g, cmd_args.trainMethod, true);
  std::cout << "CartPos: Elapsed time is " << timer.toc() << "\n";

  //std::cout << io_::bold << io_::green << "DMP orient training..." << io_::reset << "\n";
  arma::vec Q0 = trainData.Q_data.col(0);
  arma::vec Qg = trainData.Q_data.col(trainData.n_data - 1);
  // dmpOrient->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_o_mse = dmpOrient->train(trainData.Time, trainData.Q_data, trainData.v_rot_data, trainData.dv_rot_data, Q0, Qg, cmd_args.trainMethod, true);
  std::cout << "Orientation: Elapsed time is " << timer.toc() << "\n";

  for (int i=0; i<Dp; i++) std::cout << "CartPos " << i+1 << " kernels = " << dmpCartPos->dmp[i]->N_kernels << "\n";
  for (int i=0; i<Do; i++) std::cout << "Orient " << i+1 << " kernels = " << dmpOrient->dmp[i]->N_kernels << "\n";

  train_dmp = false;
  //run_dmp = true; // now the DMP can run again

  // std::cout << "trainData.n_data = " << trainData.n_data << "\n";
  // std::cout << "trainData.Time: " << trainData.Time.size() << "\n";
  // std::cout << "trainData.Y_data: " << trainData.Y_data.n_rows << " x " << trainData.Y_data.n_cols << "\n";
  // std::cout << "trainData.dY_data: " << trainData.dY_data.n_rows << " x " << trainData.dY_data.n_cols << "\n";
  // std::cout << "trainData.ddY_data: " << trainData.ddY_data.n_rows << " x " << trainData.ddY_data.n_cols << "\n";
  std::cout << "offline_train_p_mse: " << offline_train_p_mse.t() <<  "\n";
  std::cout << "offline_train_o_mse: " <<  offline_train_o_mse.t() <<  "\n";

  std::cout << "y0 = " << y0.t() << "\n";
}

void DMP_UR10_controller::executeDMP()
{
  // DMP CartPos simulation
  arma::vec Y_c = cmd_args.a_py * (Y_robot - Y);
  arma::vec Z_c = arma::vec(Dp).zeros();

  arma::vec X = arma::vec(Dp).fill(x);
  arma::vec dX = arma::vec(Dp).zeros();

  arma::mat statesDot;
  statesDot = dmpCartPos->getStatesDot(X, Y, Z, Y0, Yg, Y_c, Z_c);
  dZ = statesDot.col(0);
  dY = statesDot.col(1);
  // dX = statesDot.col(2);

 // arma::vec dZ_dmp = dZ;

 // statesDot.resize(Dp,3);
  //for (int i=0; i<Dp; i++){
  //  statesDot.row(i) = dmp[i]->getStatesDot(X(i), Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i)).t();
  //}


  //dZ = statesDot.col(0);
  //dY = statesDot.col(1);



  /*

  double gAttrGating = dmp[0]->goalAttrGating(x);
  dZ = dmp[0]->goalAttrGating(x)*dmp[0]->a_z*(dmp[0]->b_z*(Yg-Y)-Z)/ ( dmp[0]->get_v_scale() );
  */


  //std::cout << "Z_err " << arma::norm(dZ_dmp - dZ) << std::endl;

  ddY = dZ / dmpCartPos->get_v_scale();

  // std::cout << " dmpCartPos->get_v_scale(): "<<  dmpCartPos->get_v_scale() << std::endl;
  // ddY_robot = ddY + (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot-Y) + Fdist_p );

  // DMP orient simulation
  arma::vec Q_c = cmd_args.a_py * quatLog(quatProd(Q_robot, quatInv(Q)));
  arma::vec eta_c = arma::vec(Do).zeros();

  X = arma::vec(Do).fill(x);
  dX = arma::vec(Do).zeros();

  std::vector<arma::vec> statesDot_o;
  statesDot_o = dmpOrient->getStatesDot(X, Q, eta, Q0, Qg, Q_c, eta_c);
  deta = statesDot_o[0];
  dQ = statesDot_o[1];
  // dX = statesDot_o[2];

  arma::vec v_rot_temp = 2 * quatProd(dQ, quatInv(Q));
  v_rot = v_rot_temp.subvec(1, 3);

  dv_rot = deta / dmpOrient->get_v_scale();
  // dv_rot_robot = dv_rot + (1/cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - cmd_args.Kd_o*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist_o );

  // Goal filtering
  if (cmd_args.USE_GOAL_FILT)
  {
    dg_p = cmd_args.a_g * (Yg2 - Yg) / canClockPtr->getTau();
    dg_o = cmd_args.a_g * quatLog(quatProd(Qg2, quatInv(Qg))) / canClockPtr->getTau();
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
    double stop_coeff = 1 / (1 + 0.5 * cmd_args.a_px * arma::norm(Y_robot - Y) + 0.5 * cmd_args.a_px * arma::norm(quatLog(quatProd(Q_robot, quatInv(Q)))));

    dx = dx * stop_coeff;
    dg_p = dg_p * stop_coeff;
    dg_o = dg_o * stop_coeff;
  }

  // Numerical integration
  x = x + dx * Ts;
  Y = Y + dY * Ts;
  Z = Z + dZ * Ts;
  Yg = Yg + dg_p * Ts;

  // std::cout << "------------------------" <<   std::endl ;
  // std::cout << "Y: " << Y.t() <<  std::endl ;
  // std::cout << "dY: " << dY.t() <<  std::endl ;

  Q = quatProd(quatExp(v_rot * Ts), Q);

  eta = eta + deta * Ts;
  Qg = quatProd(quatExp(dg_o * Ts), Qg);
}

void DMP_UR10_controller::logDemoStep()
{
  //  std::cout << "Log Demo step\n";
  trainData.Time = join_horiz(trainData.Time, t);
  trainData.Y_data = join_horiz(trainData.Y_data, Y_robot);
  trainData.dY_data = join_horiz(trainData.dY_data, dY_robot);
  trainData.ddY_data = join_horiz(trainData.ddY_data, ddY_robot);
  trainData.Q_data = join_horiz(trainData.Q_data, Q_robot);
  trainData.v_rot_data = join_horiz(trainData.v_rot_data, v_rot_robot);
  trainData.dv_rot_data = join_horiz(trainData.dv_rot_data, dv_rot_robot);
}

void DMP_UR10_controller::logOnline()
{
  //  std::cout << "Log online";
  // logDemoStep();

  log_data.Time = join_horiz(log_data.Time, t);

  log_data.y_data = join_horiz(log_data.y_data, arma::join_vert(Y, math_::quat2qpos(Q)));

  log_data.dy_data = join_horiz(log_data.dy_data, arma::join_vert(dY, v_rot));
  log_data.z_data = join_horiz(log_data.z_data, arma::join_vert(Z, eta));
  log_data.dz_data = join_horiz(log_data.dz_data, arma::join_vert(dZ, deta));

  log_data.x_data = join_horiz(log_data.x_data, x);

  log_data.y_robot_data = join_horiz(log_data.y_robot_data, arma::join_vert(Y_robot, math_::quat2qpos(Q_robot)));
  log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, arma::join_vert(dY_robot, v_rot_robot));
  log_data.ddy_robot_data = join_horiz(log_data.ddy_robot_data, arma::join_vert(ddY_robot, dv_rot_robot));

  log_data.Fdist_data = join_horiz(log_data.Fdist_data, arma::join_vert(Fdist_p, Fdist_o));
  log_data.g_data = join_horiz(log_data.g_data, arma::join_vert(Yg, math_::quat2qpos(Qg)));
}

void DMP_UR10_controller::logOffline()
{
  arma::rowvec Time_offline_train;
  arma::mat F_p_offline_train_data;
  arma::mat Fd_p_offline_train_data;
  arma::mat F_o_offline_train_data;
  arma::mat Fd_o_offline_train_data;

  arma::vec y0 = trainData.Y_data.col(0);
  arma::vec g = trainData.Y_data.col(trainData.n_data - 1);
  F_p_offline_train_data.resize(Dp, trainData.n_data);
  Fd_p_offline_train_data.resize(Dp, trainData.n_data);
  F_o_offline_train_data.resize(Do, trainData.n_data);
  Fd_o_offline_train_data.resize(Do, trainData.n_data);

  for (int j = 0; j < trainData.Time.size(); j++) {
      arma::vec X = dmpCartPos->phase(trainData.Time(j));
      F_p_offline_train_data.col(j) = dmpCartPos->learnedForcingTerm(X, y0, g);
      Fd_p_offline_train_data.col(j) = dmpCartPos->calcFd(X, trainData.Y_data.col(j), trainData.dY_data.col(j), trainData.ddY_data.col(j), y0, g);

      X = dmpOrient->phase(trainData.Time(j));
      F_o_offline_train_data.col(j) = dmpOrient->learnedForcingTerm(X, Q0, Qg);
      Fd_o_offline_train_data.col(j) = dmpOrient->calcFd(X, trainData.Q_data.col(j), trainData.v_rot_data.col(j), trainData.dv_rot_data.col(j), Q0, Qg);
  }
  Time_offline_train = trainData.Time;

  log_data.poseDataFlag = true;

  log_data.DMP_w.resize(D);
  log_data.DMP_c.resize(D);
  log_data.DMP_h.resize(D);
  for (int i = 0; i < D; i++) {
      log_data.DMP_w[i] = dmp[i]->w;
      log_data.DMP_c[i] = dmp[i]->c;
      log_data.DMP_h[i] = dmp[i]->h;
  }

  log_data.Time_demo = trainData.Time;
  arma::mat yd_data(Do, trainData.n_data);
  for (int i = 0; i < trainData.n_data; i++) {
      yd_data.col(i) = quat2qpos(trainData.Q_data.col(i));
  }
  log_data.yd_data = arma::join_vert(trainData.Y_data, yd_data);
  log_data.dyd_data = arma::join_vert(trainData.dY_data, trainData.v_rot_data);
  log_data.ddyd_data = arma::join_vert(trainData.ddY_data, trainData.dv_rot_data);

  log_data.D = D;
  log_data.Ts = Ts;
  log_data.g0 = arma::join_vert(Yg0, quat2qpos(Q0));

  log_data.Time_offline_train = Time_offline_train;
  log_data.F_offline_train_data = arma::join_vert(F_p_offline_train_data, F_o_offline_train_data);
  log_data.Fd_offline_train_data = arma::join_vert(Fd_p_offline_train_data, Fd_o_offline_train_data);
  log_data.Psi_data_train.resize(D);

  for (int i = 0; i < D; i++) {
      int n_data = Time_offline_train.size();
      log_data.Psi_data_train[i].resize(dmp[i]->N_kernels, n_data);
      for (int j = 0; j < n_data; j++) {
          double x = canClockPtr->getPhase(Time_offline_train(j));
          log_data.Psi_data_train[i].col(j) = dmp[i]->kernelFunction(x);
      }
  }

  log_data.Psi_data.resize(D);
  y0 = arma::join_vert(Y0, -quatLog(quatProd(Qg0, quatInv(Q0))));
  arma::vec g0 = arma::join_vert(Yg0, arma::vec(Do).zeros());
  for (int i = 0; i < D; i++) {
      int n_data = log_data.Time.size();
      log_data.Psi_data[i].resize(dmp[i]->N_kernels, n_data);
      log_data.Force_term_data.resize(D, n_data);
      for (int j = 0; j < n_data; j++) {
          double x = log_data.x_data(j);
          log_data.Psi_data[i].col(j) = dmp[i]->kernelFunction(x);
          log_data.Force_term_data(i, j) = dmp[i]->learnedForcingTerm(x, y0(i), g0(i));
      }
  }

  log_data.goalAttr_data.resize(log_data.Time.size());
  log_data.shapeAttr_data.resize(log_data.Time.size());
  for (int j = 0; j < log_data.Time.size(); j++) {
      double x = log_data.x_data(j);
      log_data.goalAttr_data(j) = dmp[0]->goalAttrGating(x);
      log_data.shapeAttr_data(j) = dmp[0]->shapeAttrGating(x);
  }
}

void DMP_UR10_controller::clearTrainData()
{
  trainData.n_data = 0;
  trainData.Time.clear();
  trainData.Y_data.clear();
  trainData.dY_data.clear();
  trainData.ddY_data.clear();
  trainData.Q_data.clear();
  trainData.v_rot_data.clear();
  trainData.dv_rot_data.clear();
}

void DMP_UR10_controller::clearLoggedData()
{
  log_data.clear();
}

void DMP_UR10_controller::saveAndRestartDemo()
{
  save_logged_data();
  clearAndRestartDemo();
}

void DMP_UR10_controller::clearAndRestartDemo()
{
  clearTrainData();
  clearLoggedData();
  initController();
}

void DMP_UR10_controller::save_logged_data()
{
  logOffline();

  std::ostringstream o_str;
  o_str << "";
  if (demo_save_counter > 0) o_str << demo_save_counter + 1;

  std::string suffix = cmd_args.DMP_TYPE + "_" + tm_::getTimeStamp();

  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  std::cout << "Saving to \"" << cmd_args.out_data_filename + o_str.str() <<"\" ...\n";
  log_data.save(cmd_args.out_data_filename + o_str.str(), true);
  std::cout << "Saving to \"" << cmd_args.out_data_filename + suffix <<"\" ...\n";
  log_data.save(cmd_args.out_data_filename + suffix, true);
  //log_data.save(cmd_args.out_data_filename + o_str.str(), false, 10);
  std::cout << "[DONE]!\n";

  demo_save_counter++;
}


void DMP_UR10_controller::execute()
{
  restart_demo_label:

  // ======  Read training data  =======
  recordDemo();

  // ======  Get DMP  =======
  get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  std::vector<std::shared_ptr<DMP_>> dmpVec1(Dp);
  for (int i = 0; i < Dp; i++) dmpVec1[i] = dmp[i];
  dmpCartPos.reset(new DMP_CartPos());
  dmpCartPos->init(dmpVec1);

  std::vector<std::shared_ptr<DMP_>> dmpVec2(Do);
  for (int i = 0; i < Do; i++) dmpVec2[i] = dmp[Dp + i];
  dmpOrient.reset(new DMP_orient());
  dmpOrient->init(dmpVec2);

  restart_dmp_execution_label:

  run_dmp = false;
  log_on = false;

  gotoStartPose();

  if (train_dmp)
  {
    train_DMP_thread.reset(new std::thread(&DMP_UR10_controller::trainDMP, this));
    train_DMP_thread->detach();
  }

  while (train_dmp) robotWait();


  while (!run_dmp) robotWait();

  while (save_exec_results) robotWait();

  clearLoggedData();
  //clearTrainData();

  initProgramVariables();

  // Start simulation
  while (ros::ok() && robot_->isOk() && !stop_robot)
  {
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

    if (run_dmp)
    {
      executeDMP();
      // std::cout << "------Execute DMP ------ " << std::endl ;
    }

    if (goto_start) goto restart_dmp_execution_label;

    if (arma::norm(Fee) > cmd_args.F_norm_retrain_thres) train_dmp = true;

    update();
    command();

    if (run_dmp)
    {
      // Stopping criteria
      double err_p = arma::max(arma::abs(Yg2 - Y_robot));
      double err_o = 0*arma::norm(quatLog(quatProd(Qg, quatInv(Q_robot))));
      //std::cout << "err_p !!:  " << err_p <<std::endl;
      //std::cout << "err_o !!:  " << err_o <<std::endl;
      if (err_p <= cmd_args.tol_stop && err_o <= cmd_args.orient_tol_stop && t >= tau)
      {
        goto_start = true;
        std::cout << io_::cyan << "Target reached. Moving to start pose...\n" << io_::reset;
      }
    }

  }
}

void DMP_UR10_controller::update()
{
  t = t + Ts;

  q_robot = robot_->getJointPosition();
  dq_robot = robot_->getJointVelocity();
  T_robot_ee = robot_->getTaskPose();
  V_robot = robot_->getTaskVelocity();

  //J_robot = robot_->getJacobian();
  Fee = robot_->getTaskWrench(); // Need to invert the sign of Fee ??????
  // Fee = -Fee;
  // Fee.swap_rows(3,5);

  arma::vec Fee_sign = arma::sign(Fee);
  //  std::cout<<"Fee in update() 1: "<< Fee.t();
  //  std::cout<<"Fee_sign in update(): "<< Fee_sign.t();
  //  std::cout<<"F_dead_zone in update(): "<< F_dead_zone.t();
  for(int i=0;i<6;i++){
    Fee(i) = (Fee_sign(i)) * fmax(0.0,fabs(Fee(i)) - F_dead_zone(i));
  }
       // std::cout<<"Fee in update() 2: "<< Fee.t();
//        Fee = Fee - Fee_sign % F_dead_zone;
//        Fee = 0.5 * (arma::sign(Fee) + Fee_sign) % Fee;

  Fdist_p =  Fee.rows(0,2);
  Fdist_o =  Fee.rows(3,5);

  // dq_robot = (q_robot - q_prev_robot) / Ts;
  // V_robot = J_robot * dq_robot;

  Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  dY_robot = V_robot.subvec(0, 2);
  ddY_robot = (dY_robot - dY_robot_prev) / Ts;

  Q_robot = math_::rotm2quat(T_robot_ee.submat(0, 0, 2, 2));
  v_rot_robot = V_robot.subvec(3, 5);
  dv_rot_robot = (v_rot_robot - v_rot_robot_prev) / Ts;

  q_prev_robot = q_robot;
  dY_robot_prev = dY_robot;
  v_rot_robot_prev = v_rot_robot;
}

void DMP_UR10_controller::command()
{
  double Kp = cmd_args.Kd_p;
  double Ko = cmd_args.Kd_o;
  double  factor_force = 0;

  // set the stiffness to a low value when the dmp is inactive so that the
  // user can move the robot freely
  if (!run_dmp) {
      Kp = 0.0;
      Ko = 0.0;
      factor_force = 1.0;
  }



  //ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * (dY_robot - dY) - Kp * (Y_robot - Y) + factor_force*Fdist_p);
  ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * dEp - Kp * Ep + factor_force*Fdist_p);




  //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot - v_rot) - Ko * quatLog(quatProd(Q_robot, quatInv(Q))) + factor_force*Fdist_o);
  //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot) + factor_force*Fdist_o);
  ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * dEo + factor_force*Fdist_o);



  dEp = dEp + ddEp * Ts;
  dEo = dEo + ddEo * Ts;
  Ep = Ep + dEp * Ts;


  arma::vec Vd(6);
  Vd.subvec(0, 2) = (dEp + dY - 4.0*(Y_robot - (Y + Ep)));
  Vd.subvec(3, 5) = (dEo + v_rot - 4.0*quatLog( quatProd( Q_robot, quatInv(Q) ) ) );


  if(run_dmp || goto_start){
    // if(robot_->mode != ur10_::Mode::VELOCITY_CONTROL){
    //   robot_->setMode(ur10_::Mode::VELOCITY_CONTROL);
    // }
    Vd.rows(3,5) = arma::zeros<arma::vec>(3);
    // arma::vec qd = arma::pinv(J_robot) * Vd;
    robot_->setTaskVelocity(Vd);
  }else{
    // if(robot_->mode != ur10_::Mode::TORQUE_CONTROL){
    //   robot_->setMode(ur10_::Mode::TORQUE_CONTROL);
    // }

    robot_->freedrive_mode();
    // arma::vec torque_for_rotation = arma::zeros<arma::vec>(7);
      //  torque_for_rotation = - (J_robot.rows(3,5)).t() * 5.0* quatLog(quatProd(Q_robot, quatInv(trainData.Q_data.col(0))));
    // robot_->setJointTorque(torque_for_rotation);
    //std::cout<<"torque_for_rotation: " << torque_for_rotation.t() <<std::endl;
  }
  robot_->waitNextCycle();

}

void DMP_UR10_controller::finalize()
{
    //save_logged_data();
    keyboard_ctrl_thread->join();

    // calc_simulation_mse();
    // std::cout << "offline_train_p_mse = \n" << offline_train_p_mse << "\n";
    // std::cout << "offline_train_o_mse = \n" << offline_train_o_mse << "\n";
    // std::cout << "sim_mse = \n" << sim_mse << "\n";
}