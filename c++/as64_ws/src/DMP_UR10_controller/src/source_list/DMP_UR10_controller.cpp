#include <DMP_UR10_controller.h>
#include <io_lib/io_lib.h>
#include <time_lib/time.h>
#include <plot_lib/plot_lib.h>

using namespace as64_::math_;
using namespace as64_;

DMP_UR10_controller::DMP_UR10_controller(std::shared_ptr<ur10_::Robot> robot)
{
  using namespace as64_;

  modeName.resize(6);
  modeName[0] = "STOP";
  modeName[1] = "IDLE";
  modeName[2] = "FREEDRIVE";
  modeName[3] = "MODEL_RUN";
  modeName[4] = "POSITION_CONTROL";
  modeName[5] = "ADMITTANCE_CONTROL";

  Dp = 3;
  Do = 3;
  D = Dp + Do;

  robot_ = robot;
  Ts = robot_->cycle;

  std::cout << io_::bold << io_::green << "Reading params from yaml file..." << io_::reset << "\n";
  cmd_args.parse_cmd_args();
  cmd_args.print();

  Q_robot << 1 << 0 << 0 << 0; // initialize it for use in getClosestQuat

  demo_save_counter = 0;

  initController();

  model_trained = false;

  keyboard_ctrl_thread.reset(new std::thread(&DMP_UR10_controller::keyboardCtrlThreadFun, this));
}

void DMP_UR10_controller::initControlFlags()
{
  // pause_robot = false;
  // stop_robot = false;
  // run_dmp = false;
  this->setMode(DMP_UR10_controller::Mode::IDLE);

  save_exec_results = false;
  log_on = false;

  train_model = false;
  goto_start = false;
  record_demo_on = false;
  clear_rerecord_demo_on = false;
  save_rerecord_demo_on = false;
  start_pose_set = false;
  run_model_in_loop = false;
  print_robot_state = false;
}

void DMP_UR10_controller::initModelExecution()
{
  T_robot_ee = robot_->getTaskPose();
  Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  Q_robot = math_::rotm2quat(T_robot_ee.submat(0, 0, 2, 2));

  t = 0.0;
  x = 0.0;
  dx = 0.0;

  Yg = cmd_args.goal_scale * Yg0; // Yg0 is initialized only after "recordDemo"
  Yg2 = Yg;
  dg_p = arma::vec().zeros(Dp);
  // Y = Y0;
  Y = Y_robot; // Y0
  // Y0 = Y_robot;

  // std::cout << "Y0 = " << Y.t() << "\n";
  // std::cout << "Y0 = " << Y0.t() << "\n";
  //
  // std::cout << "q0 = " << trainData.q0.t() << "\n";
  // std::cout << "q0 = " << robot_->getJointPosition().t() << "\n";

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

  ddEp = arma::zeros<arma::vec>(3);
  dEp = arma::zeros<arma::vec>(3);
  Ep =  arma::zeros<arma::vec>(3);
  ddEo =  arma::zeros<arma::vec>(3);
  dEo =  arma::zeros<arma::vec>(3);
  Eo =  arma::zeros<arma::vec>(3);

}

void DMP_UR10_controller::trainModel()
{
  model_trained = false;

  tau = trainData.Time(trainData.n_data - 1);

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

  // param_::ParamList trainParamList;
  // trainParamList.setParam("lambda", cmd_args.lambda);
  // trainParamList.setParam("P_cov", cmd_args.P_cov);

  using namespace io_;
  std::cout << yellow << bold << "Training DMP...\n" << reset;

  //std::cout << io_::bold << io_::green << "DMP CartPos training..." << io_::reset << "\n";
  trainData.n_data = trainData.Time.size();
  arma::vec y0 = trainData.Y_data.col(0);
  arma::vec g = trainData.Y_data.col(trainData.n_data - 1);
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

  train_model = false;

  std::cout << "offline_train_p_mse: " << offline_train_p_mse.t() <<  "\n";
  std::cout << "offline_train_o_mse: " <<  offline_train_o_mse.t() <<  "\n";

  model_trained = true;
}

void DMP_UR10_controller::runModel()
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

  ddY = dZ / dmpCartPos->get_v_scale();

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

  Q = quatProd(quatExp(v_rot * Ts), Q);

  eta = eta + deta * Ts;
  Qg = quatProd(quatExp(dg_o * Ts), Qg);
}

void DMP_UR10_controller::initController()
{
  initControlFlags();
  initModelExecution();
}


void DMP_UR10_controller::printHelpMenu() const
{
  using namespace as64_::io_;

  #define PRINT_KEY_ID_MACRO(key_id) reset << cyan << bold << key_id << reset
  #define PRINT_KEY_FUNCTION_MACRO(key_fun) reset << cyan << underline << key_fun << reset

  std::cout << blue << bold;
  std::cout << "********************************\n";
  std::cout << "*********  Help Menu  **********\n";
  std::cout << "********************************\n";
  //std::cout << cyan << bold;
  std::cout << PRINT_KEY_ID_MACRO("\"spacebar\"   : ") << PRINT_KEY_FUNCTION_MACRO("run model\n");
  std::cout << PRINT_KEY_ID_MACRO("     o       : ") << PRINT_KEY_FUNCTION_MACRO("run model in loop\n");
  std::cout << PRINT_KEY_ID_MACRO("     q       : ") << PRINT_KEY_FUNCTION_MACRO("Go to start pose\n");
  std::cout << PRINT_KEY_ID_MACRO("     l       : ") << PRINT_KEY_FUNCTION_MACRO("Enable/Disable data logging during model execution\n");
  std::cout << PRINT_KEY_ID_MACRO("     v       : ") << PRINT_KEY_FUNCTION_MACRO("Save model execution results\n");
  std::cout << PRINT_KEY_ID_MACRO("     i       : ") << PRINT_KEY_FUNCTION_MACRO("Enter IDLE mode\n");
  std::cout << PRINT_KEY_ID_MACRO("     f       : ") << PRINT_KEY_FUNCTION_MACRO("Enter FREEDRIVE mode\n");
  std::cout << PRINT_KEY_ID_MACRO("     a       : ") << PRINT_KEY_FUNCTION_MACRO("Enter ADMITTANCE mode\n");
  std::cout << PRINT_KEY_ID_MACRO("     r       : ") << PRINT_KEY_FUNCTION_MACRO("Start/Stop demo\n");
  std::cout << PRINT_KEY_ID_MACRO("     b       : ") << PRINT_KEY_FUNCTION_MACRO("Load training data from file\n");
  std::cout << PRINT_KEY_ID_MACRO("     y       : ") << PRINT_KEY_FUNCTION_MACRO("Print keys\n");
  std::cout << PRINT_KEY_ID_MACRO("     t       : ") << PRINT_KEY_FUNCTION_MACRO("Print robot state\n");
  std::cout << reset;

  #undef PRINT_KEY_ID_MACRO
  #undef PRINT_KEY_FUNCTION_MACRO
}

void DMP_UR10_controller::keyboardCtrlThreadFun()
{
  using namespace as64_::io_;

  int key = 0;
  while (this->getMode() != DMP_UR10_controller::Mode::STOP)
  {
    key = std::tolower(as64_::io_::getch());

    //std::cout << "Pressed " << (char)key  << "\n";
    std::cout << green << bold  << "[KEY PRESSED]: " << (char)key << "\n";

    switch (key)
    {
      case 'a':
        if (this->getMode() != DMP_UR10_controller::Mode::IDLE)
        {
          std::cout << yellow << bold << "[WARNING]: Cannot set mode to \"ADMITTANCE_CONTROL\"\n";
          std::cout << "Current mode is \"" << getModeName() << "\"\n";
          std::cout << "Wait or set mode to \"IDLE\" by pressing \"i\".";
        }
        else
        {
          this->setMode(DMP_UR10_controller::Mode::ADMITTANCE_CONTROL);
        }
        break;
      case 'o':
        run_model_in_loop = true;
      case 32:
        if (this->getMode() != DMP_UR10_controller::Mode::IDLE)
        {
          std::cout << yellow << bold << "[WARNING]: Cannot set mode to \"MODEL_RUN\"\n";
          std::cout << "Current mode is \"" << getModeName() << "\"\n";
          std::cout << "Wait or set mode to \"IDLE\" by pressing \"i\".";
        }
        else
        {
          if (model_trained)
          {
            this->setMode(DMP_UR10_controller::Mode::MODEL_RUN);
            std::cout << green << bold << "Run model\n";
          }
          else
          {
            std::cout << yellow << green << "[WARNING]: Cannot run model. The model has not been trained.\n";
          }
        }
        break;
      case 'q':
        if (this->getMode() != DMP_UR10_controller::Mode::IDLE)
        {
          std::cout << yellow << bold << "[WARNING]: Cannot go to start pose (change mode to \"POSITION_CONTROL\")\n";
          std::cout << "Current mode is \"" << getModeName() << "\"\n";
          std::cout << "Wait or set mode to \"IDLE\" by pressing \"i\".\n";
        }
        else
        {
          this->setMode(DMP_UR10_controller::Mode::POSITION_CONTROL);
          goto_start = true;
        }
        break;
      case 'l':
        if (save_exec_results && !log_on)
        {
          std::cout << yellow << bold << "[WARNING]: Cannot enable \"log on\"!\n";
          std::cout << "Reason: save execution results is active...\n";
        }
        else
        {
          log_on = !log_on;
          std::cout << "Log on " << (log_on?"enabled":"disabled") << "\n";
        }
        break;
      case 's':
        // stop_robot = true;
        this->setMode(DMP_UR10_controller::Mode::STOP);
        std::cout << "Stop robot\n";
        break;
      case 'r':
        if (this->getMode() != DMP_UR10_controller::Mode::FREEDRIVE)
        {
          std::cout << yellow << bold << "WARNING: Cannot start demo.\n";
          std::cout << "Set the mode first to \"FREEDRIVE\" by pressing \"f\".\n";
        }
        else{
          record_demo_on = !record_demo_on;
          std::cout << (record_demo_on?"Start":"Stop")<< " demo recording\n";
        }
        break;
      case 'v':
        std::cout << "Saving execution results\n";
        if (log_on)
        {
          std::cout << yellow << bold << "[WARNING]: Cannot save execution results!\n";
          std::cout << "You have to disable \"log on\" first...\n";
        }
        else
        {
          save_exec_results = true;
          save_exec_results_thread.reset(new std::thread(&DMP_UR10_controller::saveExecutionResults, this));
          save_exec_results_thread->detach();
        }
        break;
      case 'f':
        this->setMode(DMP_UR10_controller::Mode::FREEDRIVE);
        break;
      case 'b':
        if (this->getMode() != DMP_UR10_controller::Mode::IDLE)
        {
          std::cout << yellow << bold << "[WARNING]: Change mode to IDLE and then press \"b\" to load the train data.\n";
        }
        else
        {
          std::cout << "Using training data from file\n";
          loadDataAndTrainModel();
        }
        break;
      case 'h':
        this->printHelpMenu();
        break;
      case 'y':
        this->printKeys();
        break;
      case 'i':
        // pause_robot = !pause_robot;
        this->setMode(DMP_UR10_controller::Mode::IDLE);
        //std::cout << "Setting mode to \"IDLE\"\n";
        break;
      case 't':
        print_robot_state = !print_robot_state;
        if (print_robot_state) robot_->launch_printRobotStateThread(1);
        else robot_->stop_printRobotStateThread();
        break;
      default:
        std::cout << bold << yellow << "[WARNING]: Unrecognized key...\n";
    }
    std::cout << reset;
  }
}

void DMP_UR10_controller::printKeys() const
{
  using namespace as64_::io_;

  std::cout << bold << blue;
  std::cout << "==============================\n";
  std::cout << "====== Flags/keys status =====\n";
  std::cout << "==============================\n";
  std::cout << bold << cyan;
  std::cout << "==> Mode              : " << getModeName() << "\n";
  std::cout << "==> record_demo_on    : " << (record_demo_on?"true":"false") << "\n";
  std::cout << "==> log_on            : " << (log_on?"true":"false") << "\n";
  std::cout << "==> save_exec_results : " << (save_exec_results?"true":"false") << "\n";
  std::cout << "==> train_model       : " << (train_model?"true":"false") << "\n";
  std::cout << "==> model_trained     : " << (model_trained?"true":"false") << "\n";
  std::cout << "==> goto_start        : " << (goto_start?"true":"false") << "\n";
  std::cout << "==> run_model_in_loop : " << (run_model_in_loop?"true":"false") << "\n";
  std::cout << "==> print_robot_state : " << (print_robot_state?"true":"false") << "\n";
  std::cout << reset;
}

void DMP_UR10_controller::saveExecutionResults()
{
  using namespace io_;

  saveLoggedData();
  save_exec_results = false;
}

void DMP_UR10_controller::robotWait()
{
  switch (this->getMode())
  {
    case IDLE:
      //robot_->stopj(3.14);
      robot_->sleep(robot_->cycle);
      break;
    case FREEDRIVE:
      // do nothing
      break;
    case MODEL_RUN:
      robot_->setJointPosition(robot_->getJointPosition());
      break;
    case POSITION_CONTROL:
      robot_->setJointPosition(robot_->getJointPosition());
      break;
    case ADMITTANCE_CONTROL:
      robot_->setJointPosition(robot_->getJointPosition());
      break;
  }
}

void DMP_UR10_controller::recordDemo()
{
  train_model = false;
  initModelExecution(); // to zero all velocities and accelerations and set all poses/joints to the current ones
  clearTrainData();

  update();

  t = 0.0;
  q0_robot = q_robot; // save intial joint positions
  while (record_demo_on)
  {
    logDemoStep();
    update();
  }
  trainData.q0 = q0_robot;
  trainData.n_data = trainData.Time.size();

  start_pose_set = true;

  record_demo_on = false;
  train_model = true; // the DMP must be trained now with the demo data

  Dp = trainData.dY_data.n_rows;
  Do = trainData.v_rot_data.n_rows;
  D = Dp + Do;

  // save initial pose
  Y0 = trainData.Y_data.col(0);
  Q0 = trainData.Q_data.col(0);

  // save goal-target pose
  Yg0 = trainData.Y_data.col(trainData.n_data - 1);
  Qg0 = trainData.Q_data.col(trainData.n_data - 1);

  saveDemoData();

  train_model_thread.reset(new std::thread(&DMP_UR10_controller::trainModel, this));
  train_model_thread->detach();
}

void DMP_UR10_controller::loadDataAndTrainModel()
{
  train_data_loaded = false;

  std::thread thr(&DMP_UR10_controller::loadDemoData, this);
  thr.detach();

  while (!train_data_loaded)
  {
    update();
    robotWait();
  }

  Dp = trainData.dY_data.n_rows;
  Do = trainData.v_rot_data.n_rows;
  D = Dp + Do;

  // save initial pose
  Y0 = trainData.Y_data.col(0);
  Q0 = trainData.Q_data.col(0);

  // save goal-target pose
  Yg0 = trainData.Y_data.col(trainData.n_data - 1);
  Qg0 = trainData.Q_data.col(trainData.n_data - 1);

  model_trained = false;
  thr = std::thread(&DMP_UR10_controller::trainModel, this);
  thr.detach();

  while (!model_trained)
  {
    update();
    robotWait();
  }
}

void DMP_UR10_controller::saveDemoData()
{
  using namespace as64_;
  std::string train_data_save_file = cmd_args.data_output_path + "/" + cmd_args.demo_data_filename;
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

  io_::write_mat(trainData.q0, out, binary);

  out.close();
}


void DMP_UR10_controller::loadDemoData()
{
  train_data_loaded = false;

  using namespace as64_;
  std::string train_data_load_file = cmd_args.data_output_path + "/" + cmd_args.demo_data_filename;
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

  train_data_loaded = true;
  start_pose_set = true;
}

void DMP_UR10_controller::gotoStartPose()
{
  using namespace as64_::io_;

  goto_start = false;

  if (!start_pose_set)
  {
      std::cout << yellow << bold << "[WARNING]: No starting pose has been set...\n";
      std::cout << "Record a demo to register a starting pose.\n" << reset;
  }
  else
  {
    std::cout << bold << cyan << "Moving to start pose...\n" << reset;

    double duration = std::max(arma::max(arma::abs(trainData.q0-q_robot))*6.5/arma::datum::pi,2.0);
    robot_->setJointTrajectory(trainData.q0, duration);

    std::cout << bold << cyan << "Reached start pose!\n" << reset;
  }

}


void DMP_UR10_controller::logDemoStep()
{
  //  std::cout << "Log Demo step\n";
  trainData.Time = arma::join_horiz(trainData.Time, arma::mat{t});
  trainData.Y_data = arma::join_horiz(trainData.Y_data, Y_robot);
  trainData.dY_data = arma::join_horiz(trainData.dY_data, dY_robot);
  trainData.ddY_data = arma::join_horiz(trainData.ddY_data, ddY_robot);
  trainData.Q_data = arma::join_horiz(trainData.Q_data, Q_robot);
  trainData.v_rot_data = arma::join_horiz(trainData.v_rot_data, v_rot_robot);
  trainData.dv_rot_data = arma::join_horiz(trainData.dv_rot_data, dv_rot_robot);
}

void DMP_UR10_controller::logOnline()
{
  //  std::cout << "Log online";
  // logDemoStep();

  log_data.Time = arma::join_horiz(log_data.Time, arma::mat({t}));

  log_data.y_data = arma::join_horiz(log_data.y_data, arma::join_vert(Y, math_::quat2qpos(Q)));

  log_data.dy_data = arma::join_horiz(log_data.dy_data, arma::join_vert(dY, v_rot));
  log_data.z_data = arma::join_horiz(log_data.z_data, arma::join_vert(Z, eta));
  log_data.dz_data = arma::join_horiz(log_data.dz_data, arma::join_vert(dZ, deta));

  log_data.x_data = arma::join_horiz(log_data.x_data, arma::mat({x}));

  log_data.y_robot_data = arma::join_horiz(log_data.y_robot_data, arma::join_vert(Y_robot, math_::quat2qpos(Q_robot)));
  log_data.dy_robot_data = arma::join_horiz(log_data.dy_robot_data, arma::join_vert(dY_robot, v_rot_robot));
  log_data.ddy_robot_data = arma::join_horiz(log_data.ddy_robot_data, arma::join_vert(ddY_robot, dv_rot_robot));

  log_data.Fdist_data = arma::join_horiz(log_data.Fdist_data, arma::join_vert(Fdist_p, Fdist_o));
  log_data.g_data = arma::join_horiz(log_data.g_data, arma::join_vert(Yg, math_::quat2qpos(Qg)));
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

void DMP_UR10_controller::saveLoggedData()
{
  logOffline();

  std::ostringstream o_str;
  o_str << "";
  if (demo_save_counter > 0) o_str << demo_save_counter + 1;

  std::string suffix = cmd_args.DMP_TYPE + "_" + tm_::getTimeStamp();

  log_data.cmd_args = cmd_args;
  // std::cout << "Saving to \"" << cmd_args.out_data_filename + o_str.str() <<"\" ...\n";
  // log_data.save(cmd_args.out_data_filename + o_str.str(), true);
  std::cout << "Saving to \"" << cmd_args.out_data_filename + suffix <<"\" ...\n";
  log_data.save(cmd_args.out_data_filename + suffix, true);
  //log_data.save(cmd_args.out_data_filename + o_str.str(), false, 10);
  std::cout << "[DONE]!\n";

  demo_save_counter++;
}


void DMP_UR10_controller::execute()
{
  bool model_exec_init = false;
  bool  exit_ctrl_loop = false;
  bool freedrive_mode_set = false;
  bool  admittance_ctrl_init = false;

  // Start simulation
  while (ros::ok() && robot_->isOk() && !exit_ctrl_loop)
  {
    //ros::spinOnce();
    update();

    switch (this->getMode())
    {
      case STOP:
        exit_ctrl_loop = true;
        break;
      case IDLE:
        model_exec_init = false;
        freedrive_mode_set = false;
        run_model_in_loop = false;
        admittance_ctrl_init = false;
        robotWait();
        break;
      case FREEDRIVE:
        if (!freedrive_mode_set)
        {
          robot_->freedrive_mode();
          freedrive_mode_set = true;
        }
        if (record_demo_on) recordDemo();
        robotWait();
        break;
      case MODEL_RUN:
        while (train_model) robotWait();
        if (!model_exec_init){
          if (arma::norm(q_robot-trainData.q0) > 0.5e-2)
          {
            gotoStartPose();
            update();
          }
          while (save_exec_results) robotWait();
          initModelExecution();
          if (log_on) clearLoggedData();
          model_exec_init = true;
        }
        if (log_on) logOnline();
        runModel();
        command();
        if (targetReached())
        {
          if (!run_model_in_loop) this->setMode(IDLE);
          model_exec_init = false;
        }
        break;
      case POSITION_CONTROL:
        if (goto_start)
        {
          gotoStartPose();
          this->setMode(IDLE);
        }
        break;
      case ADMITTANCE_CONTROL:
        if (admittance_ctrl_init = false)
        {
          initAdmittanceController();
          admittance_ctrl_init = true;
        }
        admittanceControl();
        break;
    }
  }
  this->setMode(STOP);
}

void DMP_UR10_controller::update()
{
  robot_->waitNextCycle();

  t = t + Ts;

  q_robot = robot_->getJointPosition();
  dq_robot = robot_->getJointVelocity();
  T_robot_ee = robot_->getTaskPose();
  V_robot = robot_->getTaskVelocity();

  //J_robot = robot_->getJacobian();
  Fee = robot_->getTaskWrench(); // Need to invert the sign of Fee ??????

  // std::cout << "1) Fee = " << Fee.t() << "\n";

  arma::vec Fee_sign = arma::sign(Fee);
  for(int i=0;i<6;i++)
  {
    Fee(i) = (Fee_sign(i)) * fmax(0.0,fabs(Fee(i)) - cmd_args.Fee_dead_zone(i));
  }
  // Fee = Fee - Fee_sign % Fee_dead_zone;
  // Fee = 0.5 * (arma::sign(Fee) + Fee_sign) % Fee;

  // std::cout << "2) Fee = " << Fee.t() << "\n";

  Fdist_p =  Fee.rows(0,2);
  Fdist_o =  Fee.rows(3,5);

  // dq_robot = (q_robot - q_prev_robot) / Ts;
  // V_robot = J_robot * dq_robot;

  Y_robot = T_robot_ee.submat(0, 3, 2, 3);
  dY_robot = V_robot.subvec(0, 2);
  ddY_robot = (dY_robot - dY_robot_prev) / Ts;

  arma::vec Q_robot_prev = Q_robot;
  Q_robot = math_::rotm2quat(T_robot_ee.submat(0, 0, 2, 2));
  Q_robot = math_::getClosestQuat(Q_robot, Q_robot_prev);

  v_rot_robot = V_robot.subvec(3, 5);
  dv_rot_robot = (v_rot_robot - v_rot_robot_prev) / Ts;

  q_prev_robot = q_robot;
  dY_robot_prev = dY_robot;
  v_rot_robot_prev = v_rot_robot;
}

void DMP_UR10_controller::command()
{
  double  factor_force = 0.0;

  //ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * (dY_robot - dY) - Kp * (Y_robot - Y) + factor_force*Fdist_p);
  ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * dEp - cmd_args.Kd_p * Ep + factor_force*Fdist_p);

  //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot - v_rot) - Ko * quatLog(quatProd(Q_robot, quatInv(Q))) + factor_force*Fdist_o);
  //ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * (v_rot_robot) + factor_force*Fdist_o);
  ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * dEo - cmd_args.Kd_o * Eo + factor_force*Fdist_o);

  dEp = dEp + ddEp * Ts;
  Ep = Ep + dEp * Ts;

  dEo = dEo + ddEo * Ts;
  Eo = Eo + dEo * Ts;

  arma::vec Vd(6);
  Vd.subvec(0, 2) = (dEp + dY - 4.0*(Y_robot - (Y + Ep)));
  Vd.subvec(3, 5) = v_rot; //(dEo + v_rot - 4.0*quatLog( quatProd( Q_robot, quatInv(Q) ) ) );

  Vd.rows(3,5) = arma::zeros<arma::vec>(3);
  robot_->setTaskVelocity(Vd);

}

void DMP_UR10_controller::admittanceControl()
{
  ddEp = (1.0 / cmd_args.Md_p) * (- cmd_args.Dd_p * dEp - cmd_args.Kd_p * Ep + Fdist_p);

  ddEo = (1.0 / cmd_args.Md_o) * (- cmd_args.Dd_o * dEo - cmd_args.Kd_o * Eo + Fdist_o);

  dEp = dEp + ddEp * Ts;
  Ep = Ep + dEp * Ts;

  dEo = dEo + ddEo * Ts;
  Eo = Eo + dEo * Ts;

  arma::vec Vd(6);
  Vd.subvec(0, 2) = dEp;
  Vd.subvec(3, 5) = dEo;

  // if (arma::norm(Vd) > 0.00001)
  // {
  //   std::cout << bold << red << "[DANGER]: Too high speed!\n";
  //   std::cout << "Vd = " << Vd.t() << "\n";
  //   std::cout << "Fdist_p = " << Fdist_p.t() << "\n";
  //   std::cout << reset;
  //   Vd.fill(0.0);
  // }

  //Vd.rows(3,5) = arma::zeros<arma::vec>(3);
  // arma::vec qd = arma::pinv(J_robot) * Vd;
  robot_->setTaskVelocity(Vd);

}

void DMP_UR10_controller::initAdmittanceController()
{
  ddEp = arma::zeros<arma::vec>(3);
  dEp = arma::zeros<arma::vec>(3);
  Ep =  arma::zeros<arma::vec>(3);
  ddEo =  arma::zeros<arma::vec>(3);
  dEo =  arma::zeros<arma::vec>(3);
  Eo =  arma::zeros<arma::vec>(3);
}

void DMP_UR10_controller::finalize()
{
    if (keyboard_ctrl_thread->joinable()) keyboard_ctrl_thread->join();
}

DMP_UR10_controller::Mode DMP_UR10_controller::getMode() const
{
  return this->mode;
}

void DMP_UR10_controller::setMode(const DMP_UR10_controller::Mode &mode)
{
  this->mode = mode;
  std::cout << as64_::io_::green << as64_::io_::bold;
  std::cout << "Mode changed to \"" << getModeName() << "\"\n";
  std::cout << as64_::io_::reset;
}

std::string DMP_UR10_controller::getModeName() const
{
  return modeName[this->getMode()];
}

bool DMP_UR10_controller::targetReached() const
{
  double err_p = arma::max(arma::abs(Yg2 - Y_robot));
  double err_o = 0*arma::norm(quatLog(quatProd(Qg, quatInv(Q_robot))));
  //std::cout << "err_p !!:  " << err_p <<std::endl;
  //std::cout << "err_o !!:  " << err_o <<std::endl;
  if (err_p <= cmd_args.pos_tol_stop && err_o <= cmd_args.orient_tol_stop && t >= tau)
  {
    std::cout << as64_::io_::cyan << "Target reached!\n" << as64_::io_::reset;
    return true;
  }
  return false;
}
