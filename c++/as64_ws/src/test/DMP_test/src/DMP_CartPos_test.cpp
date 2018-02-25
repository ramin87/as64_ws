/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <dmp_lib/dmp_lib.h>
#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>
#include <sigproc_lib/sigproc_lib.h>
#include <plot_lib/plot_lib.h>

#include <utils.h>
#include <cmd_args.h>

class DMP_CartPos_test
{
public:
  void run();

  void read_train_data();

  void train();

  void init_simulation();
  void run_simulation();
  void log_online();
  void log_offline();
  void save_logged_data();
  void calc_simulation_mse();
private:
  arma::wall_clock timer;

  CMD_ARGS cmd_args;
  LogData log_data;

  std::shared_ptr<as64_::CanonicalClock> canClockPtr;
  std::shared_ptr<as64_::GatingFunction> shapeAttrGatingPtr;
  std::shared_ptr<as64_::GatingFunction> goalAttrGatingPtr;
  std::vector<std::shared_ptr<as64_::DMP_>> dmp;
  std::shared_ptr<as64_::DMP_CartPos> dmpCartPos;

  int n_data;
  arma::rowvec Time_demo;
  arma::mat Yd_data, dYd_data, ddYd_data;

  double Ts;
  double tau;
  int Dp;

  arma::mat F_p_offline_train_data;
  arma::mat Fd_p_offline_train_data;
  arma::rowvec Time_offline_train;
  arma::vec offline_train_p_mse;

  int iters;
  double t;
  double dt;
  double x, dx;
  arma::vec Yg0, Yg, Yg2, dg_p;
  arma::vec Y0, Y, dY, ddY;
  arma::vec Y_robot, dY_robot, ddY_robot;
  arma::vec Z, dZ;
  arma::vec F, Fd, Fdist_p;
  arma::vec sim_mse;
};

void DMP_CartPos_test::read_train_data()
{
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading training data..." << as64_::io_::reset << "\n";

  load_data(cmd_args.in_CartPos_data_filename, Yd_data, dYd_data, ddYd_data, Time_demo, cmd_args.binary);

  n_data = Time_demo.size();
  Dp = Yd_data.n_rows;
  Ts = arma::min(arma::diff(Time_demo));
  Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
  tau = Time_demo(n_data-1);
}

void DMP_CartPos_test::train()
{
  // as64_::param_::ParamList trainParamList;
  // trainParamList.setParam("lambda", cmd_args.lambda);
  // trainParamList.setParam("P_cov", cmd_args.P_cov);

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP training..." << as64_::io_::reset << "\n";
  Y0 = Yd_data.col(0);
  Yg = Yd_data.col(n_data-1);
  // dmpCartPos->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_p_mse= dmpCartPos->train(Time_demo, Yd_data, dYd_data, ddYd_data, Y0, Yg, cmd_args.trainMethod, true);
  std::cout << "Elapsed time is " << timer.toc() << "\n";
}

void DMP_CartPos_test::init_simulation()
{
  t = 0.0;
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
  Y_robot = Y0;
  dY_robot = arma::vec().zeros(Dp);
  ddY_robot = arma::vec().zeros(Dp);
  dZ = arma::vec().zeros(Dp);
  Z = arma::vec().zeros(Dp);

  F = arma::vec().zeros(Dp);
  Fd = arma::vec().zeros(Dp);
  Fdist_p = arma::vec().zeros(Dp);

  double tau0 = canClockPtr->getTau();
  tau = cmd_args.tau_sim_scale*tau;
  canClockPtr->setTau(tau);

  iters = 0;
  dt = cmd_args.dt;
}

void DMP_CartPos_test::log_online()
{
  log_data.Time = join_horiz(log_data.Time, t);

  log_data.y_data = join_horiz(log_data.y_data, Y);

  log_data.dy_data = join_horiz(log_data.dy_data, dY);
  log_data.z_data = join_horiz(log_data.z_data, Z);
  log_data.dz_data = join_horiz(log_data.dz_data, dZ);

  log_data.x_data = join_horiz(log_data.x_data, x);

  log_data.y_robot_data = join_horiz(log_data.y_robot_data, Y_robot);
  log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, dY_robot);
  log_data.ddy_robot_data = join_horiz(log_data.ddy_robot_data, ddY_robot);

  log_data.Fdist_data = join_horiz(log_data.Fdist_data, Fdist_p);
  log_data.g_data = join_horiz(log_data.g_data, Yg);
}

void DMP_CartPos_test::log_offline()
{
  F_p_offline_train_data.resize(Dp, n_data);
  Fd_p_offline_train_data.resize(Dp, n_data);
  for (int j=0; j<Time_demo.size(); j++)
  {
    arma::vec X = dmpCartPos->phase(Time_demo(j));
    F_p_offline_train_data.col(j) = dmpCartPos->learnedForcingTerm(X, Y0, Yg);
    Fd_p_offline_train_data.col(j) = dmpCartPos->calcFd(X, Yd_data.col(j), dYd_data.col(j), ddYd_data.col(j), Y0, Yg);
  }
  Time_offline_train = Time_demo;

  log_data.DMP_w.resize(Dp);
  log_data.DMP_c.resize(Dp);
  log_data.DMP_h.resize(Dp);
  for (int i=0;i<Dp;i++)
  {
    log_data.DMP_w[i] = dmp[i]->w;
    log_data.DMP_c[i] = dmp[i]->c;
    log_data.DMP_h[i] = dmp[i]->h;
  }

  log_data.Time_demo = Time_demo;
  log_data.yd_data = Yd_data;
  log_data.dyd_data = dYd_data;
  log_data.ddyd_data = ddYd_data;

  log_data.D = Dp;
  log_data.Ts = Ts;
  log_data.g0 = Yg0;

  log_data.Time_offline_train = Time_offline_train;
  log_data.F_offline_train_data = F_p_offline_train_data;
  log_data.Fd_offline_train_data = Fd_p_offline_train_data;
  log_data.Psi_data_train.resize(Dp);

  for (int i=0;i<Dp;i++)
  {
    int n_data = Time_offline_train.size();
    log_data.Psi_data_train[i].resize(dmp[i]->N_kernels,n_data);
    for (int j=0;j<n_data;j++)
    {
      double x = canClockPtr->getPhase(Time_offline_train(j));
      log_data.Psi_data_train[i].col(j) = dmp[i]->kernelFunction(x);
    }
  }

  log_data.Psi_data.resize(Dp);
  for (int i=0;i<Dp;i++)
  {
    int n_data = log_data.Time.size();
    log_data.Psi_data[i].resize(dmp[i]->N_kernels,n_data);
    log_data.Force_term_data.resize(Dp,n_data);
    for (int j=0;j<n_data;j++)
    {
      double x = log_data.x_data(j);
      log_data.Psi_data[i].col(j) = dmp[i]->kernelFunction(x);
      log_data.Force_term_data(i,j) = dmp[i]->learnedForcingTerm(x, Y0(i), Yg0(i));
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

void DMP_CartPos_test::save_logged_data()
{
  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_CartPos_data_filename, true);
  log_data.save(cmd_args.out_CartPos_data_filename, false, 10);
  std::cout << "[DONE]!\n";
}

void DMP_CartPos_test::calc_simulation_mse()
{
  sim_mse.resize(Dp);
  for (int i=0;i<Dp;i++)
  {
    arma::rowvec y_robot_data2;
    arma::rowvec yd_data2;
    as64_::spl_::makeSignalsEqualLength(log_data.Time, log_data.y_robot_data.row(i),
            log_data.Time_demo, log_data.yd_data.row(i), log_data.Time,
            y_robot_data2, yd_data2);
    sim_mse(i) = arma::norm(y_robot_data2-yd_data2); // /yd_data2.size();
  }
}

void DMP_CartPos_test::run()
{

  // std::string package_path = ros::package::getPath("dmp_test");
  // std::cout << "package_path = " << package_path << "\n";

  // ======  Read parameters from config file  =======
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading params from yaml file..." << as64_::io_::reset << "\n";
  cmd_args.parse_cmd_args();
  cmd_args.print();

  // ======  Read training data  =======
  read_train_data();

  // ======  Get DMP  =======
  get_canClock_gatingFuns_DMP(cmd_args, Dp, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  dmpCartPos.reset(new as64_::DMP_CartPos());
  dmpCartPos->init(dmp);

  // ======  Train the DMP  =======
  train();

  for (int i=0;i<Dp;i++)
  {
    std::cout << "DMP " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";
  }
  std::cout << "n_data = " << n_data << "\n";

  // ======  Simulate DMP  =======
  // Set initial conditions
  init_simulation();

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP simulation..." << as64_::io_::reset << "\n";
  timer.tic();
  // Start simulation
  while (true)
  {
    // ========= data logging =============
    log_online();

    // DMP simulation
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
    ddY_robot = ddY + (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot-Y) + Fdist_p );

    // Goal filtering
    if (cmd_args.USE_GOAL_FILT)
    {
      dg_p = cmd_args.a_g*(Yg2-Yg)/canClockPtr->getTau();
    }
    else
    {
      Yg = Yg2;
      dg_p.fill(0.0);
    }

    // Update phase variable

    dx = canClockPtr->getPhaseDot(x);

    // Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
    {
      // Fdist_p = Fdist_fun(t, D);
    }

    // Phase stopping
    if (cmd_args.USE_PHASE_STOP)
    {
        double stop_coeff = 1/(1+cmd_args.a_px*std::pow(arma::norm(Y-Y_robot),2));
        dx = dx*stop_coeff;
        dg_p = dg_p*stop_coeff;
    }

    // Stopping criteria
    double err_p = arma::max(arma::abs(Yg2-Y_robot));
    if (err_p <= cmd_args.tol_stop && t>=tau)
    {
        break;
    }

    iters++;

    if (t>=tau && iters>=cmd_args.max_iters)
    {
        std::cout << as64_::io_::yellow << as64_::io_::bold << "Iteration limit reached. Stopping simulation...\n";
        break;
    }

    // Numerical integration
    t = t + dt;

    x = x + dx*dt;

    Y = Y + dY*dt;
    Z = Z + dZ*dt;

    Y_robot = Y_robot + dY_robot*dt;
    dY_robot = dY_robot + ddY_robot*dt;

    Yg = Yg + dg_p*dt;

  }

  std::cout << "Elapsed time is " << timer.toc() << "\n";

  log_offline();

  // ========  Save results  ===========
  save_logged_data();

  calc_simulation_mse();

  std::cout << "offline_train_p_mse = \n" << offline_train_p_mse << "\n";
  std::cout << "sim_mse = \n" << sim_mse << "\n";
}


int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_CartPos_test");
  ros::NodeHandle nh("~");

  DMP_CartPos_test test;

  test.run();

  ros::shutdown();

  return 0;
}
