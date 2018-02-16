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

#include <DMP_lib/DMP_lib.h>
#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>
#include <signalProcessing_lib/signalProcessing_lib.h>
#include <plot_lib/plot_lib.h>

#include <utils.h>
#include <cmd_args.h>

class DMP_test
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

  int n_data;
  arma::rowvec Time_demo;
  arma::mat yd_data, dyd_data, ddyd_data;

  double Ts;
  double tau;
  int D;

  arma::mat F_offline_train_data;
  arma::mat Fd_offline_train_data;
  arma::rowvec Time_offline_train;
  arma::vec offline_train_mse;

  int iters;
  double t;
  double dt;
  double x, dx;
  arma::vec g0, g, g2, dg;
  arma::vec y0, y, dy, ddy;
  arma::vec y_robot, dy_robot, ddy_robot;
  arma::vec z, dz;
  arma::vec F, Fd, Fdist;
  arma::vec sim_mse;
};

void DMP_test::read_train_data()
{
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading training data..." << as64_::io_::reset << "\n";

  load_data(cmd_args.in_data_filename, yd_data, dyd_data, ddyd_data, Time_demo, cmd_args.binary);

  n_data = Time_demo.size();
  D = yd_data.n_rows;
  Ts = arma::min(arma::diff(Time_demo));
  Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
  tau = Time_demo(n_data-1);
}

void DMP_test::train()
{
  //as64_::param_::ParamList trainParamList;
  //trainParamList.setParam("lambda", cmd_args.lambda);
  //trainParamList.setParam("P_cov", cmd_args.P_cov);

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP training..." << as64_::io_::reset << "\n";
  y0 = yd_data.col(0);
  g = yd_data.col(n_data-1);

  offline_train_mse.resize(D);
  timer.tic();
  for (int i=0;i<D;i++)
  {
    //dmp[i]->setTrainingParams(&trainParamList);
    offline_train_mse(i) = dmp[i]->train(Time_demo, yd_data.row(i), dyd_data.row(i), ddyd_data.row(i), y0(i), g(i), cmd_args.trainMethod);
  }
  std::cout << "Elapsed time is " << timer.toc() << "\n";
}

void DMP_test::init_simulation()
{
  t = 0.0;
  x = 0.0;
  dx = 0.0;

  y0 = yd_data.col(0);
  g0 = cmd_args.goal_scale*yd_data.col(n_data-1);
  g = g0;
  g2 = g0;
  dg = arma::vec().zeros(D);
  y = y0;
  dy = arma::vec().zeros(D);
  ddy = arma::vec().zeros(D);
  y_robot = y0;
  dy_robot = arma::vec().zeros(D);
  ddy_robot = arma::vec().zeros(D);
  dz = arma::vec().zeros(D);
  z = arma::vec().zeros(D);

  F = arma::vec().zeros(D);
  Fd = arma::vec().zeros(D);
  Fdist = arma::vec().zeros(D);

  double tau0 = canClockPtr->getTau();
  tau = cmd_args.tau_sim_scale*tau;
  canClockPtr->setTau(tau);

  iters = 0;
  dt = cmd_args.dt;
}

void DMP_test::log_online()
{
  log_data.Time = join_horiz(log_data.Time, t);

  log_data.y_data = join_horiz(log_data.y_data, y);

  log_data.dy_data = join_horiz(log_data.dy_data, dy);
  log_data.z_data = join_horiz(log_data.z_data, z);
  log_data.dz_data = join_horiz(log_data.dz_data, dz);

  log_data.x_data = join_horiz(log_data.x_data, x);

  log_data.y_robot_data = join_horiz(log_data.y_robot_data, y_robot);
  log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, dy_robot);
  log_data.ddy_robot_data = join_horiz(log_data.ddy_robot_data, ddy_robot);

  log_data.Fdist_data = join_horiz(log_data.Fdist_data, Fdist);
  log_data.g_data = join_horiz(log_data.g_data, g);
}

void DMP_test::log_offline()
{
  F_offline_train_data.resize(D, n_data);
  Fd_offline_train_data.resize(D, n_data);
  for (int i=0;i<D;i++)
  {
    for (int j=0; j<Time_demo.size(); j++)
    {
      double x = dmp[i]->phase(Time_demo(j));
      F_offline_train_data(i,j) = dmp[i]->learnedForcingTerm(x, y0(i), g(i));
      Fd_offline_train_data(i,j) = dmp[i]->calcFd(x, yd_data(i,j), dyd_data(i,j), ddyd_data(i,j), y0(i), g(i));
    }
  }
  Time_offline_train = Time_demo;

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
  log_data.yd_data = yd_data;
  log_data.dyd_data = dyd_data;
  log_data.ddyd_data = ddyd_data;

  log_data.D = D;
  log_data.Ts = Ts;
  log_data.g0 = g0;

  log_data.Time_offline_train = Time_offline_train;
  log_data.F_offline_train_data = F_offline_train_data;
  log_data.Fd_offline_train_data = Fd_offline_train_data;
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

void DMP_test::save_logged_data()
{
  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_data_filename, true);
  log_data.save(cmd_args.out_data_filename, false, 10);
  std::cout << "[DONE]!\n";
}

void DMP_test::calc_simulation_mse()
{
  sim_mse.resize(D);
  for (int i=0;i<D;i++)
  {
    arma::rowvec y_robot_data2;
    arma::rowvec yd_data2;
    as64_::spl_::makeSignalsEqualLength(log_data.Time, log_data.y_robot_data.row(i),
            log_data.Time_demo, log_data.yd_data.row(i), log_data.Time,
            y_robot_data2, yd_data2);
    sim_mse(i) = arma::norm(y_robot_data2-yd_data2); // /yd_data2.size();
  }
}

void DMP_test::run()
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
  get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  // ======  Train the DMP  =======
  train();

  for (int i=0;i<D;i++)
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

    for (int i=0; i<D; i++)
    {
      double y_c = cmd_args.a_py*(y_robot(i)-y(i));
      double z_c = 0.0;

      arma::vec statesDot;
      statesDot = dmp[i]->getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);
      dz(i) = statesDot(0);
      dy(i) = statesDot(1);
      // dx = statesDot(2);

      ddy(i) = dz(i)/dmp[i]->get_v_scale();
      ddy_robot(i) = ddy(i) + (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dy_robot(i) - dy(i)) - cmd_args.Kd_p*(y_robot(i)-y(i)) + Fdist(i) );
    }

    // Goal filtering
    if (cmd_args.USE_GOAL_FILT)
    {
        dg = cmd_args.a_g*(g2-g)/canClockPtr->getTau();
    }
    else
    {
        g = g2;
        dg.fill(0.0);
    }

    // Update phase variable

    dx = canClockPtr->getPhaseDot(x);

    // Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
    {
      // Fdist = Fdist_fun(t, D);
    }

    // Phase stopping
    if (cmd_args.USE_PHASE_STOP)
    {
        double stop_coeff = 1/(1+cmd_args.a_px*std::pow(arma::norm(y-y_robot),2));
        dx = dx*stop_coeff;
        dg = dg*stop_coeff;
    }

    // Stopping criteria
    double err_p = arma::max(arma::abs(g2-y_robot));
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

    y = y + dy*dt;
    z = z + dz*dt;

    y_robot = y_robot + dy_robot*dt;
    dy_robot = dy_robot + ddy_robot*dt;

    g = g + dg*dt;

  }

  std::cout << "Elapsed time is " << timer.toc() << "\n";

  log_offline();

  // ========  Save results  ===========
  save_logged_data();

  calc_simulation_mse();

  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "sim_mse = \n" << sim_mse << "\n";
}


int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_test");
  ros::NodeHandle nh("~");

  DMP_test test;

  test.run();

  ros::shutdown();

  return 0;
}
