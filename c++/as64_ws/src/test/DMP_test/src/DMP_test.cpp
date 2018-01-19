/**
 * Copyright (C) 2017 as64
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

#include <utils.h>
#include <cmd_args.h>

int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_test");
  ros::NodeHandle nh("~");

  // std::string package_path = ros::package::getPath("dmp_test");
  // std::cout << "package_path = " << package_path << "\n";

  // ======  Read parameters from config file  =======
  std::cout << as64::io_::bold << as64::io_::green << "Reading params from yaml file..." << as64::io_::reset << "\n";
  CMD_ARGS cmd_args;
  cmd_args.parse_cmd_args();
  cmd_args.print();
  cmd_args.in_data_filename = cmd_args.data_input_path + "/" + cmd_args.in_data_filename;
  cmd_args.out_data_filename = cmd_args.data_output_path + cmd_args.out_data_filename;
  if (cmd_args.USE_PHASE_STOP == false) cmd_args.a_py = 0.0;

  // ======  Read training data  =======
  std::cout << as64::io_::bold << as64::io_::green << "Reading training data..." << as64::io_::reset << "\n";
  arma::rowvec Time_demo;
  arma::mat yd_data;
  arma::mat dyd_data;
  arma::mat ddyd_data;
  load_data(cmd_args.in_data_filename, yd_data, dyd_data, ddyd_data, Time_demo, cmd_args.binary);

  int n_data = Time_demo.size();
  int D = yd_data.n_rows;
  double Ts = arma::min(arma::diff(Time_demo));
  Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
  double tau = Time_demo(n_data-1);

  std::cout << "number_of_kernels = " << cmd_args.N_kernels << "\n";
  std::cout << "n_data = " << n_data << "\n";

  // ======  Get DMP  =======
  std::shared_ptr<as64::CanonicalClock> canClockPtr;
  std::shared_ptr<as64::GatingFunction> shapeAttrGatingPtr;
  std::shared_ptr<as64::GatingFunction> goalAttrGatingPtr;
  std::vector<std::shared_ptr<as64::DMP_>> dmp;
  get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  // ======  Train the DMP  =======
  arma::mat F_offline_train_data(D, n_data);
  arma::mat Fd_offline_train_data(D, n_data);
  arma::rowvec Time_offline_train;
  arma::vec offline_train_mse(D);

  as64::param_::ParamList trainParamList;
  trainParamList.setParam("trainMethod", cmd_args.trainMethod);
  trainParamList.setParam("lambda", cmd_args.lambda);
  trainParamList.setParam("P_cov", cmd_args.P_cov);

  std::cout << as64::io_::bold << as64::io_::green << "DMP training..." << as64::io_::reset << "\n";
  for (int i=0;i<D;i++)
  {
    arma::vec y0 = yd_data.col(0);
    arma::vec g = yd_data.col(n_data-1);

    dmp[i]->setTrainingParams(&trainParamList);
    offline_train_mse(i) = dmp[i]->train(Time_demo, yd_data.row(i), dyd_data.row(i), ddyd_data.row(i), y0(i), g(i));

    for (int j=0; j<Time_demo.size(); j++)
    {
      double x = dmp[i]->phase(Time_demo(j));
      F_offline_train_data(i,j) = dmp[i]->learnedForcingTerm(x, y0(i), g(i));
      Fd_offline_train_data(i,j) = dmp[i]->calcFd(x, yd_data(i,j), dyd_data(i,j), ddyd_data(i,j), y0(i), g(i));
    }
  }
  Time_offline_train = Time_demo;

  // ======  Simulate DMP  =======
  // Set initial conditions
  arma::vec y0 = yd_data.col(0);
  arma::vec g0 = cmd_args.goal_scale*yd_data.col(n_data-1);
  arma::vec g = g0;
  arma::vec g2 = g0;
  arma::vec dg = arma::vec().zeros(D);
  double x = 0.0;
  double dx = 0.0;
  arma::vec y = y0;
  arma::vec dy = arma::vec().zeros(D);
  arma::vec ddy = arma::vec().zeros(D);
  double t = 0.0;
  arma::vec y_robot = y0;
  arma::vec dy_robot = arma::vec().zeros(D);
  arma::vec ddy_robot = arma::vec().zeros(D);
  arma::vec dz = arma::vec().zeros(D);
  arma::vec z = arma::vec().zeros(D);

  arma::vec F = arma::vec().zeros(D);
  arma::vec Fd = arma::vec().zeros(D);

  arma::vec Fdist = arma::vec().zeros(D);

  // create log_data struct
  LogData log_data;

  log_data.DMP_w.resize(D);
  log_data.DMP_c.resize(D);
  log_data.DMP_h.resize(D);
  for (int i=0;i<D;i++)
  {
    log_data.DMP_w[i] = dmp[i]->w;
    log_data.DMP_c[i] = dmp[i]->c;
    log_data.DMP_h[i] = dmp[i]->h;
  }

  double tau0 = canClockPtr->getTau();
  tau = cmd_args.tau_sim_scale*tau;
  canClockPtr->setTau(tau);

  int iters = 0;
  double dt = cmd_args.dt;

  std::cout << as64::io_::bold << as64::io_::green << "DMP simulation..." << as64::io_::reset << "\n";
  // Start simulation
  while (true)
  {
    // ========= data logging =============
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
      ddy_robot(i) = ddy(i) + (1/cmd_args.Md) * ( - cmd_args.Dd*(dy_robot(i) - dy(i)) - cmd_args.Kd*(y_robot(i)-y(i)) + Fdist(i) );
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
        std::cout << as64::io_::yellow << as64::io_::bold << "Iteration limit reached. Stopping simulation...\n";
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
    log_data.Psi_data_train[i].resize(cmd_args.N_kernels,n_data);
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
    log_data.Psi_data[i].resize(cmd_args.N_kernels,n_data);
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
    log_data.goalAttr_data(j) = dmp[0]->shapeAttrGating(x);
    log_data.shapeAttr_data(j) = dmp[0]->goalAttrGating(x);
  }

  // ========  Save results  ===========
  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_data_filename, true);
  log_data.save(cmd_args.out_data_filename, false, 10);
  std::cout << "[DONE]!\n";

  arma::vec sim_mse(D);
  for (int i=0;i<D;i++)
  {
    arma::rowvec y_robot_data2;
    arma::rowvec yd_data2;
    as64::spl_::makeSignalsEqualLength(log_data.Time, log_data.y_robot_data.row(i),
            log_data.Time_demo, log_data.yd_data.row(i), log_data.Time,
            y_robot_data2, yd_data2);
    sim_mse(i) = arma::norm(y_robot_data2-yd_data2); // /yd_data2.size();
  }

  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "sim_mse = \n" << sim_mse << "\n";



  ros::shutdown();

  return 0;
}
