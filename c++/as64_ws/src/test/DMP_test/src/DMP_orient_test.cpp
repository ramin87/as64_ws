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

int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_test");
  ros::NodeHandle nh("~");
  arma::wall_clock timer;

  // std::string package_path = ros::package::getPath("dmp_test");
  // std::cout << "package_path = " << package_path << "\n";

  // ======  Read parameters from config file  =======
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading params from yaml file..." << as64_::io_::reset << "\n";
  CMD_ARGS cmd_args;
  cmd_args.parse_cmd_args();
  cmd_args.print();

  // ======  Read training data  =======
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading training data..." << as64_::io_::reset << "\n";
  arma::rowvec Time_demo;
  arma::mat Qd_data;
  arma::mat v_rot_d_data;
  arma::mat dv_rot_d_data;
  load_data(cmd_args.in_orient_data_filename, Qd_data, v_rot_d_data, dv_rot_d_data, Time_demo, cmd_args.binary);

  // as64_::plot_::GraphObj gObj;
  // gObj.hold_on();
  // for (int i=0;i<3;i++)
  // {
  //   arma::rowvec v = v_rot_d_data.row(i);
  //   gObj.plot(Time_demo, v);
  // }
  // gObj.title("Rotational velocities");
  // gObj.xlabel("time [s]");
  // gObj.ylabel("velocity [rad/s]");
  // gObj.hold_off();
  // as64_::io_::wait_for_key();

  int n_data = Time_demo.size();
  int D = v_rot_d_data.n_rows;
  double Ts = arma::min(arma::diff(Time_demo));
  Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
  double tau = Time_demo(n_data-1);

  // ======  Get DMP  =======
  std::shared_ptr<as64_::CanonicalClock> canClockPtr;
  std::shared_ptr<as64_::GatingFunction> shapeAttrGatingPtr;
  std::shared_ptr<as64_::GatingFunction> goalAttrGatingPtr;
  std::vector<std::shared_ptr<as64_::DMP_>> dmp;
  get_canClock_gatingFuns_DMP(cmd_args, D, tau, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, dmp);

  std::shared_ptr<as64_::DMP_orient> dmpOrient(new as64_::DMP_orient());
  dmpOrient->init(dmp);

  for (int i=0;i<D;i++)
  {
    std::cout << "DMP " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";
  }
  std::cout << "n_data = " << n_data << "\n";

  // ======  Train the DMP  =======
  arma::mat F_offline_train_data(D, n_data);
  arma::mat Fd_offline_train_data(D, n_data);
  arma::rowvec Time_offline_train;
  arma::vec offline_train_mse(D);

  as64_::param_::ParamList trainParamList;
  trainParamList.setParam("trainMethod", cmd_args.trainMethod);
  trainParamList.setParam("lambda", cmd_args.lambda);
  trainParamList.setParam("P_cov", cmd_args.P_cov);

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP training..." << as64_::io_::reset << "\n";
  arma::vec Q0 = Qd_data.col(0);
  arma::vec Qg = Qd_data.col(n_data-1);
  dmpOrient->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_mse= dmpOrient->train(Time_demo, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg);
  std::cout << "Elapsed time is " << timer.toc() << "\n";

  for (int j=0; j<Time_demo.size(); j++)
  {
    arma::vec X = dmpOrient->phase(Time_demo(j));
    F_offline_train_data.col(j) = dmpOrient->learnedForcingTerm(X, Q0, Qg);
    Fd_offline_train_data.col(j) = dmpOrient->calcFd(X, Qd_data.col(j), v_rot_d_data.col(j), dv_rot_d_data.col(j), Q0, Qg);
  }
  Time_offline_train = Time_demo;

  // ======  Simulate DMP  =======
  // Set initial conditions
  Q0 = Qd_data.col(0);
  arma::vec Qg0 = cmd_args.goal_scale*Qd_data.col(n_data-1);
  Qg = Qg0;
  arma::vec Qg2 = Qg0;
  arma::vec dg = arma::vec().zeros(D);
  double x = 0.0;
  double dx = 0.0;
  arma::vec Q = Q0;
  arma::vec v_rot = arma::vec().zeros(D);
  arma::vec dv_rot = arma::vec().zeros(D);
  double t = 0.0;
  arma::vec Q_robot = Q0;
  arma::vec v_rot_robot = arma::vec().zeros(D);
  arma::vec dv_rot_robot = arma::vec().zeros(D);
  arma::vec deta = arma::vec().zeros(D);
  arma::vec eta = arma::vec().zeros(D);

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

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP simulation..." << as64_::io_::reset << "\n";
  timer.tic();
  // Start simulation
  while (true)
  {
    // ========= data logging =============
    log_data.Time = join_horiz(log_data.Time, t);

    log_data.y_data = join_horiz(log_data.y_data, as64_::quat2qpos(Q));

    log_data.dy_data = join_horiz(log_data.dy_data, v_rot);
    log_data.z_data = join_horiz(log_data.z_data, eta);
    log_data.dz_data = join_horiz(log_data.dz_data, deta);

    log_data.x_data = join_horiz(log_data.x_data, x);

    log_data.y_robot_data = join_horiz(log_data.y_robot_data, Q_robot);
    log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, v_rot_robot);
    log_data.ddy_robot_data = join_horiz(log_data.ddy_robot_data, dv_rot_robot);

    log_data.Fdist_data = join_horiz(log_data.Fdist_data, Fdist);
    log_data.g_data = join_horiz(log_data.g_data, as64_::quat2qpos(Qg));

    // DMP simulation
    arma::vec Q_c = cmd_args.a_py*as64_::quatLog(as64_::quatProd(Q_robot,as64_::quatInv(Q)));
    arma::vec eta_c = arma::vec(D).zeros();

    arma::vec X = arma::vec(D).fill(x);
    arma::vec dX = arma::vec(D).zeros();

    arma::mat statesDot;
    statesDot = dmpOrient->getStatesDot(X, Q, eta, Q0, Qg, Q_c, eta_c);
    dz = statesDot.col(0);
    dy = statesDot.col(1);
    // dX = statesDot.col(2);

    ddy = dz/dmpOrient->get_v_scale();
    ddy_robot = ddy + (1/cmd_args.Md) * ( - cmd_args.Dd*(dy_robot - dy) - cmd_args.Kd*(y_robot-y) + Fdist );

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

  log_data.Time_demo = Time_demo;
  //log_data.yd_data = Qd_data;
  log_data.dyd_data = v_rot_d_data;
  log_data.ddyd_data = dv_rot_d_data;

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

  // ========  Save results  ===========
  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_orient_data_filename, true);
  log_data.save(cmd_args.out_orient_data_filename, false, 10);
  std::cout << "[DONE]!\n";

  arma::vec sim_mse(D);
  for (int i=0;i<D;i++)
  {
    arma::rowvec y_robot_data2;
    arma::rowvec Qd_data2;
    as64_::spl_::makeSignalsEqualLength(log_data.Time, log_data.y_robot_data.row(i),
            log_data.Time_demo, log_data.yd_data.row(i), log_data.Time,
            y_robot_data2, Qd_data2);
    sim_mse(i) = arma::norm(y_robot_data2-Qd_data2); // /Qd_data2.size();
  }

  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "sim_mse = \n" << sim_mse << "\n";



  ros::shutdown();

  return 0;
}
