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
#include <math_lib/math_lib.h>

#include <utils.h>
#include <cmd_args.h>

using namespace as64_;
using namespace as64_::math_;

class DMP_CartPos_orient_test
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
  std::shared_ptr<as64_::DMP_orient> dmpOrient;

  int n_data;
  arma::rowvec Time_demo;
  arma::mat Yd_data, dYd_data, ddYd_data;
  arma::mat Qd_data, v_rot_d_data, dv_rot_d_data;

  double Ts;
  double tau;
  int Dp;
  int Do;
  int D;

  arma::rowvec Time_offline_train;
  arma::mat F_p_offline_train_data;
  arma::mat Fd_p_offline_train_data;
  arma::vec offline_train_p_mse;
  arma::mat F_o_offline_train_data;
  arma::mat Fd_o_offline_train_data;
  arma::vec offline_train_o_mse;

  int iters;
  double t;
  double dt;
  double x, dx;

  arma::vec Yg0, Yg, Yg2, dg_p;
  arma::vec Y0, Y, dY, ddY;
  arma::vec Y_robot, dY_robot, ddY_robot;
  arma::vec Z, dZ;
  arma::vec Fdist_p;

  arma::vec Qg0, Qg, Qg2, dg_o;
  arma::vec Q0, Q, dQ, v_rot, dv_rot;
  arma::vec Q_robot, v_rot_robot, dv_rot_robot;
  arma::vec eta, deta;
  arma::vec Fdist_o;

  arma::vec F, Fd;

  arma::vec sim_mse;
};

void DMP_CartPos_orient_test::read_train_data()
{
  std::cout << as64_::io_::bold << as64_::io_::green << "Reading CartPos training data..." << as64_::io_::reset << "\n";
  load_data(cmd_args.in_CartPos_data_filename, Yd_data, dYd_data, ddYd_data, Time_demo, cmd_args.binary);

  std::cout << as64_::io_::bold << as64_::io_::green << "Reading orient training data..." << as64_::io_::reset << "\n";
  load_data(cmd_args.in_orient_data_filename, Qd_data, v_rot_d_data, dv_rot_d_data, Time_demo, cmd_args.binary);

  n_data = Time_demo.size();
  Dp = dYd_data.n_rows;
  Do = v_rot_d_data.n_rows;
  D = Dp+Do;
  Ts = arma::min(arma::diff(Time_demo));
  Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
  tau = Time_demo(n_data-1);
}

void DMP_CartPos_orient_test::train()
{
  // as64_::param_::ParamList trainParamList;
  // trainParamList.setParam("lambda", cmd_args.lambda);
  // trainParamList.setParam("P_cov", cmd_args.P_cov);

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP CartPos training..." << as64_::io_::reset << "\n";
  arma::vec y0 = Yd_data.col(0);
  arma::vec g = Yd_data.col(n_data-1);
  // dmpCartPos->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_p_mse= dmpCartPos->train(Time_demo, Yd_data, dYd_data, ddYd_data, y0, g, cmd_args.trainMethod, true);
  std::cout << "Elapsed time is " << timer.toc() << "\n";

  std::cout << as64_::io_::bold << as64_::io_::green << "DMP orient training..." << as64_::io_::reset << "\n";
  arma::vec Q0 = Qd_data.col(0);
  arma::vec Qg = Qd_data.col(n_data-1);
  // dmpOrient->setTrainingParams(&trainParamList);
  timer.tic();
  offline_train_o_mse= dmpOrient->train(Time_demo, Qd_data, v_rot_d_data, dv_rot_d_data, Q0, Qg, cmd_args.trainMethod);
  std::cout << "Elapsed time is " << timer.toc() << "\n";
}

void DMP_CartPos_orient_test::init_simulation()
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
  Fdist_p = arma::vec().zeros(Dp);

  Q0 = Qd_data.col(0);
  Qg0 = cmd_args.goal_scale*Qd_data.col(n_data-1);
  Qg = Qg0;
  Qg2 = Qg0;
  dg_o = arma::vec().zeros(Do);
  Q = Q0;
  v_rot = arma::vec().zeros(Do);
  dv_rot = arma::vec().zeros(Do);
  Q_robot = Q0;
  v_rot_robot = arma::vec().zeros(Do);
  dv_rot_robot = arma::vec().zeros(Do);
  deta = arma::vec().zeros(Do);
  eta = arma::vec().zeros(Do);
  Fdist_o = arma::vec().zeros(Do);

  F = arma::vec().zeros(Do);
  Fd = arma::vec().zeros(Do);

  double tau0 = canClockPtr->getTau();
  tau = cmd_args.tau_sim_scale*tau;
  canClockPtr->setTau(tau);

  iters = 0;
  dt = cmd_args.dt;
}

void DMP_CartPos_orient_test::log_online()
{
  log_data.Time = join_horiz(log_data.Time, t);

  log_data.y_data = join_horiz( log_data.y_data, arma::join_vert(Y, quat2qpos(Q)) );

  log_data.dy_data = join_horiz( log_data.dy_data, arma::join_vert(dY, v_rot) );
  log_data.z_data = join_horiz( log_data.z_data, arma::join_vert(Z, eta) );
  log_data.dz_data = join_horiz( log_data.dz_data, arma::join_vert(dZ, deta) );

  log_data.x_data = join_horiz(log_data.x_data, x);

  log_data.y_robot_data = join_horiz( log_data.y_robot_data, arma::join_vert(Y_robot, quat2qpos(Q_robot)) );
  log_data.dy_robot_data = join_horiz( log_data.dy_robot_data, arma::join_vert(dY_robot, v_rot_robot) );
  log_data.ddy_robot_data = join_horiz( log_data.ddy_robot_data, arma::join_vert(ddY_robot, dv_rot_robot) );

  log_data.Fdist_data = join_horiz( log_data.Fdist_data, arma::join_vert(Fdist_p, Fdist_o) );
  log_data.g_data = join_horiz( log_data.g_data, arma::join_vert(Yg, quat2qpos(Qg)) );
}

void DMP_CartPos_orient_test::log_offline()
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

void DMP_CartPos_orient_test::save_logged_data()
{
  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_data_filename, true);
  log_data.save(cmd_args.out_data_filename, false, 10);
  std::cout << "[DONE]!\n";
}

void DMP_CartPos_orient_test::calc_simulation_mse()
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

void DMP_CartPos_orient_test::run()
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

  std::vector<std::shared_ptr<as64_::DMP_>> dmpVec1(Dp);
  for (int i=0;i<Dp;i++) dmpVec1[i] = dmp[i];
  dmpCartPos.reset(new as64_::DMP_CartPos());
  dmpCartPos->init(dmpVec1);

  std::vector<std::shared_ptr<as64_::DMP_>> dmpVec2(Do);
  for (int i=0;i<Do;i++) dmpVec2[i] = dmp[Dp+i];
  dmpOrient.reset(new as64_::DMP_orient());
  dmpOrient->init(dmpVec2);

  // ======  Train the DMP  =======
  train();

  for (int i=0;i<Dp;i++)
    std::cout << "DMP CartPos " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";

  for (int i=0;i<Do;i++)
    std::cout << "DMP orient " << i+1 << ": number_of_kernels = " << dmp[i]->N_kernels << "\n";

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
    ddY_robot = ddY + (1/cmd_args.Md_p) * ( - cmd_args.Dd_p*(dY_robot - dY) - cmd_args.Kd_p*(Y_robot-Y) + Fdist_p );

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
    dv_rot_robot = dv_rot + (1/cmd_args.Md_o) * ( - cmd_args.Dd_o*(v_rot_robot - v_rot) - cmd_args.Kd_o*quatLog(quatProd(Q_robot,quatInv(Q))) + Fdist_o );

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

    // Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
    {
      //Fdist_p = Fdist_fun(t, Dp);
      //Fdist_o = Fdist_fun(t, Do);
    }

    // Phase stopping
    if (cmd_args.USE_PHASE_STOP)
    {
        double stop_coeff = 1/(1+0.5*cmd_args.a_px*arma::norm(Y_robot - Y) + 0.5*cmd_args.a_px*arma::norm(quatLog(quatProd(Q_robot,quatInv(Q)))));

        dx = dx*stop_coeff;
        dg_p = dg_p*stop_coeff;
        dg_o = dg_o*stop_coeff;
    }

    // Stopping criteria
    double err_p = arma::max(arma::abs(Yg2-Y_robot));
    double err_o = arma::norm(quatLog(quatProd(Qg,quatInv(Q_robot))));
    if (err_p <= cmd_args.tol_stop &&
      err_o <= cmd_args.orient_tol_stop && t>=tau)
    {
        break;
    }

    // std::cout << "err_p = " << err_p << "\n";
    // std::cout << "err_o = " << err_o << "\n";

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

    Q = quatProd(quatExp(v_rot*dt), Q);
    eta = eta + deta*dt;
    Q_robot = quatProd(quatExp(v_rot_robot*dt),Q_robot);
    v_rot_robot = v_rot_robot + dv_rot_robot*dt;
    Qg = quatProd(quatExp(dg_o*dt),Qg);

  }

  std::cout << "Elapsed time is " << timer.toc() << "\n";

  log_offline();

  // ========  Save results  ===========
  save_logged_data();

  calc_simulation_mse();

  std::cout << "offline_train_p_mse = \n" << offline_train_p_mse << "\n";
  std::cout << "offline_train_o_mse = \n" << offline_train_o_mse << "\n";
  std::cout << "sim_mse = \n" << sim_mse << "\n";
}


int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_CartPos_orient_test");
  ros::NodeHandle nh("~");

  DMP_CartPos_orient_test test;

  test.run();

  ros::shutdown();

  return 0;
}
