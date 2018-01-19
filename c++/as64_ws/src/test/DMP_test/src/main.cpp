/**
 * Copyright (C) 2017 as64
 */

#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <fstream>

#include <utils.h>

#include <DMP_lib/DMP_lib.h>


int main(int argc, char** argv)
{
  // ========  Initialize the ROS node  ===========
  ros::init(argc, argv, "DMP_test_node");
  ros::NodeHandle nh("~");

  arma::wall_clock timer;
  double elapsed_time;

  double tau = 2.3;
  as64::CanonicalClock canClockPtr = new(LinCanonicalClock());
  canClockPtr->init(tau);


/*
  // ========  initialize cmd params  ===========
  CMD_ARGS cmd_args;
  std::cout << "====> Parsing CMD args...\n";
  cmd_args.parse_cmd_args();
  std::string file_ext = cmd_args.binary?".bin":".txt";
  cmd_args.in_data_filename = cmd_args.data_input_path + cmd_args.in_data_filename + file_ext;
  cmd_args.out_data_filename = cmd_args.data_output_path + cmd_args.out_data_filename; // + file_ext;

  if (~cmd_args.USE_PHASE_STOP)
  {
    cmd_args.a_py = 0;
  }

  // ========  Load demos and process demos  ===========
  std::cout << "====> Loading demos and processing demos...\n";
  int D;
  int n_data;
  double Ts;
  arma::mat data;

  load_data(cmd_args.in_data_filename, data, Ts, cmd_args.binary);

  D = data.n_rows;

  arma::mat yd_data, dyd_data, ddyd_data;
  process_demos(data,Ts, yd_data, dyd_data, ddyd_data, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

  // std::cout << "Ts = " << Ts << "\n";
  // std::cout << "yd_data = " << yd_data << "\n";
  // std::cout << "dyd_data = " << dyd_data << "\n";
  // std::cout << "ddyd_data = " << ddyd_data << "\n";

  n_data = yd_data.n_cols; // number of points in each dimension
  arma::rowvec Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data) * Ts;

  // ========  Set up DMP params  ===========
  double tau = (n_data-1)*Ts;

  std::cout << "number_of_kernels = " << cmd_args.N_kernels << "\n";
  std::cout << "n_data = " << n_data << "\n";

  // set the Canonical System
  std::shared_ptr<as64::CanonicalSystem> can_sys_ptr;
  bool USE_2nd_order_can_sys = false;

  if (cmd_args.CAN_SYS_TYPE.compare("exp") == 0)
  {
    can_sys_ptr.reset(new as64::ExpCanonicalSystem());
  }
  else if (cmd_args.CAN_SYS_TYPE.compare("lin") == 0)
  {
    can_sys_ptr.reset(new as64::LinCanonicalSystem());
  }
  else if (cmd_args.CAN_SYS_TYPE.compare("spring-damper") == 0)
  {
    can_sys_ptr.reset(new as64::SpringDamperCanonicalSystem());
    USE_2nd_order_can_sys = true;
  }
  else
  {
    throw std::invalid_argument(std::string("Unsupported canonical system type: \"")+cmd_args.CAN_SYS_TYPE+"\"");
  }

  can_sys_ptr->init(cmd_args.x_end, tau);

  // set the DMP
  std::vector<std::shared_ptr<as64::DMP_>> dmp(D);
  for (int i=0; i<D; i++){

    if (cmd_args.DMP_TYPE.compare("DMP") == 0)
    {
      dmp[i].reset(new as64::DMP());
    }
    else if (cmd_args.DMP_TYPE.compare("DMP-bio") == 0)
    {
      dmp[i].reset(new as64::DMP_bio());
    }
    else if (cmd_args.DMP_TYPE.compare("DMP-plus") == 0)
    {
      dmp[i].reset(new as64::DMP_plus());
    }
    else
    {
      throw std::invalid_argument(std::string("Unsupported DMP type: \"")+cmd_args.DMP_TYPE+"\"");
    }

    std::cout << "Init DMP...\n";
    dmp[i]->init(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);

  }

  arma::mat F_offline_train_data;
  arma::mat Fd_offline_train_data;
  arma::vec offline_train_mse;
  arma::vec online_train_mse;

  arma::rowvec Time;
  arma::rowvec Time_offline_train;

  // ========  Train the DMP  ===========
  if (cmd_args.OFFLINE_DMP_TRAINING_enable)
  {
    std::cout << "DMP training...\n";
    timer.tic();

    offline_train_mse.zeros(D);

    n_data = yd_data.n_cols;

    Time = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;

    for (int i=0; i<D; i++)
    {
    	double y0 = yd_data(i,0);
    	double g0 = yd_data(i,n_data-1);

    	// ind = 1:n_data;
    	// ind = randperm(n_data);
    	// T = Time(ind);
    	// yd = yd_data(i,ind);
    	// dyd = dyd_data(i,ind);
    	// ddyd = ddyd_data(i,ind);

    	arma::rowvec T = Time;
    	arma::rowvec yd = yd_data.row(i);
    	arma::rowvec dyd = dyd_data.row(i);
    	arma::rowvec ddyd = ddyd_data.row(i);

    	arma::rowvec F_train, Fd_train;

    	dmp[i]->set_training_params(cmd_args.USE_GOAL_FILT, cmd_args.a_g, cmd_args.RLWR_lambda, cmd_args.RLWR_P);
    	offline_train_mse(i) = dmp[i]->train(T, yd, dyd, ddyd, y0, g0, cmd_args.train_method, &F_train, &Fd_train);

    	F_offline_train_data = arma::join_vert(F_offline_train_data, F_train);
    	Fd_offline_train_data = arma::join_vert(Fd_offline_train_data, Fd_train);

    }
    Time_offline_train = arma::linspace<arma::rowvec>(0,F_offline_train_data.size()-1,F_offline_train_data.size())*Ts;

    std::cout << "Elapsed time is " << timer.toc() << " seconds\n";

  }

  // ===========  DMP simulation  =============
  // set initial values
  n_data = yd_data.n_cols;
  arma::vec y0 = yd_data.col(0);
  arma::vec g0 = cmd_args.goal_sim_scale*yd_data.col(n_data-1);
  arma::vec g = cmd_args.USE_GOAL_FILT?y0:g0;
  arma::vec dg = arma::vec().zeros(D);
  double x = cmd_args.x0;
  double dx = 0;
  double u = USE_2nd_order_can_sys?0:x;
  double du = 0;
  arma::vec ddy = arma::vec().zeros(D);
  arma::vec dy = arma::vec().zeros(D);
  arma::vec y = y0;
  double t = 0;
  arma::vec y_robot = y0;
  arma::vec dy_robot = arma::vec().zeros(D);
  arma::vec dz = arma::vec().zeros(D);
  arma::vec z = arma::vec().zeros(D);
  arma::vec scaled_forcing_term = arma::vec().zeros(D);
  arma::vec shape_attr = arma::vec().zeros(D);
  arma::vec goal_attr = arma::vec().zeros(D);

  std::vector<arma::vec> P_lwr(D);
  for (int i=0; i<D; i++)
  {
    P_lwr[i] = arma::vec().ones(cmd_args.N_kernels)*cmd_args.RLWR_P;
  }
  arma::rowvec F = arma::rowvec().zeros(D,1);
  arma::rowvec Fd = arma::rowvec().zeros(D,1);

  double Fdist = 0;

  LogData log_data;

  log_data.dmp = dmp;

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

  log_data.Psi_data = std::vector<arma::mat>(D);
  log_data.P_lwr = std::vector<arma::mat>(D);
  log_data.DMP_w = std::vector<arma::mat>(D);


  tau = cmd_args.tau_sim_scale*tau;
  can_sys_ptr->set_tau(tau);

  int iters = 0;

  double dt;
  if (cmd_args.ONLINE_DMP_UPDATE_enable)
  {
    dt = Ts; // sync sim with training data
  }
  else
  {
    dt = cmd_args.sim_time_step; // simulation time_step
  }


  //logData.g0 = g0;

  std::cout << "DMP simulation...\n";
  timer.tic();


  while (true)
  {
    // data logging

    log_data.Time = join_horiz(log_data.Time, t);

    log_data.y_data = join_horiz(log_data.y_data, y);
    log_data.dy_data = join_horiz(log_data.dy_data, dy);

    log_data.z_data = join_horiz(log_data.z_data, z);
    log_data.dz_data = join_horiz(log_data.dz_data, dz);

    log_data.x_data = join_horiz(log_data.x_data, x);
    log_data.u_data = join_horiz(log_data.u_data, u);

    log_data.y_robot_data = join_horiz(log_data.y_robot_data, y_robot);
    log_data.dy_robot_data = join_horiz(log_data.dy_robot_data, dy_robot);


    log_data.Fdist_data = join_horiz(log_data.Fdist_data, Fdist);
    log_data.Force_term_data = join_horiz(log_data.Force_term_data, scaled_forcing_term);
    log_data.g_data = join_horiz(log_data.g_data, g);
    log_data.shape_attr_data = join_horiz(log_data.shape_attr_data, shape_attr);
    log_data.goal_attr_data = join_horiz(log_data.goal_attr_data, goal_attr);


    for (int i=0; i<D; i++){

      if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
      {
	log_data.DMP_w[i] = join_horiz(log_data.DMP_w[i], dmp[i]->w);
	log_data.P_lwr[i] = join_horiz(log_data.P_lwr[i], P_lwr[i]);

	double yd = yd_data(i,iters+1);
	double dyd = dyd_data(i,iters+1);
	double ddyd = ddyd_data(i,iters+1);
	dmp[i]->update_weights(x, u, yd, dyd, ddyd, y0(i), g0(i), g(i), P_lwr[i], cmd_args.RLWR_lambda);

	F(i) = dmp[i]->calc_Fd(y(i), dy(i), ddy(i), u, y0(i), g0(i), g(i));
	Fd(i) = dmp[i]->calc_Fd(yd, dyd, ddyd, u, y0(i), g0(i), g(i));
      }

      arma::vec Psi = dmp[i]->activation_function(x);
      log_data.Psi_data[i] = arma::join_horiz(log_data.Psi_data[i], Psi);

      arma::vec X(2);
      X << x << u;
      shape_attr(i) = dmp[i]->shape_attractor(X,g0(i),y0(i));
      goal_attr(i) = dmp[i]->goal_attractor(y(i),dy(i),g(i));

      scaled_forcing_term(i) = dmp[i]->forcing_term(x)*dmp[i]->forcing_term_scaling(u, y0(i), g0(i));

      double y_c = cmd_args.a_py*(y_robot(i)-y(i));
      double z_c = 0;

      arma::vec states_dot;
      states_dot = dmp[i]->get_states_dot(y(i), z(i), x, u, y0(i), g0(i), g(i), y_c, z_c);

      dy(i) = states_dot(0);
      dz(i) = states_dot(1);

      dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd;

    }

    if (cmd_args.ONLINE_DMP_UPDATE_enable && iters<n_data)
    {
        log_data.Time_online_train = join_horiz(log_data.Time_online_train, t);
	log_data.F_online_train_data = join_horiz(log_data.F_online_train_data, F);
	log_data.Fd_online_train_data = join_horiz(log_data.Fd_online_train_data, Fd);
    }

    if (cmd_args.USE_GOAL_FILT)
    {
      dg = -cmd_args.a_g*(g-g0)/can_sys_ptr->get_tau();
    }
    else
    {
      dg.zeros(D);
    }

    // Update phase variable

    arma::vec X_in(1);
    X_in(0) = x;
    if (USE_2nd_order_can_sys)
    {
      X_in.resize(2);
      X_in(1) = u;
    }

    arma::vec X_out = can_sys_ptr->get_derivative(X_in);

    dx = X_out(0);
    if (X_out.n_elem > 1)
    {
      du = X_out(1);
    }
    else
    {
      du = dx;
    }

    // Update disturbance force
    Fdist = cmd_args.APPLY_DISTURBANCE?Fdist_fun(t, cmd_args):0;

    if (cmd_args.USE_PHASE_STOP)
    {
      double stop_coeff = 1/(1+cmd_args.a_px*std::pow(arma::norm(y-y_robot),2));

      dx = dx*stop_coeff;
      du = du*stop_coeff;
      dg = dg*stop_coeff;
    }

    double err = arma::max(arma::abs(g0-y_robot));
    if (err <= cmd_args.sim_tol_stop) break;

    //std::cout << "err = " << err << "\n";

    iters++;
    if (iters >= cmd_args.sim_max_iters) break;

    // Numerical integration
    t = t + dt;

    y = y + dy*dt;

    z = z + dz*dt;

    g = g + dg*dt;

    x = x + dx*dt;
    if (x<0) x=0; // zero crossing can occur due to numberical integration

    u = USE_2nd_order_can_sys?u + du*dt:x;

    y_robot = y_robot + dy_robot*dt;

  }
  std::cout << "Elapsed time is " << timer.toc() << " seconds\n";

  if (cmd_args.OFFLINE_DMP_TRAINING_enable)
  {
    std::cout << "offline_train_mse:\n" << offline_train_mse << "\n";
  }

  if (cmd_args.ONLINE_DMP_UPDATE_enable)
  {
    std::cout << "online_train_mse:\n" << online_train_mse << "\n";
  }

  std::cout << "Saving results...";
  log_data.cmd_args = cmd_args;
  log_data.save(cmd_args.out_data_filename, true);
  log_data.save(cmd_args.out_data_filename, false, 10);
  std::cout << "[DONE]!\n";

  // std::cout << "c = " << dmp[0]->c.t() << "\n";
  // std::cout << "h = " << dmp[0]->h.t() << "\n";
  // std::cout << "w = " << dmp[0]->w << "\n";
  arma::vec w = dmp[0]->w;
  std::cout << "w = \n";
  for (int i=0;i<w.size();i++) std::cout << std::setprecision(10) << w(i) << "\n";


  /*
  // =============  export data  ================
  std::ofstream out(cmd_args.out_data_filename.c_str(), std::ios::out);
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: ") + cmd_args.out_data_filename);

  out << D << "\n";
  n_data = yd_data.n_cols;
  out << n_data << "\n";
  for (int i=0;i<n_data;i++) out << Time_demo(i) << " ";
  out << "\n";
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) out << yd_data(i,j) << " ";
    out << "\n";
    for (int j=0;j<n_data;j++) out << dyd_data(i,j) << " ";
    out << "\n";
    for (int j=0;j<n_data;j++) out << ddyd_data(i,j) << " ";
    out << "\n";
  }

  n_data = y_data.n_cols;
  out << n_data << "\n";
  for (int i=0;i<n_data;i++) out << Time[i] << " ";
  out << "\n";
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) out << y_data(i,j) << " ";
    out << "\n";
    for (int j=0;j<n_data;j++) out << dy_data(i,j) << " ";
    out << "\n";
    for (int j=0;j<n_data;j++) out << ddy_data(i,j) << " ";
    out << "\n";
  }

  out.close();

*/

  // =============================================
  // =============================================

  // Shutdown ROS node
  ros::shutdown();

  return 0;
}
