/**
 * Copyright (C) 2017 as64
 */

#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <fstream>

#include <utils.h>

#include <DMP_lib/DMP/DMP.h>
#include <DMP_lib/DMP/DMPBio.h>

#include <DMP_lib/CanonicalSystem/ExpCanonicalSystem.h>
#include <DMP_lib/CanonicalSystem/LinCanonicalSystem.h>
#include <DMP_lib/CanonicalSystem/SpringDamperCanonicalSystem.h>

CMD_ARGS cmd_args;

double Fdist_fun(double t)
{
  double Fmax = cmd_args.Fdist_max;
  double Fmin = cmd_args.Fdist_min;
  double t1 = cmd_args.t1_fdist;
  double t2 = cmd_args.t2_fdist;

  double tf = t2 - t1;

  double tb = tf*0.15;
  double a = (Fmax - Fmin) / tb;

  double Fdist = Fmin;

  if (t>t1 && t<t2){
      if (t < t1+tb) Fdist = a*(t-t1) + Fmin;
      else if (t < t2-tb) Fdist = Fmax;
      else Fdist = -a*(t-t2) + Fmin;
  }
  
  return Fdist;

}

int main(int argc, char** argv)
{    
  // Initialize the ROS node
  ros::init(argc, argv, "DMP_test_node");
  ros::NodeHandle nh("~");
  
  arma::wall_clock timer;
  double elapsed_time;

  // ============================================
  // ============================================

  bool USE_2nd_order_can_sys = false;
  
  cmd_args.parse_cmd_args();
  cmd_args.print(std::cout);
  
  std::shared_ptr<as64::CanonicalSystem> can_sys_ptr;
  
  if (cmd_args.CAN_SYS_TYPE.compare("exp") == 0) can_sys_ptr.reset(new as64::ExpCanonicalSystem());
  else if (cmd_args.CAN_SYS_TYPE.compare("lin") == 0) can_sys_ptr.reset(new as64::LinCanonicalSystem());
  else if (cmd_args.CAN_SYS_TYPE.compare("spring-damper") == 0){
    can_sys_ptr.reset(new as64::SpringDamperCanonicalSystem());
    USE_2nd_order_can_sys = true;
  }else throw std::runtime_error(std::string("Unsupported canonical system type: ")+cmd_args.CAN_SYS_TYPE);
  
  int D;
  int n_data;
  double Ts;
  arma::mat data;

  // ========  Load demos and process demos  ===========
  load_data(cmd_args.in_data_filename, data, Ts);
  
  D = data.n_rows;
  
  LogData logData(D);
  
  arma::mat yd_data, dyd_data, ddyd_data;
  
  process_demos(data,Ts, yd_data, dyd_data, ddyd_data, cmd_args.add_points_percent, cmd_args.smooth_points_percent);
  
  n_data = yd_data.n_cols; // number of points in each dimension
  arma::rowvec Time_demo = arma::linspace<arma::rowvec>(0,n_data-1,n_data) * Ts;
  logData.Time_demo = Time_demo;
  logData.yd_data = yd_data;
  logData.dyd_data = dyd_data;
  logData.ddyd_data = ddyd_data;
  logData.D = D;
  logData.Ts = Ts;
  
  double tau = (n_data-1)*Ts;

  can_sys_ptr->init(cmd_args.x_end, tau);
  
  std::vector<as64::DMP> dmp(D);
  for (int i=0; i<D; i++){
      dmp[i].init(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K, cmd_args.USE_GOAL_FILT, cmd_args.a_g);
  }
  
  // =========   Train the DMP  ============
  std::cout << "DMP training...\n";
  
  timer.tic();
  arma::vec train_mse = arma::vec().zeros(D);
  for (int i=0; i<D; i++){
    arma::rowvec F_train, Fd_train;
    
    train_mse(i) = dmp[i].train(yd_data.row(i), dyd_data.row(i), ddyd_data.row(i), Ts, cmd_args.train_method, F_train, Fd_train);
    
    logData.F_train_data = arma::join_vert(logData.F_train_data, F_train);
    logData.Fd_train_data = arma::join_vert(logData.Fd_train_data, Fd_train);
    
    //std::cout << "c = " << dmp[i].c.t() << "\n\n";
    //std::cout << "h = " << dmp[i].h.t() << "\n\n";
    //std::cout << "w = " << dmp[i].w.t() << "\n\n";
      
  }
  std::cout << "Elapsed time is " << timer.toc() << " seconds\n";

  n_data = logData.F_train_data.n_cols;
  arma::rowvec Time_train = arma::linspace<arma::rowvec>(0,n_data-1,n_data) * Ts;
  logData.Time_train = Time_train;
  
  // ===========  DMP simulation  =============
  // set initial values
  n_data = yd_data.n_cols;
  
  arma::vec y0 = yd_data.col(0);
  arma::vec g0 = yd_data.col(n_data-1);
  arma::vec g = cmd_args.USE_GOAL_FILT?y0:g0;
  arma::vec dg = arma::vec().zeros(D);
  double x = cmd_args.x0;
  double dx = 0;
  double u = USE_2nd_order_can_sys?0:x;
  double du = 0;
  arma::vec dy = arma::vec().zeros(D);
  arma::vec y = y0;
  double t = 0;
  arma::vec y_robot = y0;
  arma::vec dy_robot = arma::vec().zeros(D);
  arma::vec dz = arma::vec().zeros(D);
  arma::vec z = arma::vec().zeros(D);
  arma::vec force_term = arma::vec().zeros(D);
  arma::vec shape_attr = arma::vec().zeros(D);
  arma::vec goal_attr = arma::vec().zeros(D);
  
  double Fdist = 0;

  double dt = cmd_args.sim_time_step; // simulation time_step

  // Variables to store the simulation data
  // ...

  
  tau = tau * cmd_args.tau_sim_scale;
  can_sys_ptr->set_tau(tau);
  
  int iters = 0;
  
  logData.g0 = g0;

  std::cout << "DMP simulation...\n";
  timer.tic();
  
  
  while (true){
    // data logging
    logData.Time.push_back(t);
    
    logData.dy_data = arma::join_horiz(logData.dy_data, dy);
    logData.y_data = arma::join_horiz(logData.y_data, y);

    logData.dy_robot_data = arma::join_horiz(logData.dy_robot_data, dy_robot);
    logData.y_robot_data = arma::join_horiz(logData.y_robot_data, y_robot);

    logData.z_data = arma::join_horiz(logData.z_data, z);
    logData.dz_data = arma::join_horiz(logData.dz_data, dz);
        
    logData.x_data.push_back(x);
    logData.u_data.push_back(u);
    
    logData.Fdist_data.push_back(Fdist);
    
    logData.Force_term_data = arma::join_horiz(logData.Force_term_data, force_term);
    
    logData.g_data = arma::join_horiz(logData.g_data, g); 
    
    logData.shape_attr_data = arma::join_horiz(logData.shape_attr_data, shape_attr);
    
    logData.goal_attr_data = arma::join_horiz(logData.goal_attr_data, goal_attr);
    
    for (int i=0;i<D;i++){
        
        double v_scale = dmp[i].get_v_scale(); // 1 / (tau*dmp[i].a_s);
        
        arma::vec Psi = dmp[i].activation_function(x);
        logData.Psi_data[i] = arma::join_horiz(logData.Psi_data[i], Psi);
       
	//arma::vec X(2);
	//X(0) = x;
	//X(1) = u;
        //shape_attr(i) = dmp[i].shape_attractor(X,g0(i),y0(i));
        //goal_attr(i) = dmp[i].goal_attractor(y(i),dy(i),g(i));
        
        force_term(i) = dmp[i].forcing_term(x)*u*(g0(i)-y0(i));
        
        dz(i) = ( dmp[i].a_z*(dmp[i].b_z*(g(i)-y(i))-z(i)) + force_term(i) ) / v_scale;
        
        dy(i) = ( z(i) - cmd_args.a_py*(y_robot(i)-y(i)) ) / v_scale;
        
        dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd; 
      
    }
    
    if (cmd_args.USE_GOAL_FILT) dg = -cmd_args.a_g*(g-g0)/can_sys_ptr->get_tau();
    else dg.zeros(D);
    
    // Update phase variable
    
    arma::vec X_in(1);
    X_in(0) = x;
    if (USE_2nd_order_can_sys){
      X_in.resize(2);
      X_in(1) = u;
    }
        
    arma::vec X_out = can_sys_ptr->get_derivative(X_in);
    
    dx = X_out(0);
    if (X_out.n_elem > 1) du = X_out(1);
    else du = dx;

    // Update disturbance force
    Fdist = cmd_args.APPLY_DISTURBANCE?Fdist_fun(t):0;
     
    if (cmd_args.USE_PHASE_STOP){
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
    
    u = u + du*dt;
    
    y_robot = y_robot + dy_robot*dt;
      
  }
  std::cout << "Elapsed time is " << timer.toc() << " seconds\n";

  std::cout << "train_mse:\n" << train_mse << "\n";
  
  std::cout << "Saving results...";
  logData.cmd_args = cmd_args;
  logData.save(cmd_args.out_data_filename);
  std::cout << "[DONE]!";
  
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
