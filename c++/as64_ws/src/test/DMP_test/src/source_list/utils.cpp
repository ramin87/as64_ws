#include <utils.h>


void movingAverageFilter(const arma::rowvec &y, arma::rowvec &y_filt, int win_n)
{ 
  if (win_n == 1){
      y_filt = y;
      return;
  }
  
  if ((win_n%2) == 0) win_n++;

  int add_points = std::floor((double)win_n/2)-1;
  int n = y.n_elem;

  arma::rowvec y_filt_temp;

  y_filt_temp.resize(add_points + n + add_points);
  y_filt_temp.subvec(0, add_points-1).fill(y(0));
  y_filt_temp.subvec(add_points, n+add_points-1) = y;
  y_filt_temp.subvec(n+add_points, n+add_points+add_points-1).fill(y(n-1));

  int k = std::ceil((double)win_n/2) - 1;
  double s = arma::sum(y_filt_temp.subvec(0,win_n-2));
  n = y_filt_temp.n_elem;
  
  for (int i=win_n-1; i<n; i++){   
    s = s + y_filt_temp(i);
    double y_k = y_filt_temp(k);
    y_filt_temp(k) = s/win_n;
    s = s - y_filt_temp(i-win_n+1) - y_k + y_filt_temp(k);
    k = k+1;
  }
  
  y_filt = y_filt_temp.subvec(add_points, n-add_points-1);

}

void process_demos(const arma::mat &data, double Ts, arma::mat &yd_data, arma::mat &dyd_data, arma::mat &ddyd_data, double add_points_percent, double smooth_points_percent)
{
  yd_data = data;
  
  int D = yd_data.n_rows;
  int n_data = yd_data.n_cols;
  
  int add_points = std::ceil(n_data*add_points_percent);
  arma::vec y0 = yd_data.col(0);
  arma::vec yend = yd_data.col(n_data-1);
  
  yd_data = arma::join_horiz(arma::join_horiz(arma::repmat(y0,1,add_points), yd_data), arma::repmat(yend,1,add_points));

  n_data = yd_data.n_cols;
  int smooth_points = std::ceil(n_data*smooth_points_percent);
  int smooth_times = 2;

  dyd_data.resize(D,n_data);
  ddyd_data.resize(D,n_data);
  
  for (int i=0;i<D;i++){
    dyd_data.row(i) = arma::join_horiz(arma::vec().zeros(1), arma::diff(yd_data.row(i)))/Ts;
    ddyd_data.row(i) = arma::join_horiz(arma::vec().zeros(1), arma::diff(dyd_data.row(i)))/Ts;
  }
  // This doesn't work for some reason...???
  //dyd_data = arma::join_horiz(arma::vec().zeros(D), arma::diff(yd_data,0))/Ts;
  //ddyd_data = arma::join_horiz(arma::vec().zeros(D), arma::diff(dyd_data,0))/Ts;
  
  for (int i=0;i<D;i++){
    for (int k=0;k<smooth_times;k++){
      arma::rowvec y_filt = dyd_data.row(i);
      movingAverageFilter(dyd_data.row(i), y_filt, smooth_points);
      dyd_data.row(i) = y_filt;
      
      y_filt = ddyd_data.row(i);
      movingAverageFilter(ddyd_data.row(i), y_filt, smooth_points);
      ddyd_data.row(i) = y_filt;
    }
  }
  
}

void load_data(const std::string &data_file_name, arma::mat &data, double &Ts)
{
  std::ifstream in(data_file_name.c_str());
  
  if (!in) throw std::ios_base::failure(std::string("Couldn't open file: ") + data_file_name);
  
  int D, n_data;
  
  in >> D >> n_data >> Ts;
  
  data.resize(D, n_data);
  
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) in >> data(i,j);
  }
  
}

/*
// **********************************
// **********    LogData   **********
// **********************************
void LogData::push()
{}
			
bool LogData::save(const std::string &filename)
{
	std::ofstream out(filename);
	if (!out){
		std::cerr << "Couldn't create file: \"" << filename << "\"...\n";
		return false;
	}
	
	out.precision(10);
	
	
	return true;
}*/


LogData::LogData(int D){
  Psi_data.resize(D);
}

void LogData::print_mat(const arma::mat &m)
{
  int D = m.n_rows;
  int n_data = m.n_cols;
  
  out << D << " " << n_data << "\n";
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) out << m(i,j) << " ";
    out << "\n";
  }
  out << "\n";
}

void LogData::print_vec(const arma::vec &v)
{
  int n_data = v.n_elem;
  out << n_data << "\n";
  for (int i=0;i<n_data;i++) out << v(i) << "\n";
  out << "\n";
}

void LogData::print_vec(const std::vector<double> &v)
{
  int n_data = v.size();
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v[j] << "\n";
  out << "\n";
}


void LogData::print_rowVec(const arma::rowvec &v)
{
  int n_data = v.n_elem;
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v(j) << " ";
  out << "\n\n";
}

void LogData::print_rowVec(const std::vector<double> &v)
{
  int n_data = v.size();
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v[j] << " ";
  out << "\n\n";
}


void LogData::print_vec_vec(const std::vector<arma::rowvec> &m)
{
  int D = m.size();
  int n_data = m[0].n_elem;
  out << D << " " << n_data << "\n";
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) out << m[i](j) << "\n";
    out << "\n";
  }
}

void LogData::print_vec_mat(const std::vector<arma::mat> &m)
{
  int D = m.size();
  int rows = m[0].n_rows;
  int cols = m[0].n_cols;

  out << D << " " << rows << " " << cols << "\n";
  for (int k=0;k<D;k++){
    for (int i=0;i<rows;i++){
      for (int j=0;j<cols;j++) out << m[k](i,j) << " ";
      out << "\n";
    }
  }
  out << "\n";
}




void LogData::save(const std::string &filename)
{
  out.open(filename);
  int n_data;
  
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: ") + filename);

  int D = F_train_data.n_rows;
  
  // =========  D, Ts, g0  ===========
  
  out << D << "\n\n";
  
  out << Ts << "\n\n";
  
  print_vec(g0);
  
  // ========   Time_demo, yd_data, dyd_data, ddyd_data  ==========

  print_rowVec(Time_demo);
  
  print_mat(yd_data);
  print_mat(dyd_data);
  print_mat(ddyd_data);
  
  // ========   Time_train, F_train_data, Fd_train_data  ==========
  
  print_rowVec(Time_train);
  
  print_mat(F_train_data);
  print_mat(Fd_train_data);
  
  // ===========  Time, y_data, dy_data, y_robot_data, dy_robot_data, z_data, dz_data  ============
		
  print_rowVec(Time);
  
  print_mat(y_data);
  print_mat(dy_data);
  
  print_mat(y_robot_data);
  print_mat(dy_robot_data);
  
  print_mat(z_data);
  print_mat(dz_data);

  // =========== x_data, u_data, Fdist_data, Force_term_data, g_data  =============

  print_rowVec(x_data);
  print_rowVec(u_data);
  print_rowVec(Fdist_data);
  print_mat(Force_term_data);
  print_mat(g_data);

  // =========== Psi_data, shape_attr_data, goal_attr_data  =============

  print_mat(shape_attr_data);
  print_mat(goal_attr_data);
  print_vec_mat(Psi_data);
  
  out.close();
}
	

// ***********************************
// **********    CMD_ARGS   **********
// ***********************************

CMD_ARGS::CMD_ARGS() {}

  std::string data_input_path;
  std::string data_output_path;
  
  std::string in_data_filename;
  std::string out_data_filename;
  
bool CMD_ARGS::parse_cmd_args()
{
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("a_z", a_z)) a_z = 80;
  if (!nh_.getParam("b_z", b_z)) b_z = a_z/4;
  if (!nh_.getParam("x0", x0)) x0 = 1;
  if (!nh_.getParam("x_end", x_end)) x_end = 0.01;
  
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 60;
  if (!nh_.getParam("std_K", std_K)) std_K = 1;
  
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  if (!nh_.getParam("CAN_SYS_TYPE", CAN_SYS_TYPE)) CAN_SYS_TYPE = "exp";
  if (!nh_.getParam("DMP_TYPE", DMP_TYPE)) DMP_TYPE = "DMP";
  
  if (!nh_.getParam("add_points_percent", add_points_percent)) add_points_percent = 0.02;
  if (!nh_.getParam("smooth_points_percent", smooth_points_percent)) smooth_points_percent = 0.02;
  
  if (!nh_.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = false;
  if (!nh_.getParam("a_g", a_g)) a_g = 5;
  if (!nh_.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = false;
  if (!nh_.getParam("a_px", a_px)) a_px = 150;
  if (!nh_.getParam("a_py", a_py)) a_py = b_z;
  
  if (!nh_.getParam("Kd", Kd)) Kd = 50;
  if (!nh_.getParam("Dd", Dd)) Dd = 1.5;

  if (!nh_.getParam("APPLY_DISTURBANCE", APPLY_DISTURBANCE)) APPLY_DISTURBANCE = false;
  if (!nh_.getParam("Fdist_min", Fdist_min)) Fdist_min = 3;
  if (!nh_.getParam("Fdist_max", Fdist_max)) Fdist_max = 60;
  if (!nh_.getParam("t1_fdist", t1_fdist)) t1_fdist = 0.2;
  if (!nh_.getParam("t2_fdist", t2_fdist)) t2_fdist = 1.5;
      
  if (!nh_.getParam("sim_time_step", sim_time_step)) sim_time_step = 0.005;
  if (!nh_.getParam("sim_tol_stop", sim_tol_stop)) sim_tol_stop = 0.5*1e-3;
  if (!nh_.getParam("sim_max_iters", sim_max_iters)) sim_max_iters = 2000;
  if (!nh_.getParam("tau_sim_scale", tau_sim_scale)) tau_sim_scale = 1;
  
  if (!nh_.getParam("data_input_path", data_input_path)) data_input_path = "";
  if (!nh_.getParam("data_output_path", data_output_path)) data_output_path = "";
  
  if (!nh_.getParam("in_data_filename", in_data_filename)) in_data_filename = "";
  if (!nh_.getParam("out_data_filename", out_data_filename)) out_data_filename = "";
  
  in_data_filename = data_input_path + in_data_filename;
  out_data_filename = data_output_path + out_data_filename;

}
	
void CMD_ARGS::print(std::ostream &out) const
{
  out << "a_z: " << a_z << "\n";
  out << "b_z: " << b_z << "\n";
  out << "x0: " << x0 << "\n";
  out << "x_end: " << x_end << "\n";
  out << "N_kernels: " << N_kernels << "\n";
  out << "std_K: " << std_K << "\n";

  out << "train_method: " << train_method << "\n";
  out << "CAN_SYS_TYPE: " << CAN_SYS_TYPE << "\n";
  out << "DMP_TYPE: " << DMP_TYPE << "\n";
  
  out << "add_points_percent: " << add_points_percent << "\n";
  out << "smooth_points_percent: " << smooth_points_percent << "\n";
  	
  out << "USE_GOAL_FILT: " << USE_GOAL_FILT << "\n";
  out << "a_g: " << a_g << "\n";
  out << "USE_PHASE_STOP: " << USE_PHASE_STOP << "\n";
  out << "a_px: " << a_px << "\n";
  out << "a_py: " << a_py << "\n";
  
  out << "Kd: " << Kd << "\n";
  out << "Dd: " << Dd << "\n";
  
  out << "APPLY_DISTURBANCE: " << APPLY_DISTURBANCE << "\n";
  out << "Fdist_min: " << Fdist_min << "\n";
  out << "Fdist_max: " << Fdist_max << "\n";
  out << "t1_fdist: " << t1_fdist << "\n";
  out << "t2_fdist: " << t2_fdist << "\n";
  
  out << "sim_time_step: " << sim_time_step << "\n";
  out << "sim_tol_stop: " << sim_tol_stop << "\n";
  out << "sim_max_iters: " << sim_max_iters << "\n";
  out << "tau_sim_scale: " << tau_sim_scale << "\n";
  
  out << "data_input_path: " << data_input_path << "\n";
  out << "data_output_path: " << data_output_path << "\n";
  
  out << "in_data_filename: " << in_data_filename << "\n";
  out << "out_data_filename: " << out_data_filename << "\n";
}



