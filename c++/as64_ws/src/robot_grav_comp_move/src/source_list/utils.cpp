#include <utils.h>

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


// **********************************
// **********    LogData   **********
// **********************************
LogData::LogData(int D){
  Psi_data.resize(D);
}

void LogData::print_mat(const arma::mat &m, std::ostream &out)
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

void LogData::print_vec(const arma::vec &v, std::ostream &out)
{
  int n_data = v.n_elem;
  out << n_data << "\n";
  for (int i=0;i<n_data;i++) out << v(i) << "\n";
  out << "\n";
}

void LogData::print_vec(const std::vector<double> &v, std::ostream &out)
{
  int n_data = v.size();
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v[j] << "\n";
  out << "\n";
}


void LogData::print_rowVec(const arma::rowvec &v, std::ostream &out)
{
  int n_data = v.n_elem;
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v(j) << " ";
  out << "\n\n";
}

void LogData::print_rowVec(const std::vector<double> &v, std::ostream &out)
{
  int n_data = v.size();
  out << n_data << "\n";
  for (int j=0;j<n_data;j++) out << v[j] << " ";
  out << "\n\n";
}


void LogData::print_vec_vec(const std::vector<arma::rowvec> &m, std::ostream &out)
{
  int D = m.size();
  int n_data = m[0].n_elem;
  out << D << " " << n_data << "\n";
  for (int i=0;i<D;i++){
    for (int j=0;j<n_data;j++) out << m[i](j) << "\n";
    out << "\n";
  }
}

void LogData::print_vec_mat(const std::vector<arma::mat> &m, std::ostream &out)
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


void LogData::save_demo_data(const std::string &filename)
{
  std::ofstream out;
  out.open(filename);
  int n_data;
  
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: ") + filename);

  // ========   Time_demo, yd_data, dyd_data, ddyd_data  ==========

  print_rowVec(Time_demo,out);
  
  print_mat(yd_data,out);
  print_mat(dyd_data,out);
  print_mat(ddyd_data,out);
  
  // ========   Time_train, F_train_data, Fd_train_data  ==========
  
  print_rowVec(Time_train,out);
  
  print_mat(F_train_data,out);
  print_mat(Fd_train_data,out);
  
  // ===========  Time, y_data, dy_data, y_robot_data, dy_robot_data, z_data, dz_data  ============
    
  print_rowVec(Time,out);
  
  print_mat(y_data,out);
  print_mat(dy_data,out);
  
  print_mat(y_robot_data,out);
  print_mat(dy_robot_data,out);
  
  print_mat(z_data,out);
  print_mat(dz_data,out);

  // =========== x_data, u_data, Fdist_data, Force_term_data, g_data  =============

  print_rowVec(x_data,out);
  print_rowVec(u_data,out);
  print_rowVec(Fdist_data,out);
  print_mat(Force_term_data,out);
  print_mat(g_data,out);

  // =========== Psi_data, shape_attr_data, goal_attr_data  =============

  print_mat(shape_attr_data,out);
  print_mat(goal_attr_data,out);
  print_vec_mat(Psi_data,out);
  
  out.close();
}
  
void LogData::save_train_sim_data(const std::string &filename)
{
  std::ofstream out;
  out.open(filename);
  int n_data;
  
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: ") + filename);

  int D = F_train_data.n_rows;
  
  // =========  D, Ts, g0  ===========
  
  out << D << "\n\n";
  
  out << Ts << "\n\n";
  
  print_vec(g0,out);
  
  // ========   Time_train, F_train_data, Fd_train_data  ==========
  
  print_rowVec(Time_train,out);
  
  print_mat(F_train_data,out);
  print_mat(Fd_train_data,out);
  
  // ===========  Time, y_data, dy_data, y_robot_data, dy_robot_data, z_data, dz_data  ============
    
  print_rowVec(Time,out);
  
  print_mat(y_data,out);
  print_mat(dy_data,out);
  
  print_mat(y_robot_data,out);
  print_mat(dy_robot_data,out);
  
  print_mat(z_data,out);
  print_mat(dz_data,out);

  // =========== x_data, u_data, Fdist_data, Force_term_data, g_data  =============

  print_rowVec(x_data,out);
  print_rowVec(u_data,out);
  print_rowVec(Fdist_data,out);
  print_mat(Force_term_data,out);
  print_mat(g_data,out);

  // =========== Psi_data, shape_attr_data, goal_attr_data  =============

  print_mat(shape_attr_data,out);
  print_mat(goal_attr_data,out);
  print_vec_mat(Psi_data,out);
  
  out.close();
}
	

// ***********************************
// **********    CMD_ARGS   **********
// ***********************************

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args(ros::NodeHandle &nh_)
{
  // ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("a_z", a_z)) a_z = 80;
  if (!nh_.getParam("b_z", b_z)) b_z = a_z/4;

  if (!nh_.getParam("USE_GOAL_FILT", USE_GOAL_FILT)) USE_GOAL_FILT = false;
  if (!nh_.getParam("a_g", a_g)) a_g = -100;

  if (!nh_.getParam("x_end", x_end)) x_end = 0.01;
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 150;
  if (!nh_.getParam("ts_scale", ts_scale)) ts_scale = 2;
  if (!nh_.getParam("std_K", std_K)) std_K = 1;

  if (!nh_.getParam("USE_PHASE_STOP", USE_PHASE_STOP)) USE_PHASE_STOP = false;
  if (!nh_.getParam("a_px", a_px)) a_px = 150;
  if (!nh_.getParam("a_py", a_py)) a_py = b_z;

  
  
  
  if (!nh_.getParam("can_sys_type", can_sys_type)) can_sys_type = "exp";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  
  if (!nh_.getParam("sim_time_step", sim_time_step)) sim_time_step = 0.01;
  // if (!nh_.getParam("sim_max_iters", sim_max_iters)) sim_max_iters = 500;
  
  // if (!nh_.getParam("data_input_path", data_input_path)) data_input_path = "";
  // if (!nh_.getParam("data_output_path", data_output_path)) data_output_path = "";
  
  // if (!nh_.getParam("in_data_filename", in_data_filename)) in_data_filename = "";
  // if (!nh_.getParam("out_data_filename", out_data_filename)) out_data_filename = "";
  
  // in_data_filename = data_input_path + in_data_filename;
  // out_data_filename = data_output_path + out_data_filename;

}
	
void CMD_ARGS::print(std::ostream &out) const
{
  out << "a_z: " << a_z << "\n";
  out << "b_z: " << b_z << "\n";
  out << "USE_GOAL_FILT: " << USE_GOAL_FILT << "\n";
  out << "a_g: " << a_g << "\n";
  out << "x_end: " << x_end << "\n";
  out << "N_kernels: " << N_kernels << "\n";
  out << "ts_scale: " << ts_scale << "\n";
  out << "std_K: " << std_K << "\n";
  out << "USE_PHASE_STOP: " << USE_PHASE_STOP << "\n";
  out << "a_px: " << a_px << "\n";
  //out << "a_py: " << a_py << "\n";

  

  out << "can_sys_type: " << can_sys_type << "\n";
  out << "train_method: " << train_method << "\n";	
  
  out << "sim_time_step: " << sim_time_step << "\n";
  // out << "sim_max_iters: " << sim_max_iters << "\n";
  
  // out << "data_input_path: " << data_input_path << "\n";
  // out << "data_output_path: " << data_output_path << "\n";
  
  // out << "in_data_filename: " << in_data_filename << "\n";
  // out << "out_data_filename: " << out_data_filename << "\n";
}



