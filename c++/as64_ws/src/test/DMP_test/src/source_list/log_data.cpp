#include <log_data.h>

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

