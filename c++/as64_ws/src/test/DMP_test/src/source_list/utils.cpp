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

double Fdist_fun(double t, const CMD_ARGS &cmd_args)
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



