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

void load_data(const std::string &data_file_name, arma::mat &data, double &Ts, bool binary)
{
  std::ifstream in;

  if (binary)
  {
    in.open(data_file_name.c_str());
  }
  else{
    in.open(data_file_name.c_str(),std::ios::binary);
  }

  if (!in) throw std::ios_base::failure(std::string("Couldn't open file: ") + data_file_name);

  int D, n_data;

  // in >> Ts;
  as64_::io_::read_scalar(Ts, in, binary);

  as64_::io_::read_mat(data, in, binary);

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

arma::rowvec  join_horiz(const arma::rowvec &v, double a)
{
  arma::rowvec v3, v2(1);
  v2(0) = a;

  v3 = arma::join_horiz(v,v2);

  return v3;
}

arma::mat join_horiz(const arma::mat &v, const arma::mat &v2)
{
  arma::mat v3;

  v3 = arma::join_horiz(v,v2);

  return v3;
}
