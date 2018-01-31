#include <utils.h>

void get_canClock_gatingFuns_DMP(const CMD_ARGS &cmd_args, int D, double tau,
  std::shared_ptr<as64_::CanonicalClock> &canClockPtr,
  std::shared_ptr<as64_::GatingFunction> &shapeAttrGatingPtr,
  std::shared_ptr<as64_::GatingFunction> &goalAttrGatingPtr,
  std::vector<std::shared_ptr<as64_::DMP_>> &dmp)
{
  // ========================================================
  // Init canonical clock
  canClockPtr = as64_::getCanClock(cmd_args.CAN_CLOCK_TYPE, tau);

  // ========================================================
  // Init shape attractor gating function
  shapeAttrGatingPtr = as64_::getGatingFun(cmd_args.SHAPE_ATTR_GATTING_TYPE, cmd_args.SHAPE_ATTR_GATTING_u0, cmd_args.SHAPE_ATTR_GATTING_u_end);

  // ========================================================
  // Init goal attractor gating function
  goalAttrGatingPtr = as64_::getGatingFun(cmd_args.GOAL_ATTR_GATTING_TYPE, cmd_args.GOAL_ATTR_GATTING_u0, cmd_args.GOAL_ATTR_GATTING_u_end);

  // ========================================================
  // Extra args for the DMP
  as64_::param_::ParamList paramList;
  paramList.setParam("kernelStdScaling", cmd_args.kernelStdScaling);
  paramList.setParam("k_trunc_kernel", cmd_args.k_trunc_kernel);
  paramList.setParam("Wmin", cmd_args.Wmin);
  paramList.setParam("Freq_min", cmd_args.Freq_min);
  paramList.setParam("Freq_max", cmd_args.Freq_max);
  paramList.setParam("P1_min", cmd_args.P1_min);

  dmp.resize(D);
  for (int i=0;i<D;i++)
  {
  dmp[i] = as64_::getDMP(cmd_args.DMP_TYPE, cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z,
      canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, &paramList);
  }

}


void load_data(const std::string &data_file_name, arma::mat &yd_data, arma::mat &dyd_data,
               arma::mat &ddyd_data, arma::rowvec &Time_demo, bool binary)
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

  unsigned int D;

  as64_::io_::read_mat(Time_demo, in, binary);

  as64_::io_::read_scalar(D, in, binary);

  as64_::io_::read_mat(yd_data, in, binary);
  as64_::io_::read_mat(dyd_data, in, binary);
  as64_::io_::read_mat(ddyd_data, in, binary);

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
