/**
 * Copyright (C) 2017 DMP
 */

#include <DMP_lib/DMP/DMP.h>

namespace as64
{

DMP::DMP()
{}

DMP::DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K):
DMP_(N_kernels, a_z, b_z, can_sys_ptr, std_K)
{}

double DMP::forcing_term_scaling(double u, double y0, double g0)
{
	double s = u * (g0 - y0);
	return s;
}

double DMP::shape_attractor(const arma::vec &X, double g0, double y0)
{
  double x,u;

  x = X(0);

  if (X.n_elem == 1) u = x;
  else u = X(1);

  double s_attr = this->forcing_term(x) * this->forcing_term_scaling(u, y0, g0);

  return s_attr;
}

double DMP::calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0)
{
  double v_scale = this->get_v_scale();

  double g_attr = this->goal_attractor(y, v_scale*dy, g);
  double Fd = ddy * std::pow(v_scale, 2) - g_attr;

	return Fd;
}

}  // namespace as64
