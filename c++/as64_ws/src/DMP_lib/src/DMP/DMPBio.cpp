/**
 * Copyright (C) 2017 DMP
 */

#include <DMP_lib/DMP/DMPBio.h>

namespace as64
{

DMPBio::DMPBio()
{}

DMPBio::DMPBio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K):
DMP_(N_kernels, a_z, b_z, can_sys_ptr, std_K)
{}


double DMPBio::forcing_term_scaling(double u, double y0, double g0)
{
	double s = u * this->a_z * this->b_z;
	return s;
}

double DMPBio::shape_attractor(const arma::vec &X, double g0, double y0)
{
	double x,u;
  
  x = X(0);
  
  if (X.n_elem == 1) u = x;
  else u = X(1);
  
  double K = this->a_z * this->b_z;

  double f = this->forcing_term(x);
  
  double s_attr = f * K * u - K * (g0 - y0) * u;
				
  return s_attr;
}

double DMPBio::calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0)
{	
  double v_scale = this->get_v_scale();
  
  double g_attr = -this->goal_attractor(y, v_scale*dy, g);
  
  double Fd = (ddy *  std::pow(v_scale,2) + g_attr + this->a_z * this->b_z * (g0 - y0) * u);
	
	return Fd;
}

} //as64
