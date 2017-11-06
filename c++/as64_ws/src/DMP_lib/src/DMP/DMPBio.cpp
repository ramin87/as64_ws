/**
 * Copyright (C) 2017 DMP
 */

#include <DMP_lib/DMP/DMPBio.h>

namespace as64
{

DMPBio::DMPBio()
{}

DMPBio::DMPBio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K, bool USE_GOAL_FILT, double a_g):
DMPBase(N_kernels, a_z, b_z, cs_ptr, std_K, USE_GOAL_FILT, a_g)
{}

arma::vec DMPBio::forcing_term_scaling(arma::rowvec &u, arma::rowvec &y0, arma::rowvec &g0)
{
	return u.t() * this->a_z * this->b_z;
}

double DMPBio::forcing_term_scaling(double u, double y0, double g0)
{
	return u * this->a_z * this->b_z;
}

double DMPBio::shape_attractor(const arma::vec X, double g0, double y0)
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

void DMPBio::calculate_Fd(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, arma::rowvec &u, arma::rowvec &g, double g0, double y0, arma::rowvec &Fd)
{	
  double v_scale = this->get_v_scale();
  
  arma::rowvec ddzd_data = ddyd_data * std::pow(v_scale,2);
  arma::rowvec g_attr_data = -this->a_z * (this->b_z * (g - yd_data) - dyd_data * v_scale);
  
  Fd = (ddzd_data + g_attr_data + this->a_z * this->b_z * (g0 - y0) * u);
}

} //as64
