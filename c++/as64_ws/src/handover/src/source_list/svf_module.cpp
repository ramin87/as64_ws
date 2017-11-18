#include "svf_module.h"


/**
* @brief SVF_module::init
* @param sigma_min, minimum allowable eigen value.
* @param shape_f, the greater the shape factor the closer are the filtered eigenvalues to the initial ones.
*/
SVF_module::SVF_module(double sigma_min, double shape_f)
{
  set_sigma_min(sigma_min);
  set_shape_factor(shape_f);
}


/**
* @brief SVF_module::set_sigma_min
* @param sigma_min, minimum allowable eigen value.
*/
void SVF_module::set_sigma_min(double sigma_min)
{
  sigma0 = sigma_min;
}


/**
* @brief SVF_module::set_shape_factor
* @param shape_f, the greater the shape factor the closer are the filtered eigenvalues to the initial ones.
*/
void SVF_module::set_shape_factor(double shape_f)
{
  v = shape_f;
}

double SVF_module::get_sigma_min() const
{
  return sigma0;
}

double SVF_module::get_shape_factor() const
{
  return v; 
}

double SVF_module::filter_eig_val(double sigma) const
{
  double filt_sigma = ( std::pow(sigma,3.0) + v*std::pow(sigma,2.0) + 2*sigma + 2*sigma0 ) / (std::pow(sigma,2.0) + v*sigma + 2);
  
  return filt_sigma;
}

Eigen::MatrixXd SVF_module::inv(Eigen::MatrixXd M) const
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd S = svd.singularValues().asDiagonal();
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();
  
  // Eigen::MatrixXd M_reconstr = U*S*V.transpose();
  
  for (int i=0;i<S.cols(); i++) S(i,i) = 1/filter_eig_val(S(i,i));
  
  return V*S*U.transpose();
}
