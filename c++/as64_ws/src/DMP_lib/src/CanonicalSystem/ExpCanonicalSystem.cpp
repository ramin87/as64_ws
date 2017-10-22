
#include <DMP_lib/CanonicalSystem/ExpCanonicalSystem.h>

namespace as64
{
	
ExpCanonicalSystem::ExpCanonicalSystem()
{}


void ExpCanonicalSystem::set_can_sys_params(double x_end)
{
  a_x = -std::log(x_end);
}


arma::vec ExpCanonicalSystem::get_derivative(const arma::vec &X)
{
  arma::vec dX(1);
  double tau = get_tau();
  
  dX(0) = -a_x*X(0)/tau;
  
  return dX;
}


arma::mat ExpCanonicalSystem::get_continuous_output(const arma::rowvec &t, double x0)
{
  arma::mat x;
  double tau = get_tau();
  
  x = x0*arma::exp(-a_x*t/tau);
  
  return x;
}

}  // namespace as64

