
#include <DMP_lib/CanonicalSystem/LinCanonicalSystem.h>

namespace as64
{
	
LinCanonicalSystem::LinCanonicalSystem()
{}


void LinCanonicalSystem::set_can_sys_params(double x_end)
{
  a_x = x0-x_end;
}


arma::vec LinCanonicalSystem::get_derivative(const arma::vec &X)
{
  arma::vec dX(1);
  double tau = get_tau();
  
  dX(0) = -a_x/tau;
  
  return dX;
}


arma::mat LinCanonicalSystem::get_continuous_output(const arma::rowvec &t)
{
  arma::mat x;
  double tau = get_tau();
  
  x = x0 - a_x*t/tau;

  return x;
}

}  // namespace as64

