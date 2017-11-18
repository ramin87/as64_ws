
#include <DMP_lib/CanonicalSystem/CanonicalSystem.h>

namespace as64
{

CanonicalSystem::CanonicalSystem()
{}

double CanonicalSystem::get_tau()
{
  return tau;
}

void CanonicalSystem::set_tau(double tau)
{
  this->tau = tau;
}

void CanonicalSystem::init(double x_end, double tau, double x0)
{
  set_can_sys_params(x_end);
  set_tau(tau);    
  this->x0 = x0;
}

}  // namespace as64

