
#include <DMP_lib/CanonicalSystem/CanonicalSystem.h>

namespace as64
{

CanonicalSystem::CanonicalSystem()
{}

double CanonicalSystem::get_tau()
{
  return tau;
}

void CanonicalSystem::set_tau(double t)
{
  tau = t;
}

void CanonicalSystem::init(double x_end, double tau)
{
  set_can_sys_params(x_end);
  set_tau(tau);    
}

}  // namespace as64

