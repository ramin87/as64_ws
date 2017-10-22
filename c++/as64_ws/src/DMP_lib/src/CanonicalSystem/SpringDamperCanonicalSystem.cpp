
#include <DMP_lib/CanonicalSystem/SpringDamperCanonicalSystem.h>

namespace as64
{
	
SpringDamperCanonicalSystem::SpringDamperCanonicalSystem()
{}


void SpringDamperCanonicalSystem::set_can_sys_params(double x_end)
{
  #define g(x) (1 + x/2)*exp(-x/2) - x_end
  #define dg(x) -(x/2).^2*exp(-x/2)
  
  //std::function<double(double)> g = [](double x) -> double { (1 + x/2)*std::exp(-x/2); };
  
  //double (*g)(double) = [](double x) -> double { (1 + x/2)*std::exp(-x/2); };
  //double (*dg)(double) = [](double x) -> double { -std::pow((x/2),2)*std::exp(-x/2); };
          
  double a1 = 0;
  double a3 = 800;
  double a2 = (a1+a3)/2;
  double a2_prev = a2;
  double g1, g2, g3;
  
  double tol_stop = 1e-12;

  int iters = 0;

  while (true){
      a2_prev = a2;

      a2 = (a1+a3)/2;

      g1 = g(a1);// - x_end;

      g2 = g(a2);// - x_end;

      g3 = g(a3);// - x_end;

      if (g1*g2<0) a3 = a2;
      else if (g2*g3<0) a1 = a2;
      else throw std::logic_error("SpringDamperCanonicalSystem::set_can_sys_params: No feasible solution exists in this interval");

      a2 = (a1+a3)/2;

      iters++;

      if (std::abs(a2-a2_prev) < tol_stop) break;
  }
  
  a_u = a2;
  b_u = a_u/4;
}


arma::vec SpringDamperCanonicalSystem::get_derivative(const arma::vec &X)
{
  arma::vec dX(2);
  double tau = get_tau();
  
  double x = X(0);
  double u = X(1);
          
  double du = a_u*(-b_u*x - u) / tau;
  double dx = u / tau;
          
  dX(0) = dx;
  dX(1) = du;
  
  return dX;
}


arma::mat SpringDamperCanonicalSystem::get_continuous_output(const arma::rowvec &t, double x0)
{
  arma::mat X(2,t.n_elem);
  double tau = get_tau();
            
  double a = a_u/(2*tau);
  arma::rowvec exp_at = arma::exp(-a*t);
  
  arma::rowvec x = x0 * (1+a*t)%exp_at;

  arma::rowvec dx = x0 * ( -a*a*t%exp_at );
  
  arma::rowvec u = dx * tau;
  
  X.row(0) = x;
  X.row(1) = u;
  
  return X;
}

}  // namespace as64

