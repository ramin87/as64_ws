/** DMP class
*  Implements 1-D DMP plus. 
*
*/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_PLUS_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_PLUS_H

#include <DMP_lib/DMP/DMP_.h>

namespace as64
{

class DMP_plus:public DMP_
{
public:
	
	arma::vec b; ///< N_kernelsx1 vector with the bias term for each weight of the DMP
	double k_trunc_kernel; ///< gain multiplied by the std of each kernel to define the truncated kernels width
	
	DMP_plus();
	
	/** \brief DMP plus constructor
	 *  @param[in] N_kernels: the number of kernels
	 *  @param[in] a_z: Parameter \a a_z relating to the spring-damper system.
	 *  @param[in] b_z: Parameter \a b_z relating to the spring-damper system.
	 *  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
	 *  @param[in] std_K: Scales the std of each kernel (optional, default = 1).
	 */ 
	DMP_plus(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);
	
	void init();
	
	void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);
	
	double train(const arma:: rowvec &Time, const arma::rowvec &yd_data, const arma::rowvec &dyd_data, 
							 const arma::rowvec &ddyd_data, double y0, double g0, const std::string &train_method,
							 arma::rowvec *Fd_ptr=NULL, arma::rowvec *F_ptr=NULL);
	
	double forcing_term(double x);
	
	double forcing_term_scaling(double u, double y0, double g0);
	
	double shape_attractor(const arma::vec &X, double g0, double y0);
	
	arma::vec activation_function(double x);
	
	double calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0);
	
private:
	
	/** \brief Trains the DMP weights using LWR (Locally Weighted Regression) with biased and trunckated kernels
	 *  The k-th weight is set to w_k = (s'*Psi*Fd) / (s'*Psi*s), 
	 *  where Psi = exp(-h(k)*(x-c(k)).^2)
	 *  @param[in] x: Row vector with the values of the phase variable.
	 *  @param[in] Fd: Row vector with the desired values of the shape attractor.
	 */
	void train_LWR_DMP_plus(const arma::rowvec &x, arma::rowvec &Fd);
	
};
}

#endif  // DYNAMIC_MOVEMENT_PRIMITIVES_PLUS_H