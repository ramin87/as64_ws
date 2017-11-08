/**
* Copyright (C) 2017 DMP
*/


/** DMP class
*  Implements an 1-D bio-inspired DMP. 
*
*/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_BIO_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_BIO_H

#include <DMP_lib/DMP/DMP_.h>

namespace as64
{

class DMP_bio:public DMP_
{
public:

  DMP_bio();

	/** \brief Bioinspired DMP constructor
	 *  @param[in] N_kernels: the number of kernels
	 *  @param[in] a_z: Parameter \a a_z relating to the spring-damper system.
	 *  @param[in] b_z: Parameter \a b_z relating to the spring-damper system.
	 *  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
	 * @param[in] std_K: Scales the std of each kernel (optional, default = 1).
	 */ 
  DMP_bio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);
  
  double forcing_term_scaling(double u, double y0, double g0);
  
  double shape_attractor(const arma::vec &X, double g0, double y0);

  double calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0);

};
}  //as64

#endif  // DYNAMIC_MOVEMENT_PRIMITIVES_BIO_H
