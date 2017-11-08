/**
* Copyright (C) 2017 DMP
*/


/** DMP class
*  Implements an 1-D bio-inspired DMP. 
*
*/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H

#include <DMP_lib/DMP/DMP_.h>

namespace as64
{

class DMPBio:public DMP_
{
public:

  DMPBio();

  /** \brief initializes a DMP
   *  \param[in] a_z parameter a_z of the linear (spring-damper) part of the DMP
   *  \param[in] b_z parameter b_z of the linear (spring-damper) part of the DMP
   * 
   * 
   */
  // 
  // N_kernels: contains the number of kernels
  // a_z: param 'a_z'
  // b_z: param 'b_z'
  // ax: the decay factor of the phase variable
  // centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
  // std_K: scales the std of each kernel
  DMPBio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);
  
  //Returns the scaling of the forcing term
  double forcing_term_scaling(double u, double y0, double g0);
  
  // Returns the shape-attractor of the DMP
  double shape_attractor(const arma::vec &X, double g0, double y0);

private:
  // calculates the desired force for the demonstraded path
  double calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0);

};
}  //as64

#endif  // DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H
