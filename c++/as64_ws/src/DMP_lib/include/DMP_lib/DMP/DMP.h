/**
 * Copyright (C) 2017 DMP
 */


/** DMP class
 *  Implements an 1-D DMP.
 *
 */

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_2002_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_2002_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <DMP_lib/DMP/DMPBase.h>

namespace as64
{

class DMP:public DMPBase
{
public:

  DMP();

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
  DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K=1, bool USE_GOAL_FILT=false, double a_g=0);
  
  //Returns the scaling of the forcing term
  arma::vec forcing_term_scaling(arma::rowvec &u, double y0, double g0);
  
  double forcing_term_scaling(double u, double y0, double g0);
  
  // Returns the shape-attractor of the DMP
  double shape_attractor(const arma::vec X, double g0, double y0);

private:
  // calculates the desired force for the demonstraded path
  void calculate_Fd(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, arma::mat &u, arma::rowvec &g, double g0, double y0, arma::rowvec &Fd);

};

}  // namespace as64

#endif  // DS_DYNAMICAL_SYSTEM_GMR_H
