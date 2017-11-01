/**
 * Copyright (C) 2017 DMP
 */


/** DMP class
 *  Implements DMP Base Class.
 *
 */

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_BASE_2002_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_BASE_2002_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <DMP_lib/CanonicalSystem/CanonicalSystem.h>

namespace as64
{

class DMPBase
{
public:

  DMPBase();

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
  DMPBase(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K=1, bool USE_GOAL_FILT=false, double a_g=0);

  // initializes a DMP
  // N_kernels: contains the number of kernels
  // a_z: param 'a_z'
  // b_z: param 'b_z'
  // ax: the decay factor of the phase variable
  // centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
  // std_K: scales the std of each kernel
  void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K=1, bool USE_GOAL_FILT=false, double a_g=0);

  // Sets the centers for the activation functions  of the DMP
  void set_centers(const std::string &part_type="");

  // Sets the standard deviations for the activation functions  of the DMP
  void set_stds(double s=1);

  // Trains the DMP
  double train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method, arma::rowvec &F_train, arma::rowvec &Fd_train);

  double train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method);
  
  //Returns the scaling of the forcing term
  virtual arma::vec forcing_term_scaling(arma::rowvec &u, double y0, double g0) = 0;

  virtual double forcing_term_scaling(double u, double y0, double g0) = 0;
  
  // Returns the shape-attractor of the DMP
  virtual double shape_attractor(const arma::vec X, double g0, double y0) = 0;

  // Returns the goal-attractor of the DMP
  double goal_attractor(double y, double dy, double g);

  // Returns the output of the DMP
  double get_output(double y, double dy, double g, double y0, const arma::vec &X);

  // Returns the forcing term of the DMP
  double forcing_term(double x);

  // Returns a vector with the activation functions for the DMP
  arma::vec activation_function(double x);

  void set_tau(double t);
  void set_a_z(double a);
  void set_b_z(double a);

  double get_a_z() const;
  double get_b_z() const;
  double get_tau();
  double get_v_scale();
  
  double a_z; // parameter 'a_z' relating to the spring-damper system
  double b_z; // parameter 'b_z' relating to the spring-damper system
  
  double g0; // goal of the DMP
  
  arma::vec w; // N_kernelsx1 vector with the weights of the DMP
  arma::vec c; // N_kernelsx1 vector with the kernel centers of the DMP
  arma::vec h; // N_kernelsx1 vector with the kernel stds of the DMP

private:
  // calculates the desired force for the demonstraded path
  virtual void calculate_Fd(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, arma::mat &u, arma::rowvec &g, double g0, double y0, arma::rowvec &Fd) = 0;

  int N_kernels; // number of kernels (basis functions)

  std::shared_ptr<CanonicalSystem> can_sys_ptr; // handle (pointer) to the canonical system

  bool USE_GOAL_FILT; // flag determining whether to use goal filtering
  double a_g; // time contant determining how fast 'g' converges to 'g0' (the higher 'a_g'
             // is, the faster the convergence).
      
  double a_s; // scaling factor to ensure smaller changes in the accelaration
  
  long double zero_tol; // tolerance value used to avoid divisions with very small numbers
};

}  // namespace as64

#endif  // DS_DYNAMICAL_SYSTEM_GMR_H
