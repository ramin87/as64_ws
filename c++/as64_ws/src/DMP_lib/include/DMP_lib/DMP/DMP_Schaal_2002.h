/**
 * Copyright (C) 2017 DS
 */


/** DMP class
 *  Implements an 1-D DMP.
 *  The DMP is driven by the canonical system:
 *  dx = -ax*x/ts
 *  where x is the phase variable and ax the decay term.
 * 
 *  The DMP has the following form:
 *
 *     ddy = ts^2 * ( a_z*(b_z*(g-y)-dy/ts) + f*x*(g-y0) ); 
 *
 *  where
 *     ts: is scaling factor defining the duration of the motion
 *     a_z, b_z: constants relating to a spring-damper system
 *     g: the goal position
 *     y0: the initial position
 *    x: the phase variable
 *     y,dy,ddy: the position, velocity and accelaration of the motion
 *     f: the forcing term defined by the weighted sum of the activation
 *        functions (gaussian kernels), i.e.: 
 *        f = w'*Psi/ sum(Psi);
 *     
 *  The output of the DMP can be writtern compactly in the following form:
 *     ddy = ts^2 * ( goal_attractor(y,dy,g) + shape_attractor(x,g,y0) );
 *     
 *   where:
 *     goal_attractor(y,dy,g) = a_z*(b_z*(g-y)-dy/ts)
 *     shape_attractor(x,g,y0) = f*x*(g-y0)
 *
 */

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_2002_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_2002_H

#include <cmath>
#include <vector>
#include <cstring>
#include <exception>
#include <armadillo>

namespace as64
{
	
class DMP
{
public:
  
  int N_kernels; // number of kernels (basis functions)

  double a_z; // parameter 'a_z' relating to the spring-damper system
  double b_z; // parameter 'b_z' relating to the spring-damper system

  double ax; // the decay factor of the phase variable
  double ts; // movement duration (can be used to scale temporally the motion)

  arma::vec w; // N_kernelsx1 vector with the weights of the DMP
  arma::vec c; // N_kernelsx1 vector with the kernel centers of the DMP
  arma::vec h; // N_kernelsx1 vector with the kernel stds of the DMP

  DMP();

  // initializes a DMP
  // N_kernels: contains the number of kernels
  // a_z: param 'a_z'
  // b_z: param 'b_z'
  // ax: the decay factor of the phase variable
  // centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
  // std_K: scales the std of each kernel
  DMP(int N_kernels, double a_z, double b_z, double ts, double ax, const std::string &centers_part_type="exp", double std_K=1);

  // initializes a DMP
  // N_kernels: contains the number of kernels
  // a_z: param 'a_z'
  // b_z: param 'b_z'
  // ax: the decay factor of the phase variable
  // centers_part_type: method to partition the centers of the activation function kernels (supported: {'lin', 'exp'})
  // std_K: scales the std of each kernel
  void init(int N_kernels, double a_z, double b_z, double ts, double ax, const std::string &centers_part_type="exp", double std_K=1);

  // Sets the centers for the activation functions  of the DMP
  void set_centers(const std::string &part_type);

  // Sets the standard deviations for the activation functions  of the DMP
  void set_stds(double s=1);

  // Trains the DMP
  double train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method);

  // Returns the shape-attractor of the DMP
  double shape_attractor(double x, double g, double y0);

  // Returns the goal-attractor of the DMP
  double goal_attractor(double y, double dy, double g);

  // Returns the output of the DMP
  double get_output(double y, double dy, double g, double y0, double x);

  // Returns the forcing term of the DMP
  double forcing_term(double x);

  // Returns a vector with the activation functions for the DMP
  arma::vec activation_function(double x);

  void set_a_z(double a);
  void set_b_z(double a);
  void set_ax(double a);
  void set_ts(double t);

  double get_a_z() const;
  double get_b_z() const;
  double get_ax() const;
  double get_ts() const;

private:
  /*
  int N_kernels; // number of kernels (basis functions)

  double a_z; // parameter 'a_z' relating to the spring-damper system
  double b_z; // parameter 'b_z' relating to the spring-damper system

  double ax; // the decay factor of the phase variable
  double ts; // movement duration (can be used to scale temporally the motion)

  arma::vec w; // N_kernelsx1 vector with the weights of the DMP
  arma::vec c; // N_kernelsx1 vector with the kernel centers of the DMP
  arma::vec h; // N_kernelsx1 vector with the kernel stds of the DMP
*/
  long double zero_tol; // tolerance value used to avoid divisions with very small numbers
};

}  // namespace as64

#endif  // DS_DYNAMICAL_SYSTEM_GMR_H
