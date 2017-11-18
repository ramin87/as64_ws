/**
 * Copyright (C) 2017 CAN_SYS
 */

#ifndef DMP_CANONICAL_SYSTEM_64_H
#define DMP_CANONICAL_SYSTEM_64_H

#include <cmath>
#include <vector>
#include <cstring>
#include <exception>
#include <armadillo>

namespace as64
{
	
/**
 *  \brief CanonicalSystem class, used as a clocking mechanism for a DMP.
 * 
 * The canonical system is used as a clocking mechanism for a DMP. 
 * It implements a function \f$ x=f(t) \f$,  producing the so called phase variable \f$ x \f$ at each time step \f$ t \f$. 
 * \f$ x \f$ is used in a DMP instead of explicitly using the time \f$ t \f$. 
 * The function \f$ x = f(t) \f$ starts at \f$ x_0 = f(t=0) \f$ and monotonically decreases to \f$ x_{end} = f(t=\tau ) \f$, 
 * where \f$ \tau \f$ is typically the DMP's movement duration.
 */

class CanonicalSystem
{
public:
  
  /** \brief Constructor of the canonical system.
   */
  CanonicalSystem();
  
  /** \brief Initializes the canonical system.
   *  @param[in] x_end Value of the phase variable \f$ x \f$ at \f$ t = \tau \f$.
   *  @param[in] tau The time \f$ \tau \f$, at which \f$ x( \tau ) = x_{end} \f$.
   *  \param[in] x0 The value of the phase variable \f$ x \f$ at \f$ t=0 \f$ (default = 1).
   */
  void init(double x_end, double tau, double x0 = 1);
    
  /** \brief Returns the time at which the phase variable reaches its final value, i.e. \f$ x( \tau ) = x_{end} \f$.
   */
  double get_tau();
  
  /** \brief Sets the time at which the phase variable reaches its final value, i.e. \f$ x( \tau ) = x_{end} \f$.
   *  @param[in] tau The time at which the phase variable reaches its final value.
   */
  void set_tau(double tau);
  
  /** \brief Sets the canonical system's time constants based on the value of the phase variable at \f$ t = \tau \f$.
   *  @param[in] x_end Value of the phase variable at \f$ t = \tau \f$.
   */
  virtual void set_can_sys_params(double x_end) = 0;

  /** \brief Returns the derivative of the canonical system.
   *  @param[in] X Current value of the phase variable.
   *  @return The derivative of the canonical system.
   */
  virtual arma::vec get_derivative(const arma::vec &X) = 0;

  /** \brief Returns the output of the canonical system for a continuous time interval.
   *  @param[in] t Row vector with the time instances at which the canonical system's output is to be computed.
   *  @return The derivative of the canonical system at time instances t.
   */
  virtual arma::mat get_continuous_output(const arma::rowvec &t) = 0;

protected:
  double tau; ///< the time \f$ \tau \f$, at which \f$ x( \tau ) = x_{end} \f$.
  double x0; ///< initial value of the phase variable \f$ x \f$.
};

}  // namespace as64

#endif  // DMP_CANONICAL_SYSTEM_64_H
