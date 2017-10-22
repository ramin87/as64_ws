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
	
/** \class CanonicalSystem
 *  \brief CanonicalSystem class.
 * 
 *  Implements an abstact canonical system, which is used as a clocking mechanism for a DMP.
 */

class CanonicalSystem
{
public:
  
  /** \brief Constructor of the canonical system.
   */
  CanonicalSystem();
  
  /** \brief Initializes the canonical system.
   *  \param[in] x_end value of phase variable at the end of the motion
   *  \param[in] tau cycle time of the canonical system (can be used to scale temporally the motion)
   */
  void init(double x_end, double tau);
    
  /** \brief Returns the cycle time of the canonical system.
   */
  double get_tau();
  
  /** \brief Sets the cycle time of the canonical system.
   *  \param[in] t time scale factor of the canonical system
   */
  void set_tau(double t);
  
  /** \brief Sets the canonical system's time constants based on the value of the phase variable at the end of the movement.
   *  \param[in] x_end value of the phase variable at the end of the cycle time (t = tau)
   */
  virtual void set_can_sys_params(double x_end) = 0;

  /** \brief Returns the derivative of the canonical system.
   *  \param[in] X current value of the phase variable
   *  \return the derivative of the canonical system
   */
  virtual arma::vec get_derivative(const arma::vec &X) = 0;

  /** \brief Returns the output of the canonical system for a continuous time interval.
   *  \param[in] t a row vector with the time instances at which the canonical system's output is to be computed
   *  \param[in] x0 initial value of the phase variable (default = 1)
   *  \return the derivative of the canonical system at time instances t.
   */
  virtual arma::mat get_continuous_output(const arma::rowvec &t, double x0) = 0;

private:
  double tau; ///< cycle time of the canonical system
};

}  // namespace as64

#endif  // DMP_CANONICAL_SYSTEM_64_H
