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

class DMP_
{
public:
	int N_kernels; ///< number of kernels (basis functions)

	double a_z; ///< parameter 'a_z' relating to the spring-damper system
	double b_z; ///< parameter 'b_z' relating to the spring-damper system

	std::shared_ptr<as64::CanonicalSystem> can_sys_ptr; ///< handle (pointer) to the canonical system

	arma::vec w; ///< N_kernelsx1 vector with the weights of the DMP
	arma::vec c; ///< N_kernelsx1 vector with the kernel centers of the DMP
	arma::vec h; ///< N_kernelsx1 vector with the kernel stds of the DMP

	double zero_tol; ///< tolerance value used to avoid divisions with very small numbers

	double a_s; ///< scaling factor to ensure smaller changes in the accelaration to improve the training

	bool USE_GOAL_FILT; ///<
	double a_g; ///<
	
	double lambda; ///<
	double P_rlwr; ///<
	
  DMP_();

  /** \brief DMP constructor
	 *  @param[in] N_kernels: the number of kernels
	 *  @param[in] a_z: Parameter \a a_z relating to the spring-damper system.
	 *  @param[in] b_z: Parameter \a b_z relating to the spring-damper system.
	 *  @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
	 * @param[in] std_K: Scales the std of each kernel (optional, default = 1).
	 */ 
	 DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);

 
	/** \brief Initializes the DMP
	 * @param[in] N_kernels: the number of kernels
	 * @param[in] a_z: Parameter \a a_z relating to the spring-damper system.
	 * @param[in] b_z: Parameter \a b_z relating to the spring-damper system.
	 * @param[in] can_sys_ptr: Pointer to a DMP canonical system object.
	 * @param[in] std_K: Scales the std of each kernel (optional, default = 1).
	 */ 
	void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1);
	
	/** \brief Sets the centers for the activation functions of the DMP according to the partition method specified
	 *  @param[in] part_type: Partitioning method for the kernel centers (optional, default = "").
	 */ 
	void set_centers(const std::string  &part_type="");
	
	/** \brief Sets the standard deviations for the activation functions  of the DMP
	 *  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
	 *  @param[in] s: Scales the variance of each kernel by 's' (optional, default = 1).
	 */ 
	void set_stds(double s=1);

	/** \brief Trains the DMP
   *  @param[in] Time: Row vector with the timestamps of the training data points.
   *  @param[in] yd_data: Row vector with the desired potition.
   *  @param[in] dyd_data: Row vector with the desired velocity.
   *  @param[in] ddyd_data: Row vector with the desired accelaration.
   *  @param[in] y0: Initial position.
   *  @param[in] g0: Target-goal position.
   *  @param[in] train_method: Method used to train the DMP weights.
   *  @param[in] USE_GOAL_FILT: flag indicating whether to use filtered goal (optional, default = false).
   *  @param[in] a_g: Parameter of the goal filter.
   *
   *  \note The timestamps in \a Time and the corresponding position,
   *  velocity and acceleration data in \a yd_data, \a dyd_data and \a
   *  ddyd_data need not be sequantial in time.
	 */
    double train(const arma:: rowvec &Time, const arma::rowvec &yd_data, const arma::rowvec &dyd_data, 
								 const arma::rowvec &ddyd_data, double y0, double g0, const std::string &train_method,
								 arma::rowvec *Fd_ptr=NULL, arma::rowvec *F_ptr=NULL);
	
  
		void set_training_params(bool USE_GOAL_FILT, double a_g, double lambda, double P_rlwr);

	/** \brief Returns the scaling factor of the forcing term.
   *	@param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
   *	@param[in] y0: initial position.
   *	@param[in] g0: final goal.
   *	@param[out] s: The scaling factor of the forcing term.
   */
  virtual double forcing_term_scaling(double u, double y0, double g0) = 0;
	
  /** Returns the shape attractor of the DMP.
	 *  @param[in] x: The phase variable.
	 *  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
	 *  @param[in] y0: initial position.
	 *  @param[in] g0: final goal.
	 *  @param[out] shape_attr: The shape_attr of the DMP.
   */
  virtual double shape_attractor(const arma::vec &X, double g0, double y0) = 0;

  /** \brief Returns the goal attractor of the DMP.
	 *  @param[in] y: \a y state of the DMP.
	 *  @param[in] z: \a z state of the DMP.
	 *  @param[in] g: Goal position.
	 *  @param[out] goal_attr: The goal attractor of the DMP.
   */
  virtual double goal_attractor(double y, double z, double g);


  /** \brief Returns the forcing term of the DMP
   *  @param[in] x: The phase variable.
   *  @param[out] f: The normalized weighted sum of Gaussians.
   */
  virtual double forcing_term(double x);
	
  // Returns a vector with the activation functions for the DMP
  arma::vec activation_function(double x);

  double get_tau();
  double get_v_scale();

protected:
	void init_helper(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K = 1)
	{
		this->zero_tol = 1e-250;

    this->N_kernels = N_kernels;
    this->a_z = a_z;
    this->b_z = b_z;
    this->can_sys_ptr = can_sys_ptr;

    double tau = this->get_tau();
    if (tau > 1) this->a_s = 1 / (std::pow(this->can_sys_ptr->get_tau(),2));
    else this->a_s = 1;

    this->w = arma::vec().zeros(this->N_kernels); // rand(this->N_kernels,1);
    this->set_centers();
    this->set_stds(std_K);
		
		this->USE_GOAL_FILT = false;
		this->a_g = 0;
		this->lambda = 0.95;
		this->P_rlwr = 1000;
 		
	}
	
  /** Calculates the desired value of the scaled forcing term.
    *  @param[in] y: Position.
    *  @param[in] dy: Velocity.
    *  @param[in] ddy: Acceleration.
    *  @param[in] u: multiplier of the forcing term ensuring its convergens to zero at the end of the motion.
    *  @param[in] y0: initial position.
    *  @param[in] g0: final goal.
    *  @param[in] g: current goal (if for instance the transition from y0 to g0 is done using a filter).
    *  @param[out] Fd: Desired value of the scaled forcing term.
		*/ 
	virtual double calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0) = 0;

private:
	void train_LWR(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd);
	
	void train_RLWR(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd);
	
	void train_LS(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd);

};

}  // namespace as64

#endif  // DS_DYNAMICAL_SYSTEM_GMR_H
