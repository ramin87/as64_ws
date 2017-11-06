/**
 * Copyright (C) 2017 DMP
 */

#include <DMP_lib/DMP/DMPBase.h>

namespace as64
{

DMPBase::DMPBase()
{}

DMPBase::DMPBase(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K, bool USE_GOAL_FILT, double a_g)
{
  init(N_kernels, a_z, b_z, cs_ptr, std_K, USE_GOAL_FILT, a_g); 
}

void DMPBase::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K, bool USE_GOAL_FILT, double a_g)
{
  this->N_kernels = N_kernels;
  this->a_z = a_z;
  this->b_z = b_z;
  this->can_sys_ptr = cs_ptr;
  
  double tau = this->get_tau();
  
  if (tau > 1) this->a_s = 1 / (std::pow(tau,2));
  else this->a_s = std::pow(tau,2);
          
  this->w.randu(this->N_kernels);
  this->set_centers();
  this->set_stds(std_K);

  this->zero_tol = 1e-250;
          
  if (USE_GOAL_FILT){
    this->USE_GOAL_FILT = true;
    this->a_g = a_g;
  }
  else{
		this->USE_GOAL_FILT = false;
	}

}

void DMPBase::set_centers(const std::string &part_type)
{
	int n_centers = this->N_kernels;
	
	arma::rowvec t = ( arma::linspace<arma::rowvec>(1,n_centers,n_centers) - 1 ) / (n_centers-1);
	
	if ( part_type.compare("lin") == 0 ) this->c = t.t();
	//else if ( part_type.compare("exp") == 0 ) this->c = arma::exp(-this->ax*t);
	//else throw std::invalid_argument(std::string("Unsupported partition type: ") + part_type);
	else{
		double x0 = 1;
		double tau = get_tau();
		arma::mat X = can_sys_ptr->get_continuous_output(t*tau, x0);
		
		this->c = X.row(0).t();
	}    
}


void DMPBase::set_stds(double s)
{
	int n = this->c.n_elem;
	this->h.resize(n);
	
	for (int i=0;i<n-1;i++)
			this->h(i) = s/std::pow((this->c(i+1) - this->c(i)), 2);
			
	this->h(n-1) = this->h(n-2);
}


double DMPBase::train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method)
{
	int n_data = yd_data.n_elem;

	arma::rowvec t = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
	this->set_tau(t(n_data-1));
	double g0 = yd_data(n_data-1);
	double y0 = yd_data(0);
	double x0 = 1;
	double tau = get_tau();
	double v_scale = this->get_v_scale();
	
	arma::mat X = this->can_sys_ptr->get_continuous_output(t, x0);
	arma::rowvec x;
	arma::rowvec u;
	
	if (X.n_rows == 1){
		x = X;
		u = x;
	}else{
		x = X.row(0);
		u = X.row(1);
	}
	
	//arma::vec s = forcing_term_scaling(u, y0, g0);
	arma::rowvec g = g0 * arma::rowvec().ones(t.n_elem);
	
	if (this->USE_GOAL_FILT){
		arma::rowvec exp_t = arma::exp(-this->a_g*t/tau);
		g = y0*exp_t + g0*(1-exp_t);
	}
	
	arma::rowvec y0_vec = arma::rowvec().ones(g.n_elem) * y0;
	
	arma::vec s = forcing_term_scaling(u, y0_vec, g);
	
	arma::rowvec Fd(t.n_elem);
	//arma::rowvec ddzd_data = ddyd_data*std::pow(v_scale,2);
	//arma::rowvec g_attr_data = -this->a_z*(this->b_z*(g-yd_data)-dyd_data*v_scale);
	//arma::rowvec Fd = (ddzd_data + g_attr_data);
	
	this->calculate_Fd(yd_data, dyd_data, ddyd_data, u, g, g0, y0, Fd);

	if (train_method.compare("LWR") == 0){
			
			arma::mat Psi;
			
			for (int k=0;k<this->N_kernels;k++){
					//Psi = arma::diagmat( arma::exp(-this->h(k)*arma::pow(x-this->c(k),2)) );
		//this->w(k) = arma::as_scalar(s.t()*Psi*Fd.t()) / arma::as_scalar(s.t()*Psi*s + this->zero_tol);
		
		Psi = arma::exp(-this->h(k)*arma::pow(x-this->c(k),2));
		arma::rowvec temp = s.t()%Psi;
		this->w(k) = arma::dot(temp,Fd) / (arma::dot(temp,s) + this->zero_tol);
			}
			
	}else if (train_method.compare("LS") == 0){

			arma::mat H = arma::zeros(this->N_kernels, n_data);
			arma::vec Psi;
			for (int i=0;i<n_data;i++){
					Psi = arma::exp(-this->h % arma::pow(-this->c + x(i), 2));
					Psi = s(i)*Psi / (arma::sum(Psi) + this->zero_tol);
					H.col(i) = Psi;
			}

			this->w = (Fd*arma::pinv(H)).t();
			
	}else{   
			throw std::invalid_argument(std::string("Unsopported training method: ") + train_method);
	}
	
	arma::rowvec F = arma::zeros<arma::rowvec>(Fd.n_elem);
			
	for (int i=0;i<F.n_elem;i++)
			F(i) = this->shape_attractor(X.col(i),g0,y0);
	
	double train_error = arma::norm(F-Fd)/F.n_elem;

	return train_error;
}

double DMPBase::train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method, arma::rowvec &F_train, arma::rowvec &Fd_train)
{
	int n_data = yd_data.n_elem;

	arma::rowvec t = arma::linspace<arma::rowvec>(0,n_data-1,n_data)*Ts;
	this->set_tau(t(n_data-1));
	double g0 = yd_data(n_data-1);
	double y0 = yd_data(0);
	double x0 = 1;
	double tau = get_tau();
	double v_scale = this->get_v_scale();

	arma::mat X = this->can_sys_ptr->get_continuous_output(t, x0);
	arma::rowvec x;
	arma::rowvec u;

	if (X.n_rows == 1){
		x = X;
		u = x;
	}else{
		x = X.row(0);
		u = X.row(1);
	}

	//arma::vec s = forcing_term_scaling(u, y0, g0);
	arma::rowvec g = g0 * arma::rowvec().ones(t.n_elem);

	if (this->USE_GOAL_FILT){
		arma::rowvec exp_t = arma::exp(-this->a_g*t/tau);
		g = y0*exp_t + g0*(1-exp_t);
	}

	arma::rowvec y0_vec = arma::rowvec().ones(g.n_elem) * y0;

	arma::vec s = forcing_term_scaling(u, y0_vec, g);


	arma::rowvec Fd(t.n_elem);
	//arma::rowvec ddzd_data = ddyd_data*std::pow(v_scale,2);
	//arma::rowvec g_attr_data = -this->a_z*(this->b_z*(g-yd_data)-dyd_data*v_scale);
	//Fd = (ddzd_data + g_attr_data);

	this->calculate_Fd(yd_data, dyd_data, ddyd_data, u, g, g0, y0, Fd);

	if (train_method.compare("LWR") == 0){
			
			arma::mat Psi;
			
			for (int k=0;k<this->N_kernels;k++){
					//Psi = arma::diagmat( arma::exp(-this->h(k)*arma::pow(x-this->c(k),2)) );
				//this->w(k) = arma::as_scalar(s.t()*Psi*Fd.t()) / arma::as_scalar(s.t()*Psi*s + this->zero_tol);
				
				Psi = arma::exp(-this->h(k)*arma::pow(x-this->c(k),2));
				arma::rowvec temp = s.t()%Psi;
				this->w(k) = arma::dot(temp,Fd) / (arma::dot(temp,s) + this->zero_tol);
			}
			
	}else if (train_method.compare("LS") == 0){

			arma::mat H = arma::zeros(this->N_kernels, n_data);
			arma::vec Psi;
			for (int i=0;i<n_data;i++){
					Psi = arma::exp(-this->h % arma::pow(-this->c + x(i), 2));
					Psi = s(i)*Psi / (arma::sum(Psi) + this->zero_tol);
					H.col(i) = Psi;
			}

			this->w = (Fd*arma::pinv(H)).t();
			
	}else{   
			throw std::invalid_argument(std::string("Unsopported training method: ") + train_method);
	}

	arma::rowvec F = arma::zeros<arma::rowvec>(Fd.n_elem);
			
	for (int i=0;i<F.n_elem;i++)
			//F(i) = this->shape_attractor(X.col(i),g0,y0);
			F(i) = this->forcing_term(x(i)) *this->forcing_term_scaling(u(i), y0, g(i));

	double train_error = arma::norm(F-Fd)/F.n_elem;

	F_train = F;
	Fd_train = Fd;

	return train_error;
}

double DMPBase::goal_attractor(double y, double dy, double g)
{
  double v_scale = this->get_v_scale();
  
  double g_attr = this->a_z*(this->b_z*(g-y)-dy*v_scale);

  return g_attr;
}


double DMPBase::get_output(double y, double dy, double g, double y0, const arma::vec &X)
{
  double v_scale = this->get_v_scale();
  
  double dmp_out = ( this->goal_attractor(y,dy,g) + this->shape_attractor(X,g,y0) ) / std::pow(v_scale,2);
    
  return dmp_out;
}


double DMPBase::forcing_term(double x)
{
  arma::vec Psi = this->activation_function(x);
  double f = arma::dot(Psi,this->w) / (arma::sum(Psi)+this->zero_tol); // add zero_tol to avoid numerical issues
    
  return f;
}


arma::vec DMPBase::activation_function(double x)
{
  arma::vec psi = arma::exp(-this->h % arma::pow(x-this->c,2));

  return psi;
}
  
void DMPBase::set_a_z(double a)
{
  this->a_z = a;
}

void DMPBase::set_b_z(double a)
{
  this->b_z = a;
}

void DMPBase::set_tau(double t)
{
  can_sys_ptr->set_tau(t);
}

double DMPBase::get_a_z() const
{
  return this->a_z;
}

double DMPBase::get_b_z() const
{
  return this->b_z;
}
    
double DMPBase::get_tau()
{
  return can_sys_ptr->get_tau();
}

double DMPBase::get_v_scale()
{
  return get_tau() * a_s;
}

}  // namespace as64
