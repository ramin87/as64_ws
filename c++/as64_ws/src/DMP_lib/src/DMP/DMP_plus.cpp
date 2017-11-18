/**
 * Copyright (C) 2017 DMP 
 */

#include <DMP_lib/DMP/DMP_plus.h>

namespace as64
{

DMP_plus::DMP_plus()
{}

DMP_plus::DMP_plus(int N_kernels, double a_z, double b_z, 
									 std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K):
DMP_(N_kernels, a_z, b_z, can_sys_ptr, std_K)
{}

void DMP_plus::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K)
{
	this->init_helper(N_kernels, a_z, b_z, can_sys_ptr, std_K);
	
	this->k_trunc_kernel = 3;
	this->b = arma::vec().zeros(this->N_kernels);
}

double DMP_plus::train(const arma:: rowvec &Time, const arma::rowvec &yd_data, const arma::rowvec &dyd_data, 
												const arma::rowvec &ddyd_data, double y0, double g0, const std::string &train_method,
												arma::rowvec *Fd_ptr, arma::rowvec *F_ptr)
{
	int n_data = Time.size();
	double tau = this->can_sys_ptr->get_tau();

	arma::mat X = this->can_sys_ptr->get_continuous_output(Time);
	arma::rowvec x, u;
	
	if (X.n_rows == 1){
		x = X;
		u = x;
	}else{
		x = X.row(0);
		u = X.row(1);
	}

	arma::rowvec g = g0 * arma::rowvec().ones(x.size());
	if (USE_GOAL_FILT){
		g = y0*arma::exp(-a_g*Time/tau) + g0*(1 - arma::exp(-a_g*Time/tau));
	}

	arma::rowvec Fd(n_data);
	
	for (int i=0; i<n_data; i++){
		Fd(i) = this->calc_Fd(yd_data(i), dyd_data(i), ddyd_data(i), u(i), g(i), g0, y0);
	}

	if (train_method.compare("LWR"))
	{
		this->train_LWR_DMP_plus(x, Fd);
	}
	else
	{
		throw std::invalid_argument(std::string("Unsopported training method \"") + train_method + "\"");
	}
	
	arma::rowvec F(n_data);
	for (int i=0; i<n_data; i++)
	{
		F(i) = this->forcing_term(x(i));
	}
	
	if (Fd_ptr){
		*Fd_ptr = Fd;
	}
	if (F_ptr){
		*F_ptr = F;
	}

	double train_error = arma::norm(F-Fd)/F.size();
	
	return train_error;
}

double DMP_plus::forcing_term(double x)
{
	arma::vec Psi = this->activation_function(x);
	
	double f = arma::dot(this->w*x + this->b, Psi) / (arma::sum(Psi) + this->zero_tol);

	return f;
}

double DMP_plus::forcing_term_scaling(double u, double y0, double g0)
{
	double s = g0 - y0;
	
	return s;
}

double DMP_plus::shape_attractor(const arma::vec &X, double g0, double y0)
{
  double x,u;
  
  x = X(0);
  
  if (X.n_elem == 1) u = x;
  else u = X(1);

  double s_attr = this->forcing_term(x) * this->forcing_term_scaling(u, y0, g0);

  return s_attr;
}

arma::vec DMP_plus::activation_function(double x)
{
  arma::vec psi(this->N_kernels);
	
	arma::vec t = this->h%(arma::pow((x-this->c), 2));
	psi = arma::exp(-t);
	for (int i=0; i<this->N_kernels; i++)
	{
		if(t(i) > 2*std::pow(this->k_trunc_kernel, 2))
		{
				psi(i) = 0;
		}
	}
	//psi(t>2*std::pow(this->k_trunc_kernel, 2)) = 0;

	return psi;
}

double DMP_plus::calc_Fd(double y, double dy, double ddy, double u, double g, double g0, double y0)
{
  double v_scale = this->get_v_scale();
	
  double g_attr = -this->goal_attractor(y, v_scale*dy, g);
  double Fd = (ddy * std::pow(v_scale, 2) + g_attr) / (g0 - y0);
	
	return Fd;
}

void DMP_plus::train_LWR_DMP_plus(const arma::rowvec &x, arma::rowvec &Fd)
{
	int n_data = x.n_elem;
	arma::mat Psi(this->N_kernels, n_data);
	
	for(int i = 0; i < n_data; i++)
	{
		Psi.col(i) = 	this->activation_function(x(i));
	}

	arma::rowvec psi;
	double Sw, Sx, Sx2, Sxy, Sy;
	arma::mat A, W;
	arma::vec b;
	
	for (int k=0; k<this->N_kernels; k++)
	{
			psi = Psi.col(k);
			Sw = sum(psi); // + this->zero_tol?
			Sx = arma::dot(psi,x);
			Sx2 = arma::dot(psi, arma::square(x));
			Sxy = arma::dot(psi, x%Fd);
			Sy = arma::dot(psi,Fd);
			A << Sx2 << Sx << arma::endr << Sx << Sw;
			b << Sxy << arma::endr << Sy;
			W = arma::pinv(A) * b;
			this->w(k) = W(1);
			this->b(k) = W(2);
	}
}

} //as64
