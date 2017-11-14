/**
 * Copyright (C) 2017 DMP
 */

#include <DMP_lib/DMP/DMP_.h>

namespace as64
{
	
  DMP_::DMP_()
	{}

	 DMP_::DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K)
	 {
		 this->init(N_kernels, a_z, b_z, can_sys_ptr, std_K);
	 }

	void DMP_::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K)
	{
		this->init_helper(N_kernels, a_z, b_z, can_sys_ptr, std_K);
	}
	
	void DMP_::init_helper(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> can_sys_ptr, double std_K)
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

	void DMP_::set_centers(const std::string  &part_type)
	{
		arma::rowvec t = arma::linspace<arma::rowvec>(0,this->N_kernels-1,this->N_kernels)/(this->N_kernels-1);

    if (part_type.compare("lin")){
        this->c = t.t();
		}else if (part_type.compare("")){
        double x0 = 1;
        arma::mat x = this->can_sys_ptr->get_continuous_output(t*this->can_sys_ptr->get_tau(), x0);
        this->c = x.row(0).t();
		}else{
        throw std::invalid_argument(std::string("Unsupported partition type ") + part_type);
		}
		
	}

	void DMP_::set_stds(double s)
	{
		int n = this->c.n_elem;
		this->h.resize(n);
		
		for (int i=0;i<n-1;i++){
			this->h(i) = 1/std::pow(s*(this->c(i+1) - this->c(i)), 2);
		}
				
		this->h(n-1) = this->h(n-2);
	}

	double DMP_::train(const arma:: rowvec &Time, const arma::rowvec &yd_data, const arma::rowvec &dyd_data, 
										const arma::rowvec &ddyd_data, double y0, double g0, const std::string &train_method,
										arma::rowvec *Fd_ptr, arma::rowvec *F_ptr)
	{
		int n_data = Time.size();
		double x0 = 1;
		double tau = this->can_sys_ptr->get_tau();

		arma::mat X = this->can_sys_ptr->get_continuous_output(Time, x0);
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

		arma::rowvec s(n_data);
		arma::rowvec Fd(n_data);
		
		for (int i=0; i<n_data; i++){
			s(i) = this->forcing_term_scaling(u(i), y0, g0);
			Fd(i) = this->calc_Fd(yd_data(i), dyd_data(i), ddyd_data(i), u(i), g(i), g0, y0);
		}

		if (train_method.compare("LWR"))
		{
			this->train_LWR(x, s, Fd);
		}
		else if (train_method.compare("RLWR"))
		{
			this->train_RLWR(x, s, Fd);
		}
		else if (train_method.compare("LS"))
		{
			this->train_LS(x, s, Fd);
		}
		else
		{
			throw std::invalid_argument(std::string("Unsopported training method \"") + train_method + "\"");
		}
		
		arma::rowvec F(n_data);
		for (int i=0; i<n_data; i++)
		{
			F(i) = this->forcing_term(x(i)) * this->forcing_term_scaling(u(i), y0, g0);
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
		
		
	double DMP_::goal_attractor(double y, double z, double g)
	{ 
		double g_attr = this->a_z * (this->b_z * ( g - y ) - z);
    return g_attr;       
	}
	
	double DMP_::forcing_term(double x)
	{
		arma::vec Psi = this->activation_function(x);
    
    double f = arma::dot(this->w, Psi) / (arma::sum(Psi) + this->zero_tol);
		
		return f;
	}
	
	arma::vec DMP_::activation_function(double x)
	{
    arma::vec Psi = arma::exp(-this->h % arma::pow((x - this->c), 2));
		return Psi;
	}

	arma::vec DMP_::get_states_dot(double y, double z, double x, double u, double y0, 
																 double g0, double g, double y_c, double z_c)
	{
		double v_scale = this->get_v_scale();
		
		arma::vec X_in(2);
		X_in(0) = x;
		X_in(1) = u;
		
		double shape_attr = this->shape_attractor(X_in, y0, g0);
		double goal_attr = this->goal_attractor(y, z, g);
						
		double dz = ( goal_attr + shape_attr + z_c) / v_scale;
		double dy = ( z + y_c) / v_scale;
		
		arma::vec dydz(2);
		
		dydz(0) = dy;
		dydz(1) = dz;
		
		return dydz;
	}
	
	void DMP_::update_weights(double x, double u, double y, double dy, double ddy, 
														double y0, double g0, double g, arma::vec &P, double lambda)
	{
		double Fd = this->calc_Fd(y, dy, ddy, u, y0, g0, g);
    double s = this->forcing_term_scaling(u, y0, g0);
		
    arma::vec Psi = this->activation_function(x);
				
		arma::vec error = Fd - this->w * s;
				
		P = (P - (P%P * std::pow(s, 2)) / (lambda / Psi + P * std::pow(s, 2))) / lambda;
				
		this->w = this->w + Psi % P % error * s;
	}
			
	void DMP_::set_training_params(bool USE_GOAL_FILT, double a_g, double lambda, double P_rlwr)
	{
			this->USE_GOAL_FILT = USE_GOAL_FILT;
			this->a_g = a_g;
			this->lambda = lambda;
			this->P_rlwr = P_rlwr;
	}
	
	void DMP_::train_LWR(const arma::rowvec &x, const arma::rowvec &s, arma::rowvec &Fd)
	{
		
		arma::rowvec Psi, temp;
		
		for (int k=0; k<this->N_kernels; k++)
		{
			Psi = arma::exp(-this->h(k)* arma::pow((x-this->c(k)),2));
			temp = s.t()%Psi;
			this->w(k) = arma::dot(temp, Fd) / (arma::dot(temp, s) + this->zero_tol);
		}
	}
	
	void DMP_::train_RLWR(const arma::rowvec &x, const arma::rowvec &s, arma::rowvec &Fd)
	{
		int n_data = x.n_elem;

		arma::vec P = arma::vec().ones(this->N_kernels) * this->P_rlwr;
		this->w = arma::vec().zeros(this->N_kernels);
  
		arma::vec Psi;
		arma::vec error;
		
		for (int i=0; i<n_data; i++)
		{	
				Psi = this->activation_function(x(i));
				
				error = Fd(i) - this->w * s(i);
				
				P = (P - (P%P * std::pow(s(i), 2)) / (this->lambda / Psi + P * std::pow(s(i), 2))) / this->lambda;
				
				this->w = this->w + Psi % P % error * s(i);
		}
	}
	
	void DMP_::train_LS(const arma::rowvec &x, const arma::rowvec &s, arma::rowvec &Fd)
	{
		int n_data = x.n_elem;
		
		arma::mat H = arma::zeros(this->N_kernels, n_data);
		arma::vec Psi;
		
		for (int i=0; i<n_data; i++)
		{
				Psi = this->activation_function(x(i));
				Psi = s(i)*Psi / (arma::sum(Psi) + this->zero_tol);
				H.col(i) = Psi;
		}

		this->w = (Fd*arma::pinv(H)).t();
	}
	
	double DMP_::get_tau()
	{
		double tau = this->can_sys_ptr->get_tau();
		return tau;
	}
	
	double DMP_::get_v_scale()
	{
		double v_scale = this->get_tau() * this->a_s;
		return v_scale;
	}
	
	/*
DMP_::DMP_()
{}

DMP_::DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K, bool USE_GOAL_FILT, double a_g)
{
  init(N_kernels, a_z, b_z, cs_ptr, std_K, USE_GOAL_FILT, a_g); 
}

void DMP_::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K, bool USE_GOAL_FILT, double a_g)
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

void DMP_::set_centers(const std::string &part_type)
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

void DMP_::set_stds(double s)
{
	int n = this->c.n_elem;
	this->h.resize(n);
	
	for (int i=0;i<n-1;i++)
			this->h(i) = s/std::pow((this->c(i+1) - this->c(i)), 2);
			
	this->h(n-1) = this->h(n-2);
}

double DMP_::train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method)
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
	arma::rowvec x;// namespace as64
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
	
	arma::vec s = forcing_term_scaling(u, y0, g0);
	
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

double DMP_::train(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double Ts, const std::string &train_method, arma::rowvec &F_train, arma::rowvec &Fd_train)
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

	arma::vec s = forcing_term_scaling(u, y0, g0);

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
			throw std::in     
      
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
valid_argument(std::string("Unsopported training method: ") + train_method);
	}

	arma::rowvec F = arma::zeros<arma::rowvec>(Fd.n_elem);
			
	for (int i=0;i<F.n_elem;i++)
			//F(i) = this->shape_attractor(X.col(i),g0,y0);
			F(i) = this->forcing_term(x(i)) * this->forcing_term_scaling(u(i), y0, g0);

	double train_error = arma::norm(F-Fd)/F.n_elem;

	F_train = F;
	Fd_train = Fd;

	return train_error;
}

double DMP_::goal_attractor(double y, double dy, double g)
{
  double v_scale = this->get_v_scale();
  
  double g_attr = this->a_z*(this->b_z*(g-y)-dy*v_scale);

  return g_attr;
}

arma::rowvec DMP_::goal_attractor(arma::rowvec &y, arma::rowvec &dy, arma::rowvec &g)
{
	double v_scale = this->get_v_scale();
  
  arma::rowvec g_attr = this// namespace as64->a_z*(this->b_z*(g-y)-dy*v_scale);

  return g_attr;
}

double DMP_::get_output(double y, double dy, double g, double y0, const arma::vec &X)
{
  double v_scale = this->get_v_scale();
  
  double dmp_out = ( this->goal_attractor(y,dy,g) + this->shape_attractor(X,g,y0) ) / std::pow(v_scale,2);
    
  return dmp_out;
}


double DMP_::forcing_term(double x)
{
  arma::vec Psi = this->activation_function(x);
  double f = arma::dot(Psi,this->w) / (arma::sum(Psi)+this->zero_tol); // add zero_tol to avoid numerical issues
    
  return f;
}


arma::vec DMP_::activation_function(double x)
{
  arma::vec psi = arma::exp(-this->h % arma::pow(x-this->c,2));

  return psi;
}

void DMP_::train_LWR(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd)
{
	arma::mat Psi;
			
	for (int k=0;k<this->N_kernels;k++){
			//Psi = arma::diagmat( arma::exp(-this->h(k)*arma::pow(x-this->c(k),2)) );
		//this->w(k) = arma::as_scalar(s.t()*Psi*Fd.t()) / arma::as_scalar(s.t()*Psi*s + this->zero_tol);
		
		Psi = arma::exp(-this->h(k)*arma::pow(x-this->c(k),2));
		arma::rowvec temp = s.t()%Psi;
		this->w(k) = arma::dot(temp,Fd) / (arma::dot(temp,s) + this->zero_tol);
	}
}
	
void DMP_::train_RLWR(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd, double lambda)
{
	 n_data = Fd.n_elem;

	P = arma::vec().ones(this->N_kernels);
	this->w = arma::vec().zeros(this->N_kernels);

	for (i = 0; i < n_data; i++){
			
			arma::vec Psi = this->activation_function(x(i));
			
			arma::vec error = Fd(i) - dmp.w * s(i);
			
			P = (P - (P.^2 * s(i)^2)./ (lambda./ Psi + P * s(i) ^ 2)) / lambda;
			
			this->w = this->w + Psi % P * s(i) % error;
	}
}
	
void DMP_::train_LS(arma::rowvec &x, arma::rowvec &s, arma::rowvec &Fd)
{
	int n_data = Fd.n_elem;// namespace as64
	
	arma::mat H = arma::zeros(this->N_kernels, n_data);
	arma::vec Psi;
	for (int i=0;i<n_data;i++){
		Psi = arma::exp(-this->h % arma::pow(-this->c + x(i), 2));
		Psi = s(i)*Psi / (arma::sum(Psi) + this->zero_tol);
		H.col(i) = Psi;
	}

	this->w = (Fd*arma::pinv(H)).t();
}
  
void DMP_::set_a_z(double a)
{
  this->a_z = a;
}

void DMP_::set_b_z(double a)
{
  this->b_z = a;
}

void DMP_::set_tau(double t)
{
  can_sys_ptr->set_tau(t);
}

double DMP_::get_a_z() const
{
  return this->a_z;
}

double DMP_::get_b_z() const
{
  return this->b_z;
}
    
double DMP_::get_tau()
{
  return can_sys_ptr->get_tau();
}

double DMP_::get_v_scale()
{
  return get_tau() * a_s;
}*/

}  // namespace as64
