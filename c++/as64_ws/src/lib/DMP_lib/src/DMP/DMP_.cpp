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

        this->a_s = 1/10;

        this->w = arma::vec().zeros(this->N_kernels); // rand(this->N_kernels,1);
        this->set_centers();
        this->set_stds(std_K);

        set_training_params(false, 0, 0.99, 1e6);
	}

	void DMP_::set_centers(const std::string  &part_type)
	{
		arma::rowvec t = arma::linspace<arma::rowvec>(0,this->N_kernels-1,this->N_kernels)/(this->N_kernels-1);

        if (part_type.compare("lin")==0)
        {
            this->c = t.t();
        }
        else if (part_type.compare("")==0)
        {
            arma::mat x = this->can_sys_ptr->get_continuous_output(t*this->can_sys_ptr->get_tau());
            this->c = x.row(0).t();
        }
        else
        {
            throw std::invalid_argument(std::string("Unsupported partition type ") + part_type);
        }
		
	}

	void DMP_::set_stds(double s)
	{
		int n = this->c.n_elem;
		this->h.resize(n);
		
		for (int i=0;i<n-1;i++)
        {
			this->h(i) = 1/std::pow(s*(this->c(i+1) - this->c(i)), 2);
		}
				
		this->h(n-1) = this->h(n-2);
	}

	double DMP_::train(const arma:: rowvec &Time, const arma::rowvec &yd_data, const arma::rowvec &dyd_data,
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
	
}  // namespace as64
