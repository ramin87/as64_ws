#ifndef SVF_MODULE_H
#define SVF_MODULE_H

#include <cmath>
#include <Eigen/Dense>

class SVF_module
{
public:
    SVF_module(double sigma_min = 1e-3, double shape_f = 5);
  
    void set_sigma_min(double sigma_min);
    void set_shape_factor(double shape_f);
    
    double get_sigma_min() const;
    double get_shape_factor() const;
    
    double filter_eig_val(double sigma) const;
    
    Eigen::MatrixXd inv(Eigen::MatrixXd M) const;
private:
    double sigma0; // minimum allowable eigen value
    double v; // shape factor
};

#endif // SVF_MODULE_H
