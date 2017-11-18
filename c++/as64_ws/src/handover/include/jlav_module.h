#ifndef JLAV_MODULE_H
#define JLAV_MODULE_H


#include <armadillo>
#include <iostream>
// #include <show.h>
#include <ros/ros.h>



class Jlav_module
{
public:
    Jlav_module();
    Jlav_module( arma::vec qmin, arma::vec qmax);
    void init(arma::vec qmin, arma::vec qmax);

    void changeLowLimit( int joint_index, double low_limit);
    void changeLowLimits( arma::vec qmins);

    void changeHighLimit( int joint_index, double high_limit);
    void changeHighLimits( arma::vec qmaxs);

    void changeGain( int joint_index, double gain);
    void changeGains(double gain);
    void changeGains( arma::vec gains);

    void printParams(void);
    arma::vec getControlSignal(arma::vec q_meas);

private:
  
    void initAllVarsToZero();
    void readParams();

    // Initialization for YuMi
    int Njoints_ = 7;

    // parameters
    arma::vec qmin_;
    arma::vec qmax_;
    arma::vec qmean_;
    arma::vec rho_;
    arma::vec kq_;
    double gain_;
    
    // variables
    arma::vec e_, c_, epsilon_, dT_;
    arma::vec sig_;
    
    // node handler just to read the parameters
    ros::NodeHandle nh_;

};

#endif // JLAV_MODULE_H
