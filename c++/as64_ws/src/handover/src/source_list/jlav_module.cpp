#include "jlav_module.h"

/**
* @brief Jlav_module::Jlav_module
*/
Jlav_module::Jlav_module()
{

}




/**
* @brief Jlav_module::Jlav_module
* @param qmin, low limit of joints in a vector form
* @param qmax, high limit of joints in a vector form
* @param gain, gain for the attraction to the mean value
*/
Jlav_module::Jlav_module(arma::vec qmin, arma::vec qmax)
{

    // Create ROS node handle
    nh_ = ros::NodeHandle("~");

    // read parameters
    //readParams();

    // initialization
    init(qmin,qmax);

}

/**
* @brief Jlav_module::readParams, read the parameters from the yaml file
*/
void Jlav_module::readParams()
{
    std::cout << "[Jlav_controller::readParams] Reading the parameters from the yaml file." << std::endl;
    nh_.getParam("k_jla", gain_) ;
    //SHOW_DOUB(gain_);
}



/**
* @brief Jlav_module::init
* @param qmin, low limit of joints in a vector form
* @param qmax, high limit of joints in a vector form
*/
void Jlav_module::init(arma::vec qmin, arma::vec qmax)
{
  //SHOW_ARMA(qmin);
  //SHOW_ARMA(qmax);
    // check if vectors have the same size
    if (qmax.n_rows != qmin.n_rows) {
        std::cout << "[ERROR - Joint Limit Avoidance Controller] Joint limit vectors do not have the same size !\n" ;
        Njoints_ = -1;
        qmean_(0) = -1;
    } else {

        // Printouts
        std::cout << "[Jlav_controller::init] Number of Joint Constraints: " << qmax.n_rows << std::endl;

        // set number of joints
        Njoints_ = qmax.n_rows;

        // initialize joint mean;
        qmean_ = (qmin + qmax) / 2;

        // initialize rho
        rho_ = (qmax - qmin) / 2;
    }

    // set the limits
    qmin_ = qmin;
    qmax_ = qmax;

    // resize gain vector
    kq_ = arma::zeros<arma::vec>(Njoints_);

    for (int i = 0; i < Njoints_; i++) {
        kq_(i) = gain_;
    }
    // initialize variables
    initAllVarsToZero();


}


/**
* @brief Jlav_module::initAllVarsToZero
*/
void Jlav_module::initAllVarsToZero()
{
    e_ = arma::zeros<arma::vec>(Njoints_);
    c_ = arma::zeros<arma::vec>(Njoints_);
    epsilon_ = arma::zeros<arma::vec>(Njoints_);
    dT_ = arma::zeros<arma::vec>(Njoints_);
    sig_ = arma::zeros<arma::vec>(Njoints_);
}


/**
* @brief Jlav_module::changeLowLimit, change one joint limit value
* @param joint_index, index of the joint
* @param low_limit, the low limit value
*/
void Jlav_module::changeLowLimit(int joint_index, double low_limit)
{
    // set the value
    qmin_(joint_index) = low_limit;
}

/**
* @brief Jlav_module::changeLowLimits, change all the joint limit values
* @param qmins, joint low limits
*/
void Jlav_module::changeLowLimits(arma::vec qmins)
{
    // set the vector values
    qmin_ = qmins;
}

/**
* @brief Jlav_module::changeHighLimit, change one joint limit value
* @param joint_index, index of the joint
* @param low_limit, the high limit value
*/
void Jlav_module::changeHighLimit(int joint_index, double high_limit)
{
    // set the value
    qmax_(joint_index) = high_limit;
}


/**
* @brief Jlav_module::changeHighLimits, change all the joint limit values
* @param qmaxs, joint high limits
*/
void Jlav_module::changeHighLimits(arma::vec qmaxs)
{
    // set the vector values
    qmax_ = qmaxs;
}

/**
* @brief Jlav_module::changeGain, change one joint gain value
* @param joint_index, index of the joint
* @param gain, the gain value
*/
void Jlav_module::changeGain(int joint_index, double gain)
{
    // set the value
    kq_(joint_index) = gain;
}

/**
* @brief Jlav_module::changeGains, change all the joint gain values with the same value
* @param gain, the gain value
*/
void Jlav_module::changeGains(double gain)
{

    gain_ = gain;

// set all vector values to the same value
    for (int i = 0; i < Njoints_; i++) {
        kq_(i) = gain;
    }

}

/**
* @brief Jlav_module::changeGains, change all the joint gain values
* @param gains, the gain values in vector form
*/
void Jlav_module::changeGains(arma::vec gains)
{
    // set the vector values
    kq_ = gains;
}


/**
* @brief Jlav_module::printParams
*/
void Jlav_module::printParams(void)
{

    std::cout << "[Jlav_controller::printParams] Printing all Joint limit acoidance controller parameters ..." << std::endl;

    //SHOW_DOUB(Njoints_);
    //SHOW_ARMA_U(qmin_, "rad");
    //SHOW_ARMA_U(qmax_, "rad");
    //SHOW_ARMA_U(qmean_, "rad");
    //SHOW_ARMA_U(rho_, "rad");
    //SHOW_ARMA(kq_);

}




/**
* @brief Jlav_module::getControlSignal
* @return the control signal, which is a velocity vector of dimension of N on the joint space
* @param q_meas, measure joint position
*/
arma::vec Jlav_module::getControlSignal(arma::vec q_meas)
{

    // compute the error from the mean
    e_ = q_meas - qmean_;

    for (int i = 0; i < Njoints_; i++) {

        // compute the intermediate parameter c of controller
        c_(i) = e_(i) / rho_(i);

        // compute espilon of PPC methodology
        epsilon_(i) = std::log((1 + c_(i)) / (1 - c_(i)));

        // compute jacobian of PPC method
        dT_(i) 	=  1 / (e_(i) + rho_(i)) - 1 / (e_(i) - rho_(i));

        // signal for each joint
        sig_(i) = - kq_(i) * dT_(i) * epsilon_(i) ;

    }

    return sig_;

}

