#ifndef MATH_LIB_MATH__64_H
#define MATH_LIB_MATH__64_H

#include <Eigen/Dense>
#include <armadillo>

namespace as64
{

Eigen::Matrix3d vec2ssMat(const Eigen::Vector3d &v);
arma::mat vec2ssMat(const arma::vec &v);

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);
arma::vec rotm2quat(const arma::mat &rotm);

Eigen::Matrix3d quat2rotm(Eigen::Vector4d quat);
arma::mat quat2rotm(const arma::vec &quat);

Eigen::Vector4d rotm2axang(Eigen::Matrix3d rotm);
arma::vec rotm2axang(const arma::mat &rotm);

Eigen::Matrix3d axang2rotm(Eigen::Vector4d axang);
arma::mat axang2rotm(const arma::vec &axang);

Eigen::Vector4d axang2quat(Eigen::Vector4d axang);
arma::vec axang2quat(const arma::vec &axang);

Eigen::Vector4d quat2axang(Eigen::Vector4d quat);
arma::vec quat2axang(const arma::vec &quat);

Eigen::MatrixXd inv(const Eigen::MatrixXd &M);
arma::mat inv(const arma::mat &M);

}

#endif // MATH_LIB_MATH__64_H
