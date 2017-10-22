#ifndef MATH_64_H
#define MATH_64_H

#include <Eigen/Dense>

namespace as64
{
  
Eigen::Matrix3d vec2ssMat(const Eigen::Vector3d &v);

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);

Eigen::Matrix3d quat2rotm(Eigen::Vector4d quat);

Eigen::Vector4d rotm2axang(Eigen::Matrix3d rotm);

Eigen::Matrix3d axang2rotm(Eigen::Vector4d axang);

Eigen::Vector4d axang2quat(Eigen::Vector4d axang);

Eigen::Vector4d quat2axang(Eigen::Vector4d quat);

Eigen::MatrixXd inv(Eigen::MatrixXd M);

}

#endif
