#ifndef MATH_LIB_UNIT_QUATERNIONS_64_H
#define MATH_LIB_UNIT_QUATERNIONS_64_H

#include <Eigen/Dense>
#include <math_lib/math.h>

namespace as64
{

Eigen::Vector4d quatInv(const Eigen::Vector4d &quat);

Eigen::Matrix4d quat2mat(const Eigen::Vector4d &quat);

Eigen::Vector4d quatProd(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);

// If quat1 and quat2 were positions, this would perform quat1-quat2.
// The result is the amount of rotation needed to go from quat2 to quat1, i.e. quatD*quat2 = quat1
// Equivalently, the result is quaternion corresponding to the angular velocity which takes us from quat2 to quat1 in unit time.
Eigen::Vector4d quatDiff(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);

Eigen::Vector4d quatExp(const Eigen::Vector3d &omega, double tol=1e-15);

Eigen::Vector3d quatLog(const Eigen::Vector4d &quat);

Eigen::Vector4d get_quat_dot(const Eigen::Vector3d &omega, Eigen::Vector4d &quat);

}

#endif // MATH_LIB_UNIT_QUATERNIONS_64_H
