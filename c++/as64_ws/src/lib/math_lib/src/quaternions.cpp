#include <math_lib/quaternions.h>

namespace as64
{

Eigen::Vector4d quatInv(const Eigen::Vector4d &quat)
{
  Eigen::Vector4d quatI;

  quatI(0) = quat(0);
  quatI.segment(1,3) = - quat.segment(1,3);

  return quatI;
}


Eigen::Matrix4d quat2mat(const Eigen::Vector4d &quat)
{
  Eigen::Matrix4d mat;
  Eigen::Vector3d e = quat.segment(1,3);
  double n = quat(0);

  mat << n, -e.transpose(), e, n*Eigen::Matrix3d::Identity() + vec2ssMat(e);

  return mat;
}

Eigen::Vector4d quatProd(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2)
{
  Eigen::Vector4d quat12;

  quat12 = quat2mat(quat1) * quat2;

  return quat12;
}

// If quat1 and quat2 were positions, this would perform quat1-quat2.
// The result is the amount of rotation needed to go from quat2 to quat1, i.e. quatD*quat2 = quat1
// Equivalently, the result is quaternion corresponding to the angular velocity which takes us from quat2 to quat1 in unit time.
Eigen::Vector4d quatDiff(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2)
{
  Eigen::Vector4d quatD;

  quatD = quatProd(quat1, quatInv(quat2));

  return quatD;
}

arma::vec quatDiff(const arma::vec &quat1, const arma::vec &quat2)
{
  arma::vec qdiff(4);

  Eigen::Map<const Eigen::Vector4d> temp_quat1(quat1.memptr());
  Eigen::Map<const Eigen::Vector4d> temp_quat2(quat2.memptr());
  Eigen::Map<Eigen::Vector4d> temp_qdiff(qdiff.memptr());
  temp_qdiff = quatDiff(temp_quat1, temp_quat2);

  return qdiff;

}


Eigen::Vector4d quatExp(const Eigen::Vector3d &omega, double tol)
{
  // omega = k*theta, where k is the normalized angle of rotation
  // norm(omega) = |theta|
  // omega/norm(omega) = k * sign(sin(theta))

  Eigen::Vector4d quat;
  double norm_omega = omega.norm();
  double theta = norm_omega;

 // if (norm_omega){
    quat(0) = std::cos(theta/2);
    quat.segment(1,3) = std::sin(theta/2)*omega/(norm_omega+tol);
  //}else quat << 1, 0, 0, 0;

  quat *= quat(0);

  return quat;
}

Eigen::Vector3d quatLog(const Eigen::Vector4d &quat)
{
  Eigen::Vector3d e = quat.segment(1,3);
  double n = quat(0);

  Eigen::Vector3d omega;

  if (e.norm()) omega = 2*std::acos(n)*e/e.norm();
  else omega = Eigen::Vector3d::Zero();

  return omega;
}


Eigen::Vector4d get_quat_dot(const Eigen::Vector3d &omega, Eigen::Vector4d &quat)
{
  Eigen::Vector4d quat_dot;
  Eigen::Matrix<double,4,3> J_Q = quat2mat(quat).block(0,1,4,3);

  quat_dot = 0.5 * J_Q * omega;

  return quat_dot;
}

}
