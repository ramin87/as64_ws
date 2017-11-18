#include <math_lib/math.h>
#include <boost/concept_check.hpp>

namespace as64
{

Eigen::Matrix3d vec2ssMat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d ssMat;
  ssMat(0, 1) = -v(2);
  ssMat(0, 2) =  v(1);
  ssMat(1, 0) =  v(2);
  ssMat(1, 2) = -v(0);
  ssMat(2, 0) = -v(1);
  ssMat(2, 1) =  v(0);

  return ssMat;
}

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm)
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

Eigen::Matrix3d quat2rotm(Eigen::Vector4d quat)
{
  double qw=quat(0), qx=quat(1), qy=quat(2), qz=quat(3);

  Eigen::Matrix3d rotm;
  rotm << 1 - 2*qy*qy - 2*qz*qz, 	2*qx*qy - 2*qz*qw, 	2*qx*qz + 2*qy*qw,
	    2*qx*qy + 2*qz*qw, 	      1 - 2*qx*qx - 2*qz*qz, 	2*qy*qz - 2*qx*qw,
	    2*qx*qz - 2*qy*qw, 	        2*qy*qz + 2*qx*qw, 	1 - 2*qx*qx - 2*qy*qy;

  return rotm;
}

Eigen::Vector4d rotm2axang(Eigen::Matrix3d rotm)
{
  Eigen::AngleAxis<double> angleAxis(rotm);

  Eigen::Vector4d axang;
  axang(3) = angleAxis.angle();
  axang.segment(0,3) = angleAxis.axis();

  return axang;
}

Eigen::Matrix3d axang2rotm(Eigen::Vector4d axang)
{
  Eigen::Matrix3d rotm;
  Eigen::Vector3d axis = axang.segment(0,3);
  double angle = axang(3);

  double x=axis(0), y=axis(1), z=axis(2), c=std::cos(angle), s=std::sin(angle), t=1-c;
  rotm <<   t*x*x + c,	    t*x*y - z*s,     t*x*z + y*s,
  	    t*x*y + z*s,    t*y*y + c,	     t*y*z - x*s,
            t*x*z - y*s,    t*y*z + x*s,     t*z*z + c;

  return rotm;
}

Eigen::Vector4d axang2quat(Eigen::Vector4d axang)
{
  Eigen::Vector4d quat;
  double theta = axang(3);

  quat(0) = std::cos(theta/2);
  quat.segment(1,3) = std::sin(theta/2) * axang.segment(0,3);

  return quat;
}

Eigen::Vector4d quat2axang(Eigen::Vector4d quat)
{
  Eigen::Vector4d axang;
  Eigen::Vector3d r = quat.segment(1,3);

  if (r.norm()){
    axang(3) = 2 * std::acos(quat(0));
    axang.segment(0,3) = r/r.norm();
  }else axang << 0, 0, 1, 0;

  return axang;
}



Eigen::MatrixXd inv(Eigen::MatrixXd M)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd S = svd.singularValues().asDiagonal();
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();

  // Eigen::MatrixXd M_reconstr = U*S*V.transpose();

  return V*S*U.transpose();
}

}
