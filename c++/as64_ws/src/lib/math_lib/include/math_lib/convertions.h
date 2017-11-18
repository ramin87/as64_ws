#ifndef MATH_LIB__CONVERTIONS_H
#define MATH_LIB__CONVERTIONS_H

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <math_lib/math.h>
#include <Eigen/Dense>

namespace as64
{

void rosTransform_to_eigenTransform(const geometry_msgs::Transform &rosTrans, Eigen::Matrix4d &eigenTrans);

}

#endif // MATH_LIB__CONVERTIONS_H
