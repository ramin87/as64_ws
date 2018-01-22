#ifndef MATH_LIB_CONVERTIONS_H
#define MATH_LIB_CONVERTIONS_H

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <math_lib/math.h>

namespace as64_
{

void rosTransform_to_eigenTransform(const geometry_msgs::Transform &rosTrans, Eigen::Matrix4d &eigenTrans);

}

#endif // MATH_LIB_CONVERTIONS_H
