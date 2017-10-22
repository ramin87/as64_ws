#ifndef AS64_MATH_CONVERTIONS_H
#define AS64_MATH_CONVERTIONS_H

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <as64_utils/math/math.h>
#include <Eigen/Dense>

namespace as64
{
	
void rosTransform_to_eigenTransform(const geometry_msgs::Transform &rosTrans, Eigen::Matrix4d &eigenTrans);

}

#endif
