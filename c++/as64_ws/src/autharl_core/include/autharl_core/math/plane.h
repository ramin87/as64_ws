/*******************************************************************************
 * Copyright (c) 2016-2017 Automation and Robotics Lab, AUTh
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#ifndef AUTHARL_CORE_MATH_PLANE_H
#define AUTHARL_CORE_MATH_PLANE_H

#include <kdl/frames.hpp>
#include <vector>

namespace arl
{
namespace math
{
/**
 * @brief Checks if a set of 3D points are cooplannar.
 *
 * Epsilon = 0 means zero tolerance to errors. Epsilon = 1 means that an
 * arbitrary number of points will be considered as coplanar.
 *
 * @param point The set of points to be tested
 * @param epsilon A param which determines the error tolerance
 * @return True if they are coplaner. False otherwise
 */
bool areCoplanar(const std::vector<KDL::Vector>& point, double epsilon = 1e-6);

/**
 * @brief Sorts a set of unsorted points in circular ordering.
 *
 * This method takes the centroid of the points, calculates the vector of each
 * point to the centroid and sort them by using the minimum angle. This method
 * can not guarantee to work with large point clouds with equal minimum angles
 * occur.
 *
 * @param input The set of points to be sorted
 * @param output The resulted sorted set of points
 */
void sortCircularOrder(const std::vector<KDL::Vector>& input, std::vector<KDL::Vector>* output);

}  // namespace math
}  // namespace arl
#endif  // AUTHARL_CORE_MATH_PLANE_H
