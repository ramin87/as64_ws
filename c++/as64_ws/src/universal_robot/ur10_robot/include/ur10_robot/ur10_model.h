/*******************************************************************************
 * Copyright (C) 2018 as64
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

#ifndef UR10_ROBOT_UR10_MODEL_H
#define UR10_ROBOT_UR10_MODEL_H

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <limits>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace ur10_
{

class Model
{
public:
  /**
   * @brief An empty constructor.
   */
  Model();

  /**
   * @brief A basic constructor parsing the name of the model.
   */
  explicit Model(const std::string& n);

  void load();
  void load(const std::string& path);


  /**
   * @brief Returns the name of the robot model.
   */
  std::string getModelName();


  /**
   * @brief Returns the number of joints.
   */
  unsigned int getNrOfJoints();


  /**
   * @brief Returns the names of the joints.
   */
  std::vector<std::string> getJointNames();

  /**
   * @brief Returns the name of a joint.
   *
   * @param[in] joint The desired joint
   * @return The name of the joint
   */
  std::string getJointName(unsigned int joint);

  /**
   * @brief Returns the limit of a joint.
   *
   * @param[in] joint The desired joint.
   * @return The limit of the joint.
   */
  std::pair<double, double> getJointLimit(unsigned int joint);

  /**
   * @brief Returns the limits of model's joints.
   *
   * @return The limits of the joints.
   */
  std::vector<std::pair<double, double> > getJointLimits();


  /**
   * @brief KDL forward kinematic solver.
   */
  //KDL::ChainFkSolverPos_recursive fk_solver;

  /**
   * @brief KDL Jacobian solver. Calculates the base Jacobian matrix of the robot.
   */
  //KDL::ChainJntToJacSolver jac_solver;

  /**
   * @brief KDL Chain storing the kinematics of the robot.
   */
  KDL::Chain chain;

protected:

  /**
   * @brief The name of this Model
   */
  std::string name;

  /**
   * @brief The joint names of this Model
   */
  std::vector<std::string> joint_name;

  /**
   * @brief The joint limits of this Model
   */
  std::vector<std::pair<double, double> > joint_limit;

  /**
   * @brief The link names of this Model
   */
  std::vector<std::string> link_name;
};

}  // namespace ur10_

#endif  // UR10_ROBOT_UR10_MODEL_H
