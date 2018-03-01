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

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <ros/ros.h>
#include <ros/console.h>

#include <ur10_robot/ur10_model.h>

namespace ur10_
{

Model::Model()
{
  load();
}

Model::Model(const std::string& path)
{
  load(path);
}

void Model::load()
{
  name = "UR10";
  joint_name.push_back("ur10_shoulder_pan_joint");
  joint_name.push_back("ur10_shoulder_lift_joint");
  joint_name.push_back("ur10_elbow_joint");
  joint_name.push_back("ur10_wrist_1_joint");
  joint_name.push_back("ur10_wrist_2_joint");
  joint_name.push_back("ur10_wrist_3_joint");

  link_name.push_back("ur10_base_link");
  link_name.push_back("ur10_shoulder_link");
  link_name.push_back("ur10_upper_arm_link");
  link_name.push_back("ur10_forearm_link");
  link_name.push_back("ur10_wrist_1_link");
  link_name.push_back("ur10_wrist_2_link");
  link_name.push_back("ur10_wrist_3_link");
  link_name.push_back("ur10_ee_link");
}

void Model::load(const std::string& path)
{
  load();

  /*
  urdf::Model urdf_model;
  if (!urdf_model.initParam(path))
  {
    ROS_INFO_STREAM("Error getting robot description");
  }
  else
  {
    ROS_INFO_STREAM("Read urdf model");
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf_model, tree);
    for (size_t i = 0; i < getNrOfMainChains(); i++)
    {
      KDL::Chain new_chain;
      if (!tree.getChain(link_name.at(i).at(0), link_name.at(i).at(link_name.at(i).size() - 1), new_chain))
      {
        ROS_INFO_STREAM("Creating chain from " << link_name.at(i).at(0) << " to "
                                               << link_name.at(i).at(link_name.at(i).size() - 1) << " failed");
      }
      else
      {
        ROS_INFO_STREAM("Created chain from " << link_name.at(i).at(0) << " to "
                                              << link_name.at(i).at(link_name.at(i).size() - 1));
        fk_solver.push_back(KDL::ChainFkSolverPos_recursive(new_chain));
        jac_solver.push_back(KDL::ChainJntToJacSolver(new_chain));
        chain.push_back(new_chain);
      }
      std::vector<std::pair<double, double>> pairs;
      for (size_t j = 0; j < joint_name.at(i).size(); j++)
      {
        std::pair<double, double> limit;
        limit.first = urdf_model.getJoint(joint_name.at(i).at(j))->limits->lower;
        limit.second = urdf_model.getJoint(joint_name.at(i).at(j))->limits->upper;
        pairs.push_back(limit);
      }
      joint_limit.push_back(pairs);
    }
  }
  */
}

std::string Model::getModelName()
{
  return name;
}

unsigned int Model::getNrOfJoints()
{
  return joint_name.size();
}

std::string Model::getJointName(unsigned int joint)
{
  return joint_name.at(joint);
}

std::vector<std::string> Model::getJointNames()
{
  return joint_name;
}


std::pair<double, double> Model::getJointLimit(unsigned int joint)
{
  return joint_limit.at(joint);
}

std::vector<std::pair<double, double>> Model::getJointLimits()
{
  return joint_limit;
}


}  // namespace ur10_
