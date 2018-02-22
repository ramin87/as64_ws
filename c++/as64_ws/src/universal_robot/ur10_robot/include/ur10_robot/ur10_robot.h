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

#ifndef UR10_ROBOT_UR10_ROBOT_H
#define UR10_ROBOT_UR10_ROBOT_H

//#include <FastResearchInterface.h>
//#include <autharl_core/robot/robot.h>
//#include <autharl_core/robot/trajectories.h>
#include <ur10_robot/ur10_model.h>
#include <ros/ros.h>

#include <memory>
#include <vector>
#include <string>

#include <armadillo>

#define N_JOINTS 6


namespace ur10_
{

enum Mode {UNDEFINED = -1000, /**< For internal use */
           STOPPED = -1, /**< When the robot is stopped and does not accept commands */
           POSITION_CONTROL  = 0, /**< For sending position commands */
           VELOCITY_CONTROL  = 1, /**< For sending velocity commands */
           TORQUE_CONTROL    = 2, /**< For sending torque commands */
           IMPEDANCE_CONTROL = 3, /**< For operating in Impedance control */
           JOINT_TRAJECTORY  = 4 /**< Probably should be covered by position control */
          };

class Robot //: public ur_::Robot
{
public:

  Robot();
  explicit Robot(std::shared_ptr<ur10_::Model> m, const std::string& name = "UR10");
  void stop();
  void setMode(ur10_::Mode mode);
  ur10_::Mode getMode() const;
  void waitNextCycle();

  void setJointTrajectory(const KDL::JntArray &input, double duration);
  void setJointTrajectory(const arma::vec &input, double duration);
  void setJointTrajectory(const Eigen::VectorXd &input, double duration);
  template <typename T>
  void setJointTrajectoryTemplate(const T &input, double duration)
  {
    // // setJntPosTrajTemplate(input, duration, chain_index);
    // // inital joint position values
    // arma::vec q0 = arma::zeros<arma::vec>(7);
    // arma::vec temp = arma::zeros<arma::vec>(7);
    // for (int i = 0; i < 7; i++) {
    //   temp(i) = input(i);
    // }
    // getJointPosition(q0);
    // // keep last known robot mode
    // arl::robot::Mode prev_mode = mode;
    // arma::vec qref = q0;
    // // start conttroller
    // setMode(arl::robot::Mode::POSITION_CONTROL);
    // // initalize time
    // double t = 0.0;
    // // the main while
    // while (t < duration)
    // {
    //   // waits for the next tick also
    //   FRI->WaitForKRCTick();
    //   // compute time now
    //   t += cycle;
    //   // update trajectory
    //   qref = (arl::robot::trajectory::get5thOrder(t, q0, temp, duration)).col(0);
    //   // set joint positions
    //   setJointPosition(qref);
    // }
    // // reset last known robot mode
    // setMode(prev_mode);
  }

  void setJointPosition(const KDL::JntArray &input);
  void setJointPosition(const arma::vec &input);
  void setJointPosition(const Eigen::VectorXd &input);
  template <typename T> void setJointPositionTemplate(const T &input)
  {
    // if (this->mode == arl::robot::Mode::POSITION_CONTROL)
    // {
    //   if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    //   {
    //     printf("[JointPosController::setJointPosition] Joint positions are commanded with closed controller.\n");
    //     printf("Opening controller ...\n");
    //     // start te cotroller
    //     startJointPositionController();
    //     // wait one tick
    //     FRI->WaitForKRCTick();
    //   }
    //   // temp variables
    //   float temp[7];
    //   // put the values from arma to float[]
    //   for (int i = 0; i < 7; i++) {
    //     temp[i] = input(i);
    //   }
    //   // set commanded joint positions to qd
    //   FRI->SetCommandedJointPositions(temp);
    //   saveLastJointPosition(temp);
    // }
    // else
    // {
    //   std::cerr << "setJointPosition only available in POSITION_CONTROL mode" << std::endl;
    // }
  }

  void setJointVelocity(const KDL::JntArray &input);
  void setJointVelocity(const arma::vec &input);
  void setJointVelocity(const Eigen::VectorXd &input);
  template <typename T> void setJointVelocityTemplate(const T &input)
  {
    // if (this->mode == arl::robot::Mode::VELOCITY_CONTROL)
    // {
    //   static float temp[7];
    //   for (size_t i = 0; i < 7; i++)
    //   {
    //     last_jnt_pos[i] += input(i) * cycle;
    //   }
    //   FRI->SetCommandedJointPositions(last_jnt_pos);
    // }
    // else
    // {
    //   std::cerr << "setJointVelocity only available in VELOCITY_CONTROL mode" << std::endl;
    // }
  }

  void setJointTorque(const KDL::JntArray &input);
  void setJointTorque(const arma::vec &input);
  void setJointTorque(const Eigen::VectorXd &input);
  template <typename T> void setJointTorqueTemplate(const T &input)
  {
    // if (this->mode == arl::robot::Mode::TORQUE_CONTROL)
    // {
    //   static float torques[7];
    //   for (size_t i = 0; i < 7; i++)
    //   {
    //     torques[i] = input(i);
    //   }
    //   FRI->SetCommandedJointTorques(torques);
    //   // Mirror the joint positions and the cartesian pose in order to avoid
    //   // cartesian deviation errors
    //   static float temp_position[7];
    //   FRI->GetMeasuredJointPositions(temp_position);
    //   FRI->SetCommandedJointPositions(temp_position);
    //   static float temp_pose[12];
    //   FRI->GetMeasuredCartPose(temp_pose);
    //   FRI->SetCommandedCartPose(temp_pose);
    //
    //   saveLastJointPosition(temp_position);
    // }
    // else
    // {
    //   std::cerr << "setJointTorque only available in TORQUE_CONTROL mode" << std::endl;
    // }
  }

  void getJointPosition(KDL::JntArray &output);
  void getJointPosition(arma::vec &output);
  void getJointPosition(Eigen::VectorXd &output);
  template <typename T> void getJointPositionTemplate(T &output)
  {
    output.resize(model->getNrOfJoints());
    static float temp[7];
    // FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < N_JOINTS; i++) {
      output(i) = temp[i];
    }
  }

  void getJointVelocity(KDL::JntArray &output);
  void getJointVelocity(arma::vec &output);
  void getJointVelocity(Eigen::VectorXd &output);
  template <typename T> void getJointVelocityTemplate(T &output)
  {
    // output.resize(model->getNrOfJoints(chain_index));
    // static float temp[7];
    // FRI->GetMeasuredJointPositions(temp);
    // for (size_t i = 0; i < 7; i++) {
    //   output(i) = (temp[i] - last_jnt_pos[i]) / cycle;
    // }
  }

  void getJointTorque(KDL::JntArray &output);
  void getJointTorque(arma::vec &output);
  void getJointTorque(Eigen::VectorXd &output);
  template <typename T> void getJointTorqueTemplate(T &output)
  {
    // output.resize(model->getNrOfJoints(chain_index));
    // static float joint_torques[7];
    // FRI->GetMeasuredJointTorques(joint_torques);
    // output(0) = joint_torques[0];
    // output(1) = joint_torques[1];
    // output(2) = joint_torques[2];
    // output(3) = joint_torques[3];
    // output(4) = joint_torques[4];
    // output(5) = joint_torques[5];
    // output(6) = joint_torques[6];
  }

  void getJointExternalTorque(KDL::JntArray &output);
  void getJointExternalTorque(arma::vec &output);
  void getJointExternalTorque(Eigen::VectorXd &output);
  template <typename T> void getJointExternalTorqueTemplate(T &output)
  {
    // output.resize(model->getNrOfJoints(chain_index));
    // static float estimated_external_joint_torques[7];
    // FRI->GetEstimatedExternalJointTorques(estimated_external_joint_torques);
    // output(0) = estimated_external_joint_torques[0];
    // output(1) = estimated_external_joint_torques[1];
    // output(2) = estimated_external_joint_torques[2];
    // output(3) = estimated_external_joint_torques[3];
    // output(4) = estimated_external_joint_torques[4];
    // output(5) = estimated_external_joint_torques[5];
    // output(6) = estimated_external_joint_torques[6];
  }

  void getJacobian(KDL::Jacobian &output);
  void getJacobian(arma::mat &output);
  void getJacobian(Eigen::MatrixXd &output);
  template <typename T> void getJacobianTemplate(T &output)
  {
    // float **temp;
    // temp = reinterpret_cast<float**>(malloc(sizeof(float *) * 6));
    // for (size_t i = 0; i < 6; i++) {
    //   temp[i] = reinterpret_cast<float*>(malloc(sizeof(float) * 7));
    // }
    // FRI->GetCurrentJacobianMatrix(temp);
    // std::vector<int> jac_indexes{0, 1, 2, 5, 4, 3};
    // for (size_t i = 0; i < jac_indexes.size(); i++)
    // {
    //   for (size_t j = 0; j < 7; j++)
    //   {
    //     output(i, j) = temp[jac_indexes[i]][j];
    //   }
    // }
  }

  void getTaskPose(KDL::Frame &output);
  void getTaskPose(arma::mat &output);
  void getTaskPose(Eigen::MatrixXd &output);
  template <typename T> void getTaskPoseTemplate(T &output)
  {
    // output.resize(3, 4);
    // static float temp[12];
    // FRI->GetMeasuredCartPose(temp);
    // for (size_t i = 0; i < 3; i++)
    // {
    //   for (size_t j = 0; j < 4; j++)
    //   {
    //     output(i, j) = temp[i * 4 + j];
    //   }
    // }
  }

  void getTaskPosition(KDL::Vector &output);
  void getTaskPosition(arma::vec &output);
  void getTaskPosition(Eigen::Vector3d &output);
  template <typename T> void getTaskPositionTemplate(T &output)
  {
    // static float temp[12];
    // FRI->GetMeasuredCartPose(temp);
    // output(0) = temp[3];
    // output(1) = temp[7];
    // output(2) = temp[11];
  }

  void getTaskOrientation(KDL::Rotation &output);
  void getTaskOrientation(arma::mat &output);
  void getTaskOrientation(Eigen::Matrix3d &output);
  template <typename T> void getTaskOrientationTemplate(T &output)
  {
    // static float temp[12];
    // FRI->GetMeasuredCartPose(temp);
    // output(0, 0) = temp[0];
    // output(0, 1) = temp[1];
    // output(0, 2) = temp[2];
    // output(1, 0) = temp[4];
    // output(1, 1) = temp[5];
    // output(1, 2) = temp[6];
    // output(2, 0) = temp[8];
    // output(2, 1) = temp[9];
    // output(2, 2) = temp[10];
  }

  void getTwist(KDL::Twist &output);
  void getTwist(arma::vec &output);
  void getTwist(Eigen::VectorXd &output);
  template <typename T> void getTwistTemplate(T &output)
  {
    // KDL::Jacobian jac(7);
    // KDL::JntArray vel(7);
    // getJointVelocity(vel, chain_index);
    // getJacobian(jac, chain_index);
    // Eigen::VectorXd task_vel = jac.data * vel.data;
    // output(0) = task_vel(0);
    // output(1) = task_vel(1);
    // output(2) = task_vel(2);
    // output(3) = task_vel(3);
    // output(4) = task_vel(4);
    // output(5) = task_vel(5);
  }

  void getExternalWrench(KDL::Wrench &output);
  void getExternalWrench(arma::vec &output);
  void getExternalWrench(Eigen::VectorXd &output);
  template <typename T> void getExternalWrenchTemplate(T &output)
  {
    // static float estimated_external_cart_forces_and_torques[6];
    // FRI->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
    // output(0) = estimated_external_cart_forces_and_torques[0];
    // output(1) = estimated_external_cart_forces_and_torques[1];
    // output(2) = estimated_external_cart_forces_and_torques[2];
    // output(3) = estimated_external_cart_forces_and_torques[3];
    // output(4) = estimated_external_cart_forces_and_torques[4];
    // output(5) = estimated_external_cart_forces_and_torques[5];
  }

  void setWrench(const KDL::Wrench &input);
  void setWrench(const arma::vec &input);
  void setWrench(const Eigen::VectorXd &input);
  template <typename T> void setWrenchTemplate(T &input)
  {
    // if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    // {
    //   if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    //   {
    //     printf("[CartImpedanceController::setWrench] Cartesian wrench is commanded with closed controller.\n");
    //     printf("Opening controller ...\n");
    //     // start te controller
    //     startCartImpController();
    //     // wait one tick
    //     FRI->WaitForKRCTick();
    //   }
    //   // temp variables
    //   float temp[6];
    //   // put the values from arma to float[]
    //   for (int i = 0; i < 6; i++) {
    //     temp[i] = input(i);
    //   }
    //
    //   // set commanded Cartesian forces/torques
    //   FRI->SetCommandedCartForcesAndTorques(temp);
    //
    //   static float temp_position[7];
    //   FRI->GetMeasuredJointPositions(temp_position);
    //   // FRI->SetCommandedJointPositions(temp_position);
    //   // static float temp_pose[12];
    //   // FRI->GetMeasuredCartPose(temp_pose);
    //   // FRI->SetCommandedCartPose(temp_pose);
    //   saveLastJointPosition(temp_position); //needed for numeric differentation to obtain q_dot [isn't it?]
    //
    // }
    // else
    // {
    //   std::cerr << "setWrench only available in IMPEDANCE_CONTROL mode" << std::endl;
    // }
  }

  void setTaskPose(const KDL::Frame &input);
  void setTaskPose(const arma::mat &input);
  void setTaskPose(const Eigen::MatrixXd &input);
  template <typename T> void setTaskPoseTemplate(T &input)
  {
    // if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    // {
    //   if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    //   {
    //     printf("[CartImpedanceController::setTaskPose] Cartesian wrench is commanded with closed controller.\n");
    //     printf("Opening controller ...\n");
    //     // start te controller
    //     startCartImpController();
    //     // wait one tick
    //     FRI->WaitForKRCTick();
    //   }
    //   // temp variables
    //   float temp[12];
    //   for (size_t i = 0; i < 3; i++)
    //     {
    //       for (size_t j = 0; j < 4; j++)
    //       {
    //         temp[i * 4 + j] = input(i, j);
    //       }
    //   }
    //
    //   // set commanded task pose
    //   FRI->SetCommandedCartPose(temp);
    // }
    // else
    // {
    //   std::cerr << "setTaskPose only available in IMPEDANCE_CONTROL mode" << std::endl;
    // }
  }

  void setCartStiffness(const KDL::Wrench &input);
  void setCartStiffness(const arma::vec &input);
  void setCartStiffness(const Eigen::VectorXd &input);
  template <typename T> void setCartStiffnessTemplate(T &input)
  {
    // if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    // {
    //   if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    //   {
    //     printf("[CartImpedanceController::setCartStiffness] Cartesian wrench is commanded with closed controller.\n");
    //     printf("Opening controller ...\n");
    //     // start te controller
    //     startCartImpController();
    //     // wait one tick
    //     FRI->WaitForKRCTick();
    //   }
    //   // temp variables
    //   float temp[6];
    //   // put the values from arma to float[]
    //   for (int i = 0; i < 6; i++) {
    //     temp[i] = input(i);
    //   }
    //
    //   // set value
    //   FRI->SetCommandedCartStiffness(temp);
    // }
    // else
    // {
    //   std::cerr << "setCartStiffness only available in IMPEDANCE_CONTROL mode" << std::endl;
    // }
  }

  void setCartDamping(const KDL::Wrench &input);
  void setCartDamping(const arma::vec &input);
  void setCartDamping(const Eigen::VectorXd &input);
  template <typename T> void setCartDampingTemplate(T &input)
  {
    // if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    // {
    //   if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    //   {
    //     printf("[CartImpedanceController::setCartDamping] Cartesian wrench is commanded with closed controller.\n");
    //     printf("Opening controller ...\n");
    //     // start te controller
    //     startCartImpController();
    //     // wait one tick
    //     FRI->WaitForKRCTick();
    //   }
    //   // temp variables
    //   float temp[6];
    //   // put the values from arma to float[]
    //   for (int i = 0; i < 6; i++) {
    //     temp[i] = input(i);
    //   }
    //
    //   // set value
    //   FRI->SetCommandedCartDamping(temp);
    // }
    // else
    // {
    //   std::cerr << "setCartDamping only available in IMPEDANCE_CONTROL mode" << std::endl;
    // }
  }

  bool isOk();

private:
  Mode mode;
  std::shared_ptr<Model> model;
  // std::shared_ptr<FastResearchInterface> FRI;
  void startJointPositionController();
  void startJointTorqueController();
  void startCartImpController();
  void stopController();
  void startLogging();
  void stopLogging();
  void saveLastJointPosition(float input[7]);
  void saveLastJointPosition();
  // float last_jnt_pos[7];
};

}  // namespace ur10_

#endif  // UR10_ROBOT_UR10_ROBOT_H
