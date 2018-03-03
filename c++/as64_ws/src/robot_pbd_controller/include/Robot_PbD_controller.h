/**
 * Copyright (C) 2018 as64_
 */

#ifndef ROBOT_PbD_CONTROLLER_H
#define ROBOT_PbD_CONTROLLER_H

#define _USE_MATH_DEFINES

#define CART_DOF_SIZE 6
#define JOINT_SIZE 6

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <random>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <mutex>

#include <armadillo>
#include <Eigen/Dense>

#include <log_data.h>

#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>
#include <math_lib/math_lib.h>

using namespace as64_;

class Robot_PbD_controller
{
  struct TrainData
  {
    arma::rowvec Time;
    arma::mat q_data;
    arma::mat dq_data;
    arma::mat Y_data, dY_data, ddY_data;
    arma::mat Q_data, v_rot_data, dv_rot_data;
    arma::mat wrench_data;
    int getNumData() const { return Time.size(); }
    bool isValid() const { return getNumData()!=0; }
  };

  struct ConfigArgs
  {
    arma::vec F_dead_zone;
    std::string data_output_path;
    double pos_tol_stop;
    double orient_tol_stop;
  };

public:
  // Constructor that initializes variables that must be set once at the start of the program.
  Robot_PbD_controller();

  ~Robot_PbD_controller();

  virtual void parseConfigFile(const char *config_file=NULL);

  // Initializes the control flags and some other variables that must be reset at each
  // repetition of the task in the same program run.
  void initController();

  // Initializes the values of the program's control flags
  void initControlFlags();

  // Initializes the program's variables.
  void initProgramVariables();

  void robotWait();

  // Executes the main control loop.
  void execute();

  // Reads the current position, orientation, joints and calculates velocities and accelerations.
  void update();

  // Implements the Admittance controller and send the position commands to the robot.
  void command();

  // Saves the logged data to an output file, waits for all threads to join and returns.
  void finalize();

  // Records a demo that starts and ends by the user control signals from the keyboard.
  // Blocking.
  // Robot moves in admittance.
  void recordDemo();

  // Moves the robot to its starting position as recorded by the last call to "recordDemo".
  // Blocking.
  // Uses position control (no admittance here).
  void gotoStartPose();

  // Trains the model using the data in the struct 'trainData'.
  virtual void trainModel() = 0;

  // Gets the derivatives of the model states and numerically integrates them.
  virtual void executeModel() = 0;

  // Saves in the struct 'trainData' the current robot pose, velocities and accelerations.
  void logDemoStep();

  // Logs the model and robot's poses, velocities and accelerations.
  void logOnline();

  // Logs some data calculated after the execution of the model such as the forcing term,
  //  goal/shape attractor values, kernel functions activations, model weights, centers, stds etc.
  void logOffline();

  // Clears all data in struct 'trainData'
  void clearTrainData();

  // Clears all data in struct 'log_data'
  void clearLoggedData();

  // Clears all data in struct 'log_data' and restarts the whole demo and execution process.
  void clearAndRestartDemo();

  // Saves all data from struct 'log_data' in an output file, clears them and restarts the whole demo and execution process.
  void saveAndRestartDemo();

  // Saves all data from struct 'log_data' in an output file.
  void saveLoggedData();

  void saveDemoData();
  void loadDemoData();

  void keyboardCtrlThreadFun();
  void saveExecutionResults();

private:

  ConfigArgs cmd_args;

  int N_JOINTS;
  int DOFs;

  virtual void getJointPosition(arma::vec &q) = 0;
  virtual void getJointVelocity(arma::vec &dq) = 0;
  virtual void getTaskPose(arma::mat &pose) = 0;
  virtual void getJacobian(arma::mat &J) = 0;
  virtual void getExternalWrench(arma::vec &wrench) = 0;

  // Program control flags
  bool train_from_file;
  bool save_exec_results;
  bool log_on; // if true, at each step of the execution, intermediate data are logged.
  bool run_model; // if true the model is executed
  bool train_model; // if true the model is (re)trained
  bool goto_start; // if true the robot goes to its starting pose
  bool stop_robot; // if true the program terminates
  bool pause_robot;
  bool start_demo; // if true the program starts logging data in the "trainData" struct
  bool end_demo; // if true the program stops logging data in the "trainData" struct
  bool save_restart_demo; // if true saves all loged data and restarts the demo process
  bool clear_restart_demo; // if true clears all loged data (without saving them) and restarts the demo process

  int demo_save_counter; // add a number to the data output file to avoid overriding the previous one

  std::shared_ptr<std::thread> keyboard_ctrl_thread; // thread for the "keyboardCtrlThreadFun()"
  std::shared_ptr<std::thread> save_exec_results_thread; // thread for the "keyboardCtrlThreadFun()"
  std::shared_ptr<std::thread> train_model_thread; // thread for the "keyboardCtrlThreadFun()"

  arma::wall_clock timer; // timer to measure elapsed time

  LogData log_data; // struct used for logging data on/off-line
  TrainData trainData; // Data used for (re)training the model
  TrainData execData; // Data from last model execution

  double Ts; // robot control cycle, used for numerical integration
  double tau; // the total duration of the demonstrated movement
  int Dp; // dimensionality of the Cartesian Position model
  int Do; // dimensionality of the orientation model
  int D; // Dp + Do


  arma::vec offline_train_p_mse; // MSE of the Forcing term during the CartPos model training
  arma::vec offline_train_o_mse; // MSE of the Forcing term during the orient model training

  double t; // holds the current time value during demo or execution

  // Y stands for column vector with Cartesian Position (x,y,z)
  arma::vec Yg0, Y0, Y, dY, ddY;
  arma::vec Y_robot, dY_robot, dY_robot_prev, ddY_robot;
  arma::vec V_robot;
  arma::vec Fpos; // Cartesian Forces exerted to the robot by the environment (human, objects etc.)

  // Q expresses a quaternion
  arma::vec Qg0, Q0, Q, dQ, v_rot, dv_rot;
  arma::vec Q_robot, v_rot_robot, v_rot_robot_prev, dv_rot_robot;
  arma::vec Forient; // Torques exerted to the robot by the environment (human, objects etc.)

  arma::vec Fee;
  arma::vec F_dead_zone; // 6x1 vector with the force and torque dead zone values.

  // Used to implement the position and orientation error in the admittance controller.
  arma::vec ddEp;
	arma::vec dEp;
	arma::vec ddEo;
	arma::vec dEo;

	arma::vec Ep;

  arma::vec sim_mse;

	arma::vec q0_robot, q_prev_robot, q_robot, dq_robot;
  arma::mat T_robot_ee; // stores the robot's forwarda kinematic
  arma::mat J_robot; // stoes the robot's Jacobian

};

#endif // ROBOT_PbD_CONTROLLER_H
