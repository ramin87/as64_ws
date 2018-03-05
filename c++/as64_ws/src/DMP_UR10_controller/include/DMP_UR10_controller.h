/**
 * Copyright (C) 2017 as64_
 */

#ifndef DMP_UR10_CONTROLLER_H
#define DMP_UR10_CONTROLLER_H

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

#include <utils.h>
#include <cmd_args.h>

#include <ur10_robot/ur10_robot.h>

#include <dmp_lib/dmp_lib.h>
#include <param_lib/param_lib.h>
#include <io_lib/io_lib.h>
#include <sigproc_lib/sigproc_lib.h>
//#include <plot_lib/plot_lib.h>
#include <math_lib/math_lib.h>


using namespace as64_;

class DMP_UR10_controller
{
  enum Mode{
    STOP = 0,
    IDLE = 1,
    FREEDRIVE = 2,
    MODEL_RUN = 3,
    POSITION_CONTROL = 4
  };

  struct TrainData
  {
    int n_data;
    arma::vec q0;
    arma::rowvec Time;
    arma::mat Y_data, dY_data, ddY_data;
    arma::mat Q_data, v_rot_data, dv_rot_data;
  };

public:
  // Constructor that initializes variables that must be set once at the start of the program.
  DMP_UR10_controller(std::shared_ptr<ur10_::Robot> robot);

  // Initializes the control flags and some other variables that must be reset at each
  // repetition of the task in the same program run.
  void initController();

  // Initializes the values of the program's control flags
  void initControlFlags();

  // Initializes the program's variables.
  void initModelexecution();

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

  // Trains the DMP using the data in the struct 'trainData'.
  void trainModel();

  // Gets the derivatives of the DMP states and numerically integrates them.
  void runModel();

  // Saves in the struct 'trainData' the current robot pose, velocities and accelerations.
  void logDemoStep();

  // Logs the DMP and robot's poses, velocities and accelerations.
  void logOnline();

  // Logs some data calculated after the execution of the DMP such as the forcing term,
  //  goal/shape attractor values, kernel functions activations, dmp weights, centers, stds etc.
  void logOffline();

  // Clears all data in struct 'trainData'
  void clearTrainData();

  // Clears all data in struct 'log_data'
  void clearLoggedData();

  // Saves all data from struct 'log_data' in an output file.
  void saveLoggedData();

  void saveDemoData();
  void loadDemoData();

  // Uses the keyboard as interface between the user and the program.
  // Runs on the different thread from the main program.
  void keyboardCtrlThreadFun();
  void saveExecutionResults();

  DMP_UR10_controller::Mode getMode() const;
  void setMode(const DMP_UR10_controller::Mode &mode);
  std::string getModeName() const;

  bool targetReached() const;

  void printHelpMenu() const;
  void printKeys() const;

  void loadDataAndTrainModel();
private:

  Mode mode;
  std::vector<std::string> modeName;

  // Program control flags
  bool save_exec_results;
  bool log_on; // if true, at each step of the execution, intermediate data are logged.
  //bool run_dmp; // if true the dmp is executed
  bool train_model; // if true the DMP is (re)trained
  bool model_trained; // true if the model has been trained
  bool goto_start; // if true the robot goes to its starting pose
  //bool stop_robot; // if true the program terminates
  //bool pause_robot;
  bool record_demo_on; // if true the program starts logging data in the "trainData" struct
  bool save_rerecord_demo_on; // if true saves all loged data and restarts the demo process
  bool clear_rerecord_demo_on; // if true clears all loged data (without saving them) and restarts the demo process
  bool start_pose_set; // true if a start pose has been registered using recordDemo
  bool train_data_loaded;

  int demo_save_counter; // add a number to the data output file to avoid overriding the previous one

  std::shared_ptr<std::thread> keyboard_ctrl_thread; // thread for the "keyboardCtrlThreadFun()"
  std::shared_ptr<std::thread> save_exec_results_thread; // thread for the "keyboardCtrlThreadFun()"
  std::shared_ptr<std::thread> train_model_thread; // thread for the "keyboardCtrlThreadFun()"

  arma::wall_clock timer; // timer to measure elapsed time

  CMD_ARGS cmd_args; // used to parse and store the arguments from the yaml file
  LogData log_data; // struct used for logging data on/off-line
  TrainData trainData; // Data used for (re)training the DMP

  // for the DMP
  std::shared_ptr<as64_::CanonicalClock> canClockPtr; // pointer to the canonical clock
  std::shared_ptr<as64_::GatingFunction> shapeAttrGatingPtr; // pointer to the shape attractor gating
  std::shared_ptr<as64_::GatingFunction> goalAttrGatingPtr; // pointer to the goal attractor gating
  std::vector<std::shared_ptr<as64_::DMP_>> dmp; // vectors of DMP supplied to the CartPos and orient DMP.
  std::shared_ptr<as64_::DMP_CartPos> dmpCartPos; // Cartesian Position DMP
  std::shared_ptr<as64_::DMP_orient> dmpOrient; // Orientation DMP


  double Ts; // robot control cycle, used for numerical integration
  double tau; // the total duration of the demonstrated movement
  int Dp; // dimensionality of the Cartesian Position DMP
  int Do; // dimensionality of the orientation DMP
  int D; // Dp + Do


  arma::vec offline_train_p_mse; // MSE of the Forcing term during the CartPos DMP training
  arma::vec offline_train_o_mse; // MSE of the Forcing term during the orient DMP training

  double t; // holds the current time value during demo or execution
  double x, dx; // DMP phase variable and its derivative

  // Y stands for column vector with Cartesian Position (x,y,z)
  arma::vec Yg0, Yg, Yg2, dg_p;
  arma::vec Y0, Y, dY, ddY;
  arma::vec Y_robot, dY_robot, dY_robot_prev, ddY_robot;
  arma::vec V_robot;
  arma::vec Z, dZ;
  arma::vec Fdist_p; // Cartesian Forces exerted to the robot by the environment (human, objects etc.)

  // Q expresses a quaternion
  arma::vec Qg0, Qg, Qg2, dg_o;
  arma::vec Q0, Q, dQ, v_rot, dv_rot;
  arma::vec Q_robot, v_rot_robot, v_rot_robot_prev, dv_rot_robot;
  arma::vec eta, deta;
  arma::vec Fdist_o; // Torques exerted to the robot by the environment (human, objects etc.)

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

	std::shared_ptr<ur10_::Robot> robot_;

};

#endif // DMP_UR10_CONTROLLER_H
