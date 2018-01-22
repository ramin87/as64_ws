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

#include <DS_lib/DS_GMR/DS_GMR.h>

#include <utils.h>

#include <math_lib/math.h>
#include <math_lib/quaternions.h>
#include <filter_lib/filter.h>
#include <time_lib/time.h>


LogData logData;
CMD_ARGS cmd_args;
bool  zero_vel;
double Ts;
Eigen::Matrix<double,CART_DOF_SIZE,1> V_endEffector, Vd_endEffector, V_objTarget_obj;
double v_scale;

double get_pose_error(const Eigen::Matrix4d &T)
{
	Eigen::Vector3d pos_err = T.block(0,3,3,1);

	Eigen::Vector4d quat = as64_::rotm2quat(T.block(0,0,3,3));
	Eigen::Vector3d orient_err = quat.segment(1,3);
	double r = cmd_args.POS_ERR_COEFF * pos_err.squaredNorm() + (1-cmd_args.POS_ERR_COEFF) * orient_err.squaredNorm();

	return r;
}

void calc_DS_input(const Eigen::Matrix4d &T_objTarget_obj, Eigen::Matrix<double,6,1> *X_in)
{
// 	Eigen::Vector3d pos_err = T_objTarget_obj.block(0,3,3,1);
// 	X_in->segment(0,3) = pos_err;
// 	Eigen::Vector4d quat_err = rotm2quat(T_objTarget_obj.block(0,0,3,3));
// 	Eigen::Vector3d orient_err = quat_err.segment(1,3);
// 	X_in->segment(3,3) = orient_err;

	X_in->segment(0,3) = T_objTarget_obj.block(0,3,3,1);
	X_in->segment(3,3) = as64_::rotm2quat(T_objTarget_obj.block(0,0,3,3)).segment(1,3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_DS_motion_node");

  V_endEffector = Vd_endEffector = Eigen::Matrix<double,CART_DOF_SIZE,1>::Zero();
  zero_vel = false;
  Ts = 0.001;

  logData.Ts = Ts;

  // ************************
  // ***  Parse cmd args  ***
  // ************************
  cmd_args.parse_cmd_args();
  cmd_args.print();


  // ************************
  // ***  initialize DS  ***
  // ************************
  boost::shared_ptr<DS::DS_abstract> ds = boost::shared_ptr<DS::DS_abstract>(new DS::DS_GMR());
  std::cout << "==> Created DS!\n";
  if (!ds->load_ds_params(cmd_args.DS_params_filename.c_str())){
	  std::cerr << "Couldn't load \"" << cmd_args.DS_params_filename << "\"...\n";
	  exit(-1);
  }
  std::cout << "Loaded params successfully!\n";

  // ****************************************
  // ***  Read init and target transform  ***
  // ****************************************
  Eigen::Matrix4d T_base_start, T_base_target;

  std::ifstream in_init_pose(cmd_args.init_pose_filename.c_str(), std::ios::in);
  if (!in_init_pose){
	  std::cerr << "Error opening \"" << cmd_args.init_pose_filename << "\"...\n";
	  return -1;
  }

  std::ifstream in_target_pose(cmd_args.target_pose_filename.c_str(), std::ios::in);
  if (!in_target_pose){
    std::cerr << "Error opening \"" << cmd_args.target_pose_filename << "\"...\n";
    return -1;
  }

  // bypass the joint values
  double dummy;
  for (int i=0;i<JOINT_SIZE;i++) in_init_pose >> dummy;

  for (int i=0;i<4;i++){
    for (int j=0;j<4;j++){
      in_init_pose >> T_base_start(i,j);
      in_target_pose >> T_base_target(i,j);
    }
  }
  in_init_pose.close();
  in_target_pose.close();

  Eigen::Matrix4d T_objTarget_obj =  T_base_target.inverse() * T_base_start;

  int max_iters = 20000;
  int iter = 0;

  while (true){
    Eigen::Matrix4d T_objTarget_obj2 = T_objTarget_obj;

    double r = get_pose_error(T_objTarget_obj);
    if (r < 1e-3) break;

    std::cout << "pose_err =" << r << "\n";

    bool mirror_occured = false;

    double x_pos = T_objTarget_obj(0,3);
    if (!zero_vel){
	double robot_x_pos;
	if (cmd_args.USE_MIRRORING){
	  robot_x_pos = T_objTarget_obj2(0,3);
	  if (robot_x_pos > cmd_args.MIRROR_THRES){
	    mirror_Htrans_wrt_yz_plane(&T_objTarget_obj2);
	    mirror_occured = true;
	  }
	}

	Eigen::Matrix<double, 6, 1> X_in;
	calc_DS_input(T_objTarget_obj2, &X_in);
	Eigen::MatrixXd X_out;
	ds->get_ds_output(X_in, &X_out);
	V_objTarget_obj = X_out.block(0,0,6,1);

	if (mirror_occured){
	  mirror_Velocity_wrt_yz_plane(&V_objTarget_obj);
	  if (robot_x_pos < cmd_args.MIRROR_SMOOTH_THRES){
	    Eigen::Matrix<double, 6, 1> V2 = V_objTarget_obj;
	    calc_DS_input(T_objTarget_obj, &X_in);
	    ds->get_ds_output(X_in, &X_out);
	    Eigen::Matrix<double, 6, 1> V1 = X_out.block(0,0,6,1);

	    V_objTarget_obj = (V2-V1)*robot_x_pos/cmd_args.MIRROR_SMOOTH_THRES + V1;
	  }
	}
	Vd_endEffector = V_objTarget_obj;
    }else Vd_endEffector = V_endEffector*cmd_args.kV; //+ Ts*(-cmd_args.kV*V_endEffector);


    if (cmd_args.NORM_VEL){
      Vd_endEffector = Vd_endEffector/Vd_endEffector.norm();
      double r2 = (r>1)?1:r;
      v_scale = cmd_args.V_SCALE_C*r2 / (std::pow(1-r2,cmd_args.V_SCALE_EXP)+cmd_args.V_SCALE_C*r2);
      Vd_endEffector *= cmd_args.VEL_SCALE*v_scale;
    }else{
      Vd_endEffector = cmd_args.VEL_SCALE*Vd_endEffector;
      v_scale = 1;
    }

    Eigen::Matrix<double, 6, 1> eV = (Vd_endEffector - V_endEffector);
    Vd_endEffector = cmd_args.ks*Vd_endEffector + (1-cmd_args.ks)*V_endEffector;

    Eigen::Vector3d pos = T_objTarget_obj.block(0,3,3,1);
    Eigen::Vector4d quat = as64_::rotm2quat(T_objTarget_obj.block(0,0,3,3));

    Eigen::Vector3d v_lin = Vd_endEffector.segment(0,3);
    Eigen::Vector3d v_rot = Vd_endEffector.segment(3,3);

    pos += v_lin*Ts;
    quat += as64_::get_quat_dot(v_rot,quat)*Ts;
    quat = quat/quat.norm();
    //quat = as64_::quatProd(as64_::quatExp(v_rot*Ts), quat);

    T_objTarget_obj.block(0,0,3,3) = as64_::quat2rotm(quat);
    T_objTarget_obj.block(0,3,3,1) = pos;
    V_endEffector = Vd_endEffector;

    double w_est_filt = 0;
    Eigen::Matrix<double, JOINT_SIZE, 1> q = Eigen::Matrix<double, JOINT_SIZE, 1>::Zero();
    Eigen::Matrix<double, JOINT_SIZE, 1> q_dot = q;
    Eigen::Matrix4d T_cam_objTarget = T_base_target;
    Eigen::Matrix4d T_cam_obj = T_base_target*T_objTarget_obj;

    if (cmd_args.LOG_DATA) logData.push(q, q_dot, V_endEffector, V_objTarget_obj, T_cam_obj, T_cam_objTarget, w_est_filt, v_scale, mirror_occured);

    iter++;
    if (iter > max_iters) break;
  }

  if (cmd_args.LOG_DATA) logData.save(cmd_args.output_path + "test_DS_motion_results.txt");

  return 0;

}
