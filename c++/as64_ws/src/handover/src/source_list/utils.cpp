#include <utils.h>

/*
 * Pauses the program until the user presses the keyboard
 */ 
void pause_program()
{ 
	printf("Program paused...(Press enter to continue)"); 
	fflush(stdin); scanf("%*c"); 
}

void print_error_message(const std::string &err_msg)
{
	std::cerr << "Error: " << err_msg << std::endl;
	pause_program();
}

/*
 * Converts a number to string represantation
 */
std::string numToString(double number)
{ 
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void mirror_Htrans_wrt_yz_plane(Eigen::Matrix4d *Htrans)
{
	(*Htrans)(0,1) = -(*Htrans)(0,1);
	(*Htrans)(0,2) = -(*Htrans)(0,2);
	(*Htrans)(0,3) = -(*Htrans)(0,3);
	
	(*Htrans)(1,0) = -(*Htrans)(1,0);
	(*Htrans)(2,0) = -(*Htrans)(2,0);
	
}

void mirror_Velocity_wrt_yz_plane(Eigen::Matrix<double, 6, 1> *Vel)
{
	(*Vel)(0) = -(*Vel)(0);
	(*Vel)(4) = -(*Vel)(4);
	(*Vel)(5) = -(*Vel)(5);
}


void transform_Jacob(const Eigen::Matrix4d &T_a_b, const Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> &J_a, Eigen::Matrix<double,CART_DOF_SIZE,JOINT_SIZE> &J_b)
{
  Eigen::Matrix<double, 6, 6> Tjac_b_a = Eigen::Matrix<double, 6, 6>::Identity();
  Tjac_b_a.block(0,0,3,3) = T_a_b.block(0,0,3,3).transpose();
  Tjac_b_a.block(3,3,3,3) = T_a_b.block(0,0,3,3).transpose();
  J_b = Tjac_b_a * J_a;
}

// **********************************
// **********    LogData   **********
// **********************************
void LogData::push(const Eigen::Matrix<double, JOINT_SIZE, 1> &q, 
			const Eigen::Matrix<double, JOINT_SIZE, 1> &dq, 
			const Eigen::Matrix<double, CART_DOF_SIZE, 1> &V_endEffector,
			const Eigen::Matrix<double, CART_DOF_SIZE, 1> &V_objTarget_obj,
			const Eigen::Matrix<double, 4, 4> &T_cam_obj,
			const Eigen::Matrix<double, 4, 4> &T_cam_objTarget,
			double Fee_z,
			const double &v_scale,
			const bool &mirror_occured)
			{
				q_data.push_back(q);
				dq_data.push_back(dq);
				V_endEffector_data.push_back(V_endEffector);
				V_objTarget_obj_data.push_back(V_objTarget_obj);
				T_cam_obj_data.push_back(T_cam_obj);
				T_cam_objTarget_data.push_back(T_cam_objTarget);
				Fee_z_data.push_back(Fee_z);
				v_scale_data.push_back(v_scale);
				mirror_occured_data.push_back(mirror_occured);
			}
			
bool LogData::save(const std::string &filename)
{
	std::ofstream out(filename);
	if (!out){
		std::cerr << "Couldn't create file: \"" << filename << "\"...\n";
		return false;
	}
	
	out.precision(10);
	
	out << Ts  << "\n";
	
	out << q_data.size() << "\n";
	for (int i=0;i<q_data.size();i++) out << q_data[i].transpose() << "\n";
	
	out << "\n";
	for (int i=0;i<dq_data.size();i++) out << dq_data[i].transpose() << "\n";
	
	out << "\n";
	for (int i=0;i<V_endEffector_data.size();i++) out << V_endEffector_data[i].transpose() << "\n";
	
	out << "\n";
	for (int i=0;i<V_objTarget_obj_data.size();i++) out << V_objTarget_obj_data[i].transpose() << "\n";
	
	out << "\n";
	for (int i=0;i<T_cam_obj_data.size();i++) out << T_cam_obj_data[i] << "\n";
	
	out << "\n";
	for (int i=0;i<T_cam_objTarget_data.size();i++) out << T_cam_objTarget_data[i] << "\n";
	
	out << "\n";
	for (int i=0;i<v_scale_data.size();i++) out << v_scale_data[i] << "\n";
	
	out << "\n";
	for (int i=0;i<mirror_occured_data.size();i++) out << mirror_occured_data[i] << "\n";
	
	out << "\n";
	for (int i=0;i<Fee_z_data.size();i++) out << Fee_z_data[i] << "\n";
	
	return true;
}
	

// ***********************************
// **********    CMD_ARGS   **********
// ***********************************

CMD_ARGS::CMD_ARGS() {}

CMD_ARGS::CMD_ARGS(int program_execution_choice, double vel_scale, double tol_stop, bool log_data, bool use_mirroring)
{
	p_ch = program_execution_choice;
	VEL_SCALE = vel_scale;
	TOL_STOP = tol_stop;
	LOG_DATA = log_data;
	USE_MIRRORING = use_mirroring;
}

bool CMD_ARGS::parse_cmd_args(int argc, char **argv)
{
	//pcl::console::parse_argument(argc, argv, "-program_execution_choice", p_ch);
	p_ch = atoi(argv[1]);
	
	pcl::console::parse_argument(argc, argv, "-TOL_STOP", TOL_STOP);
	
	pcl::console::parse_argument(argc, argv, "-LOG_DATA", LOG_DATA);
	
	pcl::console::parse_argument(argc, argv, "-USE_MIRRORING", USE_MIRRORING);
	pcl::console::parse_argument(argc, argv, "-MIRROR_THRES", MIRROR_THRES);
	
	pcl::console::parse_argument(argc, argv, "-OLT_THRES", OLT_THRES);
	pcl::console::parse_argument(argc, argv, "-WEIGHT_EST_FILT_N", WEIGHT_EST_FILT_N);
	
	pcl::console::parse_argument(argc, argv, "-c", c);
	
	pcl::console::parse_argument(argc, argv, "-HIGH_VEL_TOL_STOP", HIGH_VEL_TOL_STOP);
	pcl::console::parse_argument(argc, argv, "-START_MOVE_THRES", START_MOVE_THRES);
	
	pcl::console::parse_argument(argc, argv, "-kV", kV);
	pcl::console::parse_argument(argc, argv, "-ks", ks);
	pcl::console::parse_argument(argc, argv, "-VEL_SCALE", VEL_SCALE);
	pcl::console::parse_argument(argc, argv, "-V_Vd_equal", V_Vd_equal);
	pcl::console::parse_argument(argc, argv, "-MAX_R", MAX_R);
	pcl::console::parse_argument(argc, argv, "-V_SCALE_C", V_SCALE_C);
	pcl::console::parse_argument(argc, argv, "-V_SCALE_EXP", V_SCALE_EXP);
	
	pcl::console::parse_argument(argc, argv, "-NORM_VEL", NORM_VEL);
	
	return true;	
}

bool CMD_ARGS::parse_cmd_args(const std::string &filename)
{
	int argc;
	char **argv;
	std::ifstream in_cmd_args(filename.c_str(), std::ios::in);

	if (!in_cmd_args){
		std::cerr << "Error opening \"" << filename << "\"...\n";
		return false;
	}
	in_cmd_args >> argc;
	argc = 2*argc;
	
	argv = (char **)malloc(argc*sizeof(char *));
	if (!argv) return false;
	
	for (int i=0;i<argc;i++){
		std::string c;
		in_cmd_args >> c;
		//std::cout << c << "\n";
		argv[i] = (char *)malloc((c.size()+1)*sizeof(char));
		if (!argv[i]) return false;
		strcpy(argv[i], c.c_str());
		//std::cout << argv[i] << "\n";
	}
	in_cmd_args.close();
	
	bool ret = parse_cmd_args(argc, argv);
	
	for (int i=0;i<argc;i++) free(argv[i]);
	free(argv);
	
	return ret;
}

bool CMD_ARGS::parse_cmd_args()
{
	ros::NodeHandle nh_ = ros::NodeHandle("~");
	
	if (!nh_.getParam("program_execution_choice", p_ch)) p_ch = 1;
	
	if (!nh_.getParam("LOG_DATA", LOG_DATA)) LOG_DATA = true;
	if (!nh_.getParam("output_path", output_path)) output_path = "";

	if (!nh_.getParam("USE_MIRRORING", USE_MIRRORING)) USE_MIRRORING = true;
	if (!nh_.getParam("MIRROR_THRES", MIRROR_THRES)) MIRROR_THRES = 0.0;
	if (!nh_.getParam("MIRROR_SMOOTH_THRES", MIRROR_SMOOTH_THRES)) MIRROR_SMOOTH_THRES = 0.0025;
	
	if (!nh_.getParam("OLT_THRES", OLT_THRES)) OLT_THRES = 3.0;
	if (!nh_.getParam("WEIGHT_EST_FILT_N", WEIGHT_EST_FILT_N)) WEIGHT_EST_FILT_N = 50;
	if (!nh_.getParam("c", c)) c = 0.001;
	if (!nh_.getParam("TOL_STOP", TOL_STOP)) TOL_STOP == 0.003;
	if (!nh_.getParam("START_MOVE_THRES", START_MOVE_THRES)) START_MOVE_THRES = 0.75;
	if (!nh_.getParam("HIGH_VEL_TOL_STOP", HIGH_VEL_TOL_STOP)) HIGH_VEL_TOL_STOP = 0.65;
	if (!nh_.getParam("kV", kV)) kV = 0.994;
	if (!nh_.getParam("ks", ks)) ks = 0.005;
	if (!nh_.getParam("VEL_SCALE", VEL_SCALE)) VEL_SCALE = 0.35;
	if (!nh_.getParam("V_Vd_equal", V_Vd_equal)) V_Vd_equal = true;
	if (!nh_.getParam("MAX_R", MAX_R)) MAX_R = 1.0;
	if (!nh_.getParam("V_SCALE_C", V_SCALE_C)) V_SCALE_C = 20;
	if (!nh_.getParam("V_SCALE_EXP", V_SCALE_EXP)) V_SCALE_EXP = 5;
	if (!nh_.getParam("NORM_VEL", NORM_VEL)) NORM_VEL = false;
	
	if (!nh_.getParam("POS_ERR_COEFF", POS_ERR_COEFF)) POS_ERR_COEFF = 0.6;
	if (!nh_.getParam("ORIENT_ERR_COEFF", ORIENT_ERR_COEFF)) ORIENT_ERR_COEFF = 0.4;
	if (POS_ERR_COEFF+ORIENT_ERR_COEFF != 1) ORIENT_ERR_COEFF = 1 - POS_ERR_COEFF;
	
	if (!nh_.getParam("params_path", params_path)) params_path = "";
	if (!nh_.getParam("DS_params_filename", DS_params_filename)) DS_params_filename = "";
	if (!nh_.getParam("init_pose_filename", init_pose_filename)) init_pose_filename = "";
	if (!nh_.getParam("target_pose_filename", target_pose_filename)) target_pose_filename = "";
	if (!nh_.getParam("T_endEffector_obj_filename", T_endEffector_obj_filename)) T_endEffector_obj_filename = "";
	if (!nh_.getParam("T_robot_cam_filename", T_robot_cam_filename)) T_robot_cam_filename = "";
	
	if (!nh_.getParam("Kuka_to_Barrett_topic", Kuka_to_Barrett_topic)) T_robot_cam_filename = "Kuka_to_Barrett1";
	if (!nh_.getParam("cam_target_transform_topic", cam_target_transform_topic)) T_robot_cam_filename = "/cam_target_transform";
	
	// ====== Joint Limit Avoidance (Jla) ===========
	if (!nh_.getParam("JLAV_enable", JLAV_enable)) JLAV_enable = false;
	if (!nh_.getParam("JLAV_gain", JLAV_gain)) JLAV_gain = 0.008;

	// ====== Singular Value Filtering (SVF) ========
	if (!nh_.getParam("SVF_enable", SVF_enable)) SVF_enable = false;
	if (!nh_.getParam("SVF_sigma_min", SVF_sigma_min)) SVF_sigma_min = 0.001;
	if (!nh_.getParam("SVF_shape_factor", SVF_shape_factor)) SVF_shape_factor = 8;
	
	DS_params_filename = params_path + DS_params_filename;
	init_pose_filename = params_path + init_pose_filename;
	target_pose_filename = params_path + target_pose_filename;
	T_endEffector_obj_filename = params_path + T_endEffector_obj_filename;
	T_robot_cam_filename = params_path + T_robot_cam_filename;
}
	
void CMD_ARGS::print(std::ostream &out) const
{
	out << "program_execution_choice: " << p_ch << "\n";
	out << "TOL_STOP: " << TOL_STOP << "\n";
	
	out << "LOG_DATA: " << LOG_DATA << "\n";
	out << "output_path: " << output_path << "\n";
	
	out << "USE_MIRRORING: " << USE_MIRRORING << "\n";
	out << "MIRROR_THRES: " << MIRROR_THRES << "\n";
	out << "MIRROR_SMOOTH_THRES: " << MIRROR_SMOOTH_THRES << "\n";
	
	out << "POS_ERR_COEFF: " << POS_ERR_COEFF << "\n";
	out << "ORIENT_ERR_COEFF: " << ORIENT_ERR_COEFF << "\n";

	out << "OLT_THRES: " << OLT_THRES << "\n";
	out << "WEIGHT_EST_FILT_N: " << WEIGHT_EST_FILT_N << "\n";
	
	
	out << "HIGH_VEL_TOL_STOP: " << HIGH_VEL_TOL_STOP << "\n";
	out << "START_MOVE_THRES: " << START_MOVE_THRES << "\n";
	
	out << "kV: " << kV << "\n";
	out << "ks: " << ks << "\n";
	
	out << "VEL_SCALE: " << VEL_SCALE << "\n";
	
	out << "V_Vd_equal: " << V_Vd_equal << "\n";
	out << "MAX_R: " << MAX_R << "\n";
	out << "V_SCALE_C: " << V_SCALE_C << "\n";
	out << "V_SCALE_EXP: " << V_SCALE_EXP << "\n";
	
	out << "NORM_VEL: " << NORM_VEL << "\n";
	
	out << "DS_params_filename: " << DS_params_filename << "\n";
	out << "init_pose_filename: " << init_pose_filename << "\n";
	out << "target_pose_filename: " << target_pose_filename << "\n";
	out << "T_endEffector_obj_filename: " << T_endEffector_obj_filename << "\n";
	out << "T_robot_cam_filename: " << T_robot_cam_filename << "\n";
	
	out << "cam_target_transform_topic: " << cam_target_transform_topic << "\n";
	out << "Kuka_to_Barrett_topic: " << Kuka_to_Barrett_topic << "\n";
	
	out << " ====== Joint Limit Avoidance (Jlav) ===========\n";
	out << "JLAV_enable: " << JLAV_enable << "\n";
	out << "JLAV_gain: " << JLAV_gain << "\n";


	out << " ====== Singular Value Filtering (SVF) ========\n";
	out << "SVF_enable: " << SVF_enable << "\n";
	out << "SVF_sigma_min: " << SVF_sigma_min << "\n";
	out << "SVF_shape_factor: " << SVF_shape_factor << "\n";
	
}
