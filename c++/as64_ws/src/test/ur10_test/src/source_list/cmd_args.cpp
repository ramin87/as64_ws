#include <cmd_args.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args()
{
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("command_ur10_topic", command_ur10_topic)) command_ur10_topic = "";
  if (!nh_.getParam("pub_rate", pub_rate)) pub_rate = 100.0;
  if (!nh_.getParam("init_joint_pos", init_joint_pos)) init_joint_pos.resize(6);
  if (!nh_.getParam("goal_joint_pos", goal_joint_pos)) goal_joint_pos.resize(6);
  if (!nh_.getParam("joint_vel", joint_vel)) joint_vel = 0.5;
  if (!nh_.getParam("joint_accel", joint_accel)) joint_accel = 0.4;
  if (!nh_.getParam("Cart_vel", Cart_vel)) Cart_vel = 0.6;
  if (!nh_.getParam("Cart_accel", Cart_accel)) Cart_accel = 0.3;

  if (!nh_.getParam("Md", Md)) Md = 0.5;
  if (!nh_.getParam("Kd", Kd)) Kd = 1000;
  if (!nh_.getParam("Dd", Dd)) Dd = 2*std::sqrt(Md*Kd);

}

void CMD_ARGS::print(std::ostream &out) const
{
  out << "command_ur10_topic: " << command_ur10_topic << "\n";
  out << "pub_rate: " << pub_rate << "\n";

  out << "init_joint_pos: [";
  for (int i=0;i<init_joint_pos.size();i++) out << init_joint_pos[i] << " ";
  out << "]\n";

  out << "goal_joint_pos: [";
  for (int i=0;i<goal_joint_pos.size();i++) out << goal_joint_pos[i] << " ";
  out << "]\n";

  out << "joint_vel: " << joint_vel << "\n";
  out << "joint_accel: " << joint_accel << "\n";
  out << "Cart_vel: " << Cart_vel << "\n";
  out << "Cart_accel: " << Cart_accel << "\n";

  out << "Md: " << Md << "\n";
  out << "Kd: " << Kd << "\n";
  out << "Dd: " << Dd << "\n";

}
