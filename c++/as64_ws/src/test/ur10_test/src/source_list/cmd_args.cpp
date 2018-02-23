#include <cmd_args.h>

CMD_ARGS::CMD_ARGS() {}

bool CMD_ARGS::parse_cmd_args()
{
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("command_ur10_topic", command_ur10_topic)) command_ur10_topic = "";
  if (!nh_.getParam("pub_rate", pub_rate)) pub_rate = 100.0;

}

void CMD_ARGS::print(std::ostream &out) const
{
  out << "command_ur10_topic: " << command_ur10_topic << "\n";
  out << "pub_rate: " << pub_rate << "\n";

}
