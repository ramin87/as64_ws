#include <ur10_robot.h>
#include <math_lib/math_lib.h>

namespace ur10_
{
  Robot::Robot()
  {
    this->command_ur10_topic = "/ur_driver/URScript";
    this->read_wrench_topic = "/wrench";
    this->read_toolVel_topic = "/tool_velocity";
    this->read_jointState_topic = "/joint_states";
    this->base_frame = "base";
    this->tool_frame = "tool0_controller";
    this->pub_rate = 60;

    this->pub2ur10 = n.advertise<std_msgs::String>(this->command_ur10_topic, 1);

    wrench_sub = n.subscribe(this->read_wrench_topic, 1, &Robot::readWrenchCallback, this);
    toolVel_sub = n.subscribe(this->read_toolVel_topic, 1, &Robot::readToolVelCallback, this);
    jointState_sub = n.subscribe(this->read_jointState_topic, 1, &Robot::readJointStateCallback, this);

    this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));

    ros::Duration(4.0).sleep(); // needed to let ur initialize
  }

  void Robot::freedrive_mode() const
  {
    urScript_command("freedrive_mode()");
  }

  void Robot::end_freedrive_mode() const
  {
    urScript_command("end_freedrive_mode()");
  }

  void Robot::teach_mode() const
  {
    urScript_command("teach_mode()");
  }

  void Robot::end_teach_mode() const
  {
    urScript_command("end_teach_mode()");
  }

  void Robot::force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits)  const
  {
    if (type<1 || type>3) throw std::invalid_argument("[Error]: Robot::force_mode: type must be in {1,2,3}");
    std::ostringstream out;
    out << "force_mode(p" << print_vector(task_frame) << "," << print_vector(selection_vector) << ","
        << print_vector(wrench) << "," << type << "," << print_vector(limits) << ")";
    urScript_command(out.str());
    //sleep(0.02);
    ros::Duration(0.02).sleep();
  }

  void Robot::end_force_mode() const
  {
    urScript_command("end_force_mode()");
  }

  void Robot::force_mode_set_damping(double damping)  const
  {
    if (damping<0)
    {
      damping = 0.0;
      std::cerr << "[WARNING]: Robot::force_mode_set_damping: Saturating damping to 0.0";
    }

    if (damping>1)
    {
      damping = 1.0;
      std::cerr << "[WARNING]: Robot::force_mode_set_damping: Saturating damping to 1.0";
    }

    std::ostringstream out;
    out << "force_mode_set_damping(" << damping << ")";
    urScript_command(out.str());
  }

  void Robot::movej(const arma::vec &q, double a, double v, double t, double r) const
  {
    std::ostringstream out;
    out << "movej(" << print_vector(q) << "," << a << "," << v << "," << t << "," << r << ")";
    urScript_command(out.str());
  }

  void Robot::movel(const arma::vec &p, double a, double v, double t, double r) const
  {
    std::ostringstream out;
    out << "movel(p" << print_vector(p) << "," << a << "," << v << "," << t << "," << r << ")";
    urScript_command(out.str());
  }

  void Robot::speedj(arma::vec dq, double a, double t) const
  {
    std::ostringstream out;
    out << "speedj(" << print_vector(dq) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")";
    urScript_command(out.str());
  }

  void Robot::speedl(arma::vec dp, double a, double t) const
  {
    std::ostringstream out;
    out << "speedl(" << print_vector(dp) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")";
    urScript_command(out.str());
  }

  void Robot::stopj(double a) const
  {
    std::ostringstream out;
    out << "stopj(" << a << ")";
    urScript_command(out.str());
  }

  void Robot::stopl(double a) const
  {
    std::ostringstream out;
    out << "stopl(" << a << ")";
    urScript_command(out.str());
  }

  void Robot::set_gravity(const arma::vec &g) const
  {
    std::ostringstream out;
    out << "set_gravity(" << print_vector(g) << ")";
    urScript_command(out.str());
  }

  void Robot::set_payload(double m, const arma::vec &CoG) const
  {
    std::ostringstream out;
    out << "set_payload(" << m << "," << print_vector(CoG) << ")";
    urScript_command(out.str());
  }

  void Robot::set_payload_cog(const arma::vec &CoG) const
  {
    std::ostringstream out;
    out << "set_payload_cog(" << print_vector(CoG) << ")";
    urScript_command(out.str());
  }

  void Robot::set_payload_mass(double m) const
  {
    std::ostringstream out;
    out << "set_payload_mass(" << m << ")";
    urScript_command(out.str());
  }

  void Robot::set_tcp(const arma::vec &pose) const
  {
    std::ostringstream out;
    out << "set_tcp(p" << print_vector(pose) << ")";
    urScript_command(out.str());
  }

  void Robot::sleep(double t) const
  {
    std::ostringstream out;
    out << "sleep(" << t << ")";
    urScript_command(out.str());
  }

  void Robot::powerdown() const
  {
    urScript_command("powerdown()");
  }

  void Robot::waitNextCycle()
  {
    ros::spinOnce();
    this->readTaskPoseCallback();
  }

  void Robot::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    rSt.wrench << msg->wrench.force.x << msg->wrench.force.y << msg->wrench.force.z
              << msg->wrench.torque.x << msg->wrench.torque.y << msg->wrench.torque.z;
  }

  void Robot::readToolVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    rSt.v_lin << msg->twist.linear.x  << msg->twist.linear.y  << msg->twist.linear.z;
    rSt.v_rot << msg->twist.angular.x  << msg->twist.angular.y  << msg->twist.angular.z;
  }

  void Robot::readJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    int len = msg->position.size();
    rSt.q.resize(len);
    rSt.dq.resize(len);
    rSt.jTorques.resize(len);
    for (int i=0;i<len; i++)
    {
      rSt.q(i) = msg->position[i];
      rSt.dq(i) = msg->velocity[i];
      rSt.jTorques(i) = msg->effort[i];
    }

  }

  void Robot::readTaskPoseCallback()
  {
    try{
       this->transformStamped = tfBuffer.lookupTransform(this->base_frame, this->tool_frame, ros::Time(0));
       rSt.pos << this->transformStamped.transform.translation.x
               << this->transformStamped.transform.translation.y
               << this->transformStamped.transform.translation.z;

       rSt.Q << this->transformStamped.transform.rotation.w
             << this->transformStamped.transform.rotation.x
             << this->transformStamped.transform.rotation.y
             << this->transformStamped.transform.rotation.z;

       rSt.pose.submat(0,0,2,2) = as64_::math_::quat2rotm(rSt.Q);
       rSt.pose.submat(0,3,2,3) = rSt.pos;

       // double time = this->transformStamped.header.stamp.sec;
       // std::cout << "*** time = "<< std::setprecision(10)  << time << "\n";

     }
     catch (tf2::TransformException &ex) {
       ROS_WARN("%s",ex.what());
       // ros::Duration(1.0).sleep();
     }
  }

  void Robot::print_robot_state(std::ostream &out) const
  {
    out << "===============================\n";
    out << "=======   Robot state  ========\n";
    out << "===============================\n";
    out << "joint pos: " << getJointPosition().t()*180/arma::datum::pi << "\n";
    out << "joint vel: " << getJointVelocity().t() << "\n";
    out << "joint Torques: " << getJointTorque().t() << "\n";

    out << "Cart pos: " << getTaskPosition().t() << "\n";
    out << "Cart orient: " << getTaskOrientation().t() << "\n";
    out << "Twist: " << getTwist().t() << "\n";
    out << "Wrench: " << getExternalWrench().t() << "\n";

    out << "===============================\n";
    out << "===============================\n";
  }

} // namespace ur10_
