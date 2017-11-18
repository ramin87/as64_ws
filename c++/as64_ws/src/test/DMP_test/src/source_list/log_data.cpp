#include <log_data.h>

LogData::LogData()
{}


void LogData::save(const std::string &filename, bool binary, int precision)
{
  std::string file_ext = binary?".bin":".txt";
  std::string file = filename + file_ext;

  std::ofstream out;

  if (binary){
    out.open(file_ext, std::ios::binary);
  }
  else
  {
    out.open(file_ext);
  }

  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: \"") + file_ext + "\"");

  as64_::io_::write_mat(Time_demo, out, binary, precision);
  as64_::io_::write_mat(yd_data, out, binary, precision);
  as64_::io_::write_mat(dyd_data, out, binary, precision);
  as64_::io_::write_mat(ddyd_data, out, binary, precision);

  as64_::io_::write_scalar(D, out, binary, precision);
  if (!binary) out << "\n";

  as64_::io_::write_scalar(Ts, out, binary, precision);
  if (!binary) out << "\n";

  as64_::io_::write_mat(g0, out, binary, precision);

  as64_::io_::write_mat(Time_offline_train, out, binary, precision);
  as64_::io_::write_mat(F_offline_train_data, out, binary, precision);
  as64_::io_::write_mat(Fd_offline_train_data, out, binary, precision);

  as64_::io_::write_mat(Time_online_train, out, binary, precision);
  as64_::io_::write_mat(F_online_train_data, out, binary, precision);
  as64_::io_::write_mat(Fd_online_train_data, out, binary, precision);

  as64_::io_::write_mat(Time, out, binary, precision);
  as64_::io_::write_mat(y_data, out, binary, precision);
  as64_::io_::write_mat(dy_data, out, binary, precision);
  as64_::io_::write_mat(z_data, out, binary, precision);
  as64_::io_::write_mat(dz_data, out, binary, precision);
  as64_::io_::write_mat(x_data, out, binary, precision);
  as64_::io_::write_mat(u_data, out, binary, precision);


  as64_::io_::write_mat(y_robot_data, out, binary, precision);
  as64_::io_::write_mat(dy_robot_data, out, binary, precision);

  as64_::io_::write_mat(Fdist_data, out, binary, precision);

  as64_::io_::write_mat(Force_term_data, out, binary, precision);
  as64_::io_::write_mat(g_data, out, binary, precision);

  as64_::io_::write_vec_mat(Psi_data, out, binary, precision);
  as64_::io_::write_mat(shape_attr_data, out, binary, precision);
  as64_::io_::write_mat(goal_attr_data, out, binary, precision);


  as64_::io_::write_vec_mat(P_lwr, out, binary, precision);
  as64_::io_::write_vec_mat(DMP_w, out, binary, precision);

//   std::vector<std::shared_ptr<as64::DMP_>> dmp;
//
//   CMD_ARGS cmd_args;

  out.close();
}
