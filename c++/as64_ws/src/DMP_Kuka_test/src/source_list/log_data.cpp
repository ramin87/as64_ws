#include <log_data.h>

LogData::LogData()
{
  poseDataFlag = false;
}


void LogData::save(const std::string &filename, bool binary, int precision)
{
  std::string file_ext = binary?".bin":".txt";
  std::string file = filename + file_ext;

  std::ofstream out;

  if (binary)
  {
    out.open(file, std::ios::binary);
  }
  else
  {
    out.open(file);
  }

  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: \"") + file_ext + "\"");

  as64_::io_::write_mat(Time_demo, out, binary, precision);
  as64_::io_::write_mat(yd_data, out, binary, precision);
  as64_::io_::write_mat(dyd_data, out, binary, precision);
  as64_::io_::write_mat(ddyd_data, out, binary, precision);

  as64_::io_::write_scalar((long)D, out, binary, precision);
  if (!binary) out << "\n";

  as64_::io_::write_scalar(Ts, out, binary, precision);
  if (!binary) out << "\n";

  as64_::io_::write_mat(g0, out, binary, precision);

  as64_::io_::write_mat(Time_offline_train, out, binary, precision);
  as64_::io_::write_mat(F_offline_train_data, out, binary, precision);
  as64_::io_::write_mat(Fd_offline_train_data, out, binary, precision);
  as64_::io_::write_vec_mat(Psi_data_train, out, binary, precision);

  as64_::io_::write_mat(Time, out, binary, precision);
  as64_::io_::write_mat(y_data, out, binary, precision);
  as64_::io_::write_mat(dy_data, out, binary, precision);
  as64_::io_::write_mat(z_data, out, binary, precision);
  as64_::io_::write_mat(dz_data, out, binary, precision);
  as64_::io_::write_mat(x_data, out, binary, precision);
  as64_::io_::write_mat(goalAttr_data, out, binary, precision);
  as64_::io_::write_mat(shapeAttr_data, out, binary, precision);
  as64_::io_::write_vec_mat(Psi_data, out, binary, precision);

  as64_::io_::write_mat(y_robot_data, out, binary, precision);
  as64_::io_::write_mat(dy_robot_data, out, binary, precision);
  as64_::io_::write_mat(ddy_robot_data, out, binary, precision);

  as64_::io_::write_mat(Fdist_data, out, binary, precision);

  as64_::io_::write_mat(Force_term_data, out, binary, precision);
  as64_::io_::write_mat(g_data, out, binary, precision);

  as64_::io_::write_vec_mat(DMP_w, out, binary, precision);
  as64_::io_::write_vec_mat(DMP_c, out, binary, precision);
  as64_::io_::write_vec_mat(DMP_h, out, binary, precision);

  as64_::io_::write_scalar(poseDataFlag, out, binary, precision);

  out.close();
}
