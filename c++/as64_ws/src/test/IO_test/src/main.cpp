/**
 * Copyright (C) 2017 as64
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <IO_lib/io_utils.h>

using namespace as64_::io_;

int main(int argc, char** argv)
{    
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "IO_test_node");
  ros::NodeHandle nh_("~");
  
  std::string path = ros::package::getPath("io_test") + "/data/";

  // ===========  read cmd args  ==================
  bool binary;
  std::string filename;
  
  if (!nh_.getParam("binary", binary)) binary = false;
  if (!nh_.getParam("filename", filename)) filename = "data";

  filename = path + filename;
  if (binary) filename += ".bin";
  else filename += ".txt";
  
  std::cout << "filename = \"" << filename << "\"\n"; 
  
  // ===========  create data  ==================
  arma::mat A(24, 25, arma::fill::randu);
  arma::mat B(36, 28, arma::fill::randu);
  std::vector<arma::mat> m(2);
  m[0] = A;
  m[1] = B;
  arma::vec v(32, 1, arma::fill::randu);
  arma::rowvec rowV(1, 61, arma::fill::randu);
  
  
  // ===========  write data  ==================
  std::ofstream out;
  if (binary) out.open(filename, std::ios::out | std::ios::binary);
  else out.open(filename, std::ios::out);
  
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file \"") + filename + "\"...\n");

  write_mat(A, out, binary);
  write_rowVec(rowV, out, binary);
  write_mat(B, out, binary);
  write_vec(v, out, binary);
  write_vec_mat(m, out, binary);

  out.close();

  // ===========  read data  ==================
  std::ifstream in(filename, std::ios::in);
  if (!in) throw std::ios_base::failure(std::string("Couldn't open file \"") + filename + "\"...\n");

  arma::mat A2;
  arma::mat B2;
  std::vector<arma::mat> m2;
  arma::vec v2;
  arma::rowvec rowV2;

  read_mat(A2, in, binary);
  read_rowVec(rowV2, in, binary);
  read_mat(B2, in, binary);
  read_vec(v2, in, binary);
  read_vec_mat(m2, in, binary);

  in.close();

  //std::cout << "A2 = \n" << A2 << "\n";
  //std::cout << "rowV2 = \n" << rowV2 << "\n";
  //std::cout << "B2 = \n" << B2 << "\n";
  //std::cout << "v2 = \n" << v2 << "\n";
  //for (int k=0;k<m2.size();k++) std::cout << "m2["<<k<<"] = \n" << m2[k] << "\n";

  // ===========  test  ==================

  double A_err  = arma::norm(arma::vectorise(A)-arma::vectorise(A2));
  double B_err  = arma::norm(arma::vectorise(B)-arma::vectorise(B2));
  double v_err  = arma::norm(arma::vectorise(v)-arma::vectorise(v2));
  double rowV_err  = arma::norm(arma::vectorise(rowV)-arma::vectorise(rowV2));

  double m_err = 0;
  for (int k=0;k<m.size();k++){
      m_err = m_err + arma::norm(arma::vectorise(m[k])-arma::vectorise(m2[k]));
  }

  std::cout << "A_err = " << A_err << "\n";
  std::cout << "B_err = " << B_err << "\n";
  std::cout << "v_err = " << v_err << "\n";
  std::cout << "rowV_err = " << rowV_err << "\n";
  std::cout << "m_err = " << m_err << "\n";
  
  
  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
