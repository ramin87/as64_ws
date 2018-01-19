#include <io_lib/io_utils.h>

namespace as64_
{

namespace io_
{

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}

void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in, bool binary)
{
  m.resize(n_rows,n_cols);

  if (binary)
  {
    double *buff = new double[n_rows*n_cols];
    in.read((char *)(buff), n_rows*n_cols*sizeof(double));

    int k=0;
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) m(i,j) = buff[k++];
    }

    delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) in >> m(i,j);
    }
  }

}

void read_mat(arma::mat &m, std::istream &in, bool binary)
{
  long n_rows;
  long n_cols;

  read_scalar(n_rows, in, binary);
  read_scalar(n_cols, in, binary);

  read_mat(m, n_rows, n_cols, in, binary);
}

void read_vec(arma::vec &v, std::istream &in, bool binary)
{
  long n_rows;

  read_scalar(n_rows, in, binary);
  read_mat(v, n_rows, 1, in, binary);
}

void read_rowVec(arma::rowvec &v, std::istream &in, bool binary)
{
  long n_cols;

  read_scalar(n_cols, in, binary);
  read_mat(v, 1, n_cols, in, binary);
}

void read_vec_mat(std::vector<arma::mat> &m, std::istream &in, bool binary)
{
  long n_mat;

  read_scalar(n_mat, in, binary);

  m.resize(n_mat);

  for (int k=0;k<n_mat;k++) read_mat(m[k], in, binary);

}


void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision)
{
  if (binary)
  {
    double *buff = new double[n_rows*n_cols];

     int k=0;
     for (int i=0;i<n_rows;i++){
       for (int j=0;j<n_cols;j++) buff[k++] = m(i,j);
     }
     out.write((const char *)(buff), n_rows*n_cols*sizeof(double));

     delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) out << std::setprecision(precision) << m(i,j) << " ";
      out << "\n";
    }
  }

}

void write_mat(const arma::mat &m, std::ostream &out, bool binary, int precision)
{
  long n_rows = m.n_rows;
  long n_cols = m.n_cols;

  write_scalar(n_rows, out, binary);
  if (!binary) out << "\n";
  write_scalar(n_cols, out, binary);
  if (!binary) out << "\n";

  write_mat(m, n_rows, n_cols, out, binary, precision);
}

void write_vec(arma::vec &v, std::ostream &out, bool binary, int precision)
{
  long n_rows = v.size();

  write_scalar(n_rows, out, binary);
  if (!binary) out << "\n";

  write_mat(v, n_rows, 1, out, binary, precision);
}

void write_rowVec(arma::rowvec &v, std::ostream &out, bool binary, int precision)
{
  long n_cols = v.size();

  write_scalar(n_cols, out, binary);
  if (!binary) out << "\n";

  write_mat(v, 1, n_cols, out, binary, precision);
}

void write_vec_mat(std::vector<arma::mat> &m, std::ostream &out, bool binary, int precision)
{
  long n_mat = m.size();

  write_scalar(n_mat, out, binary);
  if (!binary) out << "\n";

  m.resize(n_mat);

  for (int k=0;k<n_mat;k++) write_mat(m[k], out, binary, precision);
}

} // namespace io_

} // namespace as64_
