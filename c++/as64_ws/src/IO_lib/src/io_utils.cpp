#include <IO_lib/io_utils.h>

namespace as64_
{
  
namespace io_
{
	
void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in, bool binary)
{
	m.resize(n_rows,n_cols);
	
    if (binary){
        double *buff = new double[n_rows*n_cols];
        in.read((char *)(buff), n_rows*n_cols*sizeof(double));
        
        int k=0;
        for (int i=0;i<n_rows;i++){
			for (int j=0;j<n_cols;j++) m(i,j) = buff[k++];
        }

        delete []buff;  
    }else{
        for (int i=0;i<n_rows;i++){
			for (int j=0;j<n_cols;j++) in >> m(i,j);
        } 
    }

}	
	
void read_mat(arma::mat &m, std::istream &in, bool binary)
{
    long n_rows;
    long n_cols;
    
    if (binary){
        in.read((char *)(&n_rows), sizeof(n_rows));
        in.read((char *)(&n_cols), sizeof(n_cols));  
    }else{
        in >> n_rows;
        in >> n_cols;
    }
    
    read_mat(m, n_rows, n_cols, in, binary);
}	

void read_vec(arma::vec &v, std::istream &in, bool binary)
{
	long n_rows;
	
    if (binary) in.read((char *)(&n_rows), sizeof(n_rows));
    else in >> n_rows;

	read_mat(v, n_rows, 1, in, binary);
}

void read_rowVec(arma::rowvec &v, std::istream &in, bool binary)
{
	long n_cols;
	
    if (binary) in.read((char *)(&n_cols), sizeof(n_cols));
    else in >> n_cols;

	read_mat(v, 1, n_cols, in, binary);
}

void read_vec_mat(std::vector<arma::mat> &m, std::istream &in, bool binary)
{
	long n_mat;
	
    if (binary) in.read((char *)(&n_mat), sizeof(n_mat));  
    else in >> n_mat;

    m.resize(n_mat);
    
    for (int k=0;k<n_mat;k++) read_mat(m[k], in, binary);

}


void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary)
{
    if (binary){
		double *buff = new double[n_rows*n_cols];
		
		int k=0;
        for (int i=0;i<n_rows;i++){
			for (int j=0;j<n_cols;j++) buff[k++] = m(i,j);
        } 
        out.write((const char *)(buff), n_rows*n_cols*sizeof(double));
        
        delete []buff;
    }else{
        for (int i=0;i<n_rows;i++){
			for (int j=0;j<n_cols;j++) out << m(i,j) << " ";
            out << "\n";
        }
    }

}

void write_mat(const arma::mat &m, std::ostream &out, bool binary)
{
    long n_rows = m.n_rows;
    long n_cols = m.n_cols;

    if (binary){
        out.write((const char *)(&n_rows), sizeof(n_rows));
        out.write((const char *)(&n_cols), sizeof(n_cols));
    }else{ 
        out << n_rows << "\n";
        out << n_cols << "\n";
    }

	write_mat(m, n_rows, n_cols, out, binary);
}

void write_vec(arma::vec &v, std::ostream &out, bool binary)
{
    long n_rows = v.size();
    
    if (binary) out.write((const char *)(&n_rows), sizeof(n_rows));
    else out << n_rows << "\n";
    
    write_mat(v, n_rows, 1, out, binary);
}

void write_rowVec(arma::rowvec &v, std::ostream &out, bool binary)
{
	long n_cols = v.size();
    
    if (binary) out.write((const char *)(&n_cols), sizeof(n_cols));
    else out << n_cols << "\n";
    
    write_mat(v, 1, n_cols, out, binary);
}
 
void write_vec_mat(std::vector<arma::mat> &m, std::ostream &out, bool binary)
{
	long n_mat = m.size();
	
    if (binary) out.write((const char *)(&n_mat), sizeof(n_mat));  
    else out << n_mat << "\n";
    
    m.resize(n_mat);
    
    for (int k=0;k<n_mat;k++) write_mat(m[k], out, binary);
}

} // namespace io_
	
} // namespace as64_
