#ifndef AS64_IO_UTILS_H
#define AS64_IO_UTILS_H

#include <iostream>
#include <fstream>
#include <memory>

#include <armadillo>

namespace as64_
{
  
namespace io_
{
	
/** \brief Reads a 2D matrix from stream \a in.
 *  \details Reads the elements of the matrix row by row.
 *  @param[out] m The 2D matrix
 *  @param[in] n_rows Number of rows of the matrix
 *  @param[in] n_cols Number of columns of the matrix
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in = std::cin, bool binary = false);


/** \brief Reads a 2D matrix from stream \a in.
 *  \details Reads first the number of rows and columns and then the elements of the matrix row by row.
 *  @param[out] m The 2D matrix
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_mat(arma::mat &m, std::istream &in = std::cin, bool binary = false);


/** \brief Reads a column vector from stream \a in.
 *  \details Reads the number of rows and then the elements of the vector.
 *  @param[out] v The column vector
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_vec(arma::vec &v, std::istream &in = std::cin, bool binary = false);


/** \brief Reads a row vector from stream \a in.
 *  \details Reads the number of columns and then the elements of the vector.
 *  @param[out] v The row vector
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_rowVec(arma::rowvec &v, std::istream &in = std::cin, bool binary = false);


/** \brief Reads a vector of 2D matrices from stream \a in.
 *  \details Reads the number of 2D matrices, the number of rows and cols and the elements of each 2D matrix row by row.
 *  @param[out] m cell array where each cell has a 2D matrix
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */ 
void read_vec_mat(std::vector<arma::mat> &m, std::istream &in = std::cin, bool binary = false);


/** \brief Writes a 2D matrix in stream 'fid'.
 *  \details Writes the elements of the matrix row by row.
 *  @param[in] m The 2D matrix
 *  @param[in] n_rows Number of rows of the matrix
 *  @param[in] n_cols Number of columns of the matrix
 *  @param[in] out The output stream (optional, default = 1 for output to screen).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out = std::cout, bool binary = false);


/** \brief Writes a 2D matrix in stream \a out.
 *  \details Writes first the number of rows and columns and then the elements of the matrix row by row.
 *  @param[in] m The 2D matrix
 *  @param[in] out The output stream (optional, default = 1 for output to screen).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void write_mat(const arma::mat &m, std::ostream &out = std::cout, bool binary = false);


/** \brief Writes a column vector in stream \a out.
 *  \details Writes the number of rows and then the elements of the vector.
 *  @param[out] v The column vector.
 *  @param[in] out The output stream (optional, default = std::cout).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void write_vec(arma::vec &v, std::ostream &out = std::cout, bool binary = false);


/** \brief Writes a row vector in stream \a out.
 *  \details Writes the number of columns and then the elements of the vector.
 *  @param[in] v The row vector
 *  @param[in] out The output stream (optional, default = std::cout).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */ 
void write_rowVec(arma::rowvec &v, std::ostream &out = std::cout, bool binary = false);


/** \brief Writes a vector of 2D matrices in stream \a out
 *  \details Writes the number of 2D matrices, and thern the number of rows and cols and the elements of each 2D matrix row by row.
 *  @param[out] m std::vector where each entry is a 2D matrix
 *  @param[in] out The output stream (optional, default = std::cout).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */ 
void write_vec_mat(std::vector<arma::mat> &m, std::ostream &out = std::cout, bool binary = false);

} // namespace io_

} // namespace as64_


#endif // AS64_IO_UTILS_H
