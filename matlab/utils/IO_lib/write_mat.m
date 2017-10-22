%% write_mat function
%  \brief Writes a 2D matrix in stream 'fid'.
%  \details Writes first the number of rows and columns and then the elements of the matrix row by row.
%  @param[in] m: The 2D matrix
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
function write_mat(m, fid, binary)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format
    
    n_rows = size(m,1);
    n_cols = size(m,2);
    
    if (binary)
        fwrite(fid, n_rows, 'int64');
        fwrite(fid, n_cols, 'int64');
    else     
        fprintf(fid, '%i\n', n_rows);
        fprintf(fid, '%i\n', n_cols);
    end
    
    write_mat2(m, n_rows, n_cols, fid, binary);
    
end
