%% write_rowVec function
%  \brief Writes a row vector in stream 'fid'.
%  \details Writes the number of columns and then the elements of the vector.
%  @param[in] v: The row vector
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
function write_rowVec(v, fid, binary)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format

    n_cols = length(v);
    
    if (binary)
        fwrite(fid, n_cols, 'int64');
    else     
        fprintf(fid, '%i\n', n_cols);
    end 

    write_mat2(v, 1, n_cols, fid, binary);
end