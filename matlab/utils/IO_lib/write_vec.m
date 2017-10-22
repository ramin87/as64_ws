%% write_vec function
%  \brief Writes a column vector in stream 'fid'.
%  \details Writes the number of rows and then the elements of the vector.
%  @param[out] v: The column vector.
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
function write_vec(v, fid, binary)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format

    n_rows = length(v);
    
    if (binary)   
        fwrite(fid, n_rows, 'int64');
    else     
        fprintf(fid, '%i\n', n_rows);
    end
    
    write_mat2(v, n_rows, 1, fid, binary);

end