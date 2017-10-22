%% read_rowVec function
%  \brief Reads a row vector from stream 'fid'.
%  \details Reads the number of columns and then the elements of the vector.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[out] v: The row vector
function v = read_rowVec(fid, binary)

    if (nargin < 2), binary = false; end % text format

    if (binary)
        n_cols = fread(fid, 1, 'int64');
    else 
        n_cols = fscanf(fid,'%i', 1);
    end
    
    v = read_mat2(fid, 1, n_cols, binary);

end