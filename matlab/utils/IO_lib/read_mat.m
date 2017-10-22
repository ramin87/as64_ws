%% read_mat function
%  \brief Reads a 2D matrix from stream 'fid'.
%  \details Reads first the number of rows and columns and then the elements of the matrix row by row.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[out] m: The 2D matrix
function m = read_mat(fid, binary)

    if (nargin < 2), binary = false; end % text format
    
    if (binary)
        n_rows = fread(fid, 1, 'int64');
        n_cols = fread(fid, 1, 'int64');
    else   
        n_rows = fscanf(fid,'%i', 1);
        n_cols = fscanf(fid,'%i', 1);
    end
    
    m = read_mat2(fid, n_rows, n_cols, binary);

end

