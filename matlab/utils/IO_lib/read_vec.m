%% read_vec function
%  \brief Reads a column vector from stream 'fid'.
%  \details Reads the number of rows and then the elements of the vector.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[out] v: The column vector
function v = read_vec(fid, binary)

    if (nargin < 2), binary = false; end % text format
    
    if (binary) 
        n_rows = fread(fid, 1, 'int64'); 
    else     
        n_rows = fscanf(fid,'%i', 1);
    end
    
    v = read_mat2(fid, n_rows, 1, binary);
    
end